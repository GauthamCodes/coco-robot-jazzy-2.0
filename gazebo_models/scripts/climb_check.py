#!/usr/bin/env python3
# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
climb_check.py
==============
Headless empirical proof that the robot can drive up the ramp — the
"teleop climb" from the verification plan, scripted so it can run in CI or
verify_all.sh without a keyboard or a GUI.

With full_world_robo.launch.py already running, this drives the robot
straight forward at a fixed speed and watches ground-truth odometry and the
IMU. It declares success only if the robot both:

  1. reaches the ramp **summit** in horizontal progress
     (RAMP_SUMMIT_X - spawn x, from coco_config.robot), and
  2. pitched nose-up past a threshold on the way — i.e. it went *up an
     incline*, rather than the ramp having simply vanished.

Against the old unclimbable mesh the robot stalls at the ~66 deg face near
the foot (progress ~3 m, never ~5.5 m) and this exits non-zero — which is
exactly the "before" evidence. Against the parametric wedge it climbs and
this exits zero.

Usage (sim running, env sourced):
  ros2 run gazebo_models climb_check.py
  ros2 run gazebo_models climb_check.py --duration 60 --speed 0.6
"""
import argparse
import math
import sys
import time

from coco_config.robot import RAMP_ANGLE_DEG, RAMP_SUMMIT_X, SPAWN_XY

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


def _pitch(q):
    """Pitch (rad) from a quaternion, clamped at the asin singularity."""
    return math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))


def run(duration, speed, node=None):
    owns = node is None
    if owns:
        if not rclpy.ok():
            rclpy.init()
        node = Node('climb_check')

    state = {'gt': None, 'odom': None, 'imu': None}
    node.create_subscription(Odometry, '/model/coco/odometry',
                             lambda m: state.update(gt=m), 10)
    node.create_subscription(Odometry, '/diff_drive_controller/odom',
                             lambda m: state.update(odom=m), 10)
    node.create_subscription(Imu, '/imu',
                             lambda m: state.update(imu=m), BEST_EFFORT)
    cmd = node.create_publisher(TwistStamped,
                                '/diff_drive_controller/cmd_vel', 10)

    # Wait for pose + attitude to arrive.
    deadline = time.time() + 15.0
    while (state['imu'] is None
           or (state['gt'] is None and state['odom'] is None)):
        if time.time() > deadline:
            print('FAIL climb_check: no odom/imu after 15 s — is the sim '
                  'running (full_world_robo.launch.py)?')
            return 1
        rclpy.spin_once(node, timeout_sec=0.1)

    def pose():
        return state['gt'] if state['gt'] is not None else state['odom']

    x0 = pose().pose.pose.position.x
    goal = RAMP_SUMMIT_X - SPAWN_XY[0]        # horizontal progress to summit
    min_pitch = math.radians(RAMP_ANGLE_DEG) * 0.4   # proof it was on a grade

    max_prog = 0.0
    peak_pitch = 0.0
    reached = False

    def drive(v):
        m = TwistStamped()
        m.header.stamp = node.get_clock().now().to_msg()
        m.twist.linear.x = float(v)
        cmd.publish(m)

    end = time.time() + duration
    while time.time() < end:
        drive(speed)
        rclpy.spin_once(node, timeout_sec=0.05)
        prog = pose().pose.pose.position.x - x0
        pitch = abs(_pitch(state['imu'].orientation))
        max_prog = max(max_prog, prog)
        peak_pitch = max(peak_pitch, pitch)
        if max_prog >= goal - 0.3 and peak_pitch >= min_pitch:
            reached = True
            break

    drive(0.0)
    rclpy.spin_once(node, timeout_sec=0.05)
    if owns:
        node.destroy_node()

    print(f'progress {max_prog:.2f} m / goal {goal:.2f} m   '
          f'peak pitch {math.degrees(peak_pitch):.1f} deg '
          f'(>= {math.degrees(min_pitch):.1f} needed)')
    if reached:
        print(f'PASS climb_check: robot reached the {RAMP_ANGLE_DEG} deg '
              f'ramp summit under forward drive.')
        return 0
    print('FAIL climb_check: robot did not reach the summit — it either '
          'stalled short (unclimbable) or never pitched onto a grade.')
    return 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=45.0,
                    help='max seconds to drive before giving up')
    ap.add_argument('--speed', type=float, default=0.6,
                    help='forward speed (m/s); matches the RL MAX_LIN')
    args = ap.parse_args()
    try:
        sys.exit(run(args.duration, args.speed))
    except KeyboardInterrupt:
        sys.exit(130)


if __name__ == '__main__':
    main()
