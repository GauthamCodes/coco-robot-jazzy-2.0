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
traverse_demo.py
================
There -> up -> over -> down -> home: the M4 handoff, end to end.

Nav2 drives the flat ground and the RL policy drives the ramp, because
the split is forced by geometry: the lidar plane sits at 0.2135 m and an
18 degree slope intersects it at world x=1.657, so from the flat the ramp
scans as a solid wall. Nav2 will not plan onto its own costmap obstacle,
and after M3 `allow_unknown: false` stops it routing through the unmapped
space behind it either.

This is the sequencer's skeleton. It owns `/mission/mode` — nothing else
should publish it, because cmd_vel_arbiter latches the last value and two
publishers would fight. The arbiter is what actually enforces the
handoff: only the selected source reaches the wheels, and teleop preempts
either of them at any time.

  1. mode=nav    Nav2 to the pre-ramp pose (0.5, lane_y), on flat ground
  2. mode=rl     /ramp/climb   — PPO policy up the slope
  3. mode=rl     /ramp/descend — scripted heading-hold down the far side
  4. mode=nav    Nav2 home

Requires: the sim with traverse:=true, nav.launch.py arbiter:=true, the
arbiter, and ramp_driver with a policy loaded.

Usage:
  ros2 run gazebo_models traverse_demo.py --lane 0.75
"""
import argparse
import math
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

# The map frame is anchored at the spawn pose (see docs/RESULTS.md).
WORLD_TO_MAP_X = 2.0

PRE_RAMP_X = 0.5      # flat ground west of the ramp foot (x=1.0), and the
                      # only clear spot in lane +0.75: the Zone A gate cubes
                      # block x -1.35..-0.85 and the cylinder x -0.4..0.0
HOME = (-2.0, 0.0)


class TraverseDemo(Node):
    """Sequences Nav2 and the ramp driver through one full traverse."""

    def __init__(self):
        super().__init__('traverse_demo')
        self.pose = None
        self.ramp_status = ''
        self.create_subscription(
            Odometry, '/model/coco/odometry', self._odom_cb, 10)
        self.create_subscription(
            String, '/ramp/status', self._ramp_cb, 10)
        # RELIABLE + VOLATILE, matching the panel and the arbiter. Asserted
        # repeatedly rather than latched: the vendored roslib in coco_web
        # cannot express transient-local, so the whole system agreed to
        # re-assert instead.
        self._mode_pub = self.create_publisher(String, '/mission/mode', 10)
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._climb = self.create_client(Trigger, '/ramp/climb')
        self._descend = self.create_client(Trigger, '/ramp/descend')

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def _ramp_cb(self, msg):
        self.ramp_status = msg.data

    def _field(self, key):
        for part in self.ramp_status.split(' '):
            if part.startswith(f'{key}='):
                return part.split('=', 1)[1]
        return None

    def set_mode(self, mode, hold=1.5):
        """Assert a mission mode and hold it long enough to be seen."""
        deadline = time.time() + hold
        while time.time() < deadline and rclpy.ok():
            self._mode_pub.publish(String(data=mode))
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'mode -> {mode}')

    def wait_ready(self, timeout=40.0):
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None and self.ramp_status:
                return True
        self.get_logger().error(
            f'not ready (odom={self.pose is not None}, '
            f'ramp_driver={bool(self.ramp_status)})')
        return False

    def nav_to(self, wx, wy, timeout=240.0):
        """Drive a flat-ground leg with Nav2. Returns True on SUCCEEDED."""
        if not self._nav.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('navigate_to_pose unavailable')
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = wx + WORLD_TO_MAP_X
        goal.pose.pose.position.y = wy
        goal.pose.pose.orientation.w = 1.0

        t0 = time.time()
        send = self._nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=20.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'goal ({wx}, {wy}) rejected')
            return False
        result = handle.get_result_async()
        while not result.done() and rclpy.ok():
            # Keep asserting the mode: if it lapsed the arbiter would stop
            # forwarding Nav2 mid-leg and the robot would simply halt.
            self._mode_pub.publish(String(data='nav'))
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > timeout:
                handle.cancel_goal_async()
                self.get_logger().error('nav leg timed out — cancelled')
                return False
        ok = result.result().status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(
            f'nav to ({wx:.2f}, {wy:.2f}): '
            f'{"SUCCEEDED" if ok else "FAILED"} in {time.time() - t0:.1f}s')
        return ok

    def ramp_segment(self, client, name, timeout=180.0):
        """Call a ramp_driver service and wait for it to finish."""
        if not client.wait_for_service(timeout_sec=20.0):
            self.get_logger().error(f'/ramp/{name} unavailable')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if future.result() is None or not future.result().success:
            self.get_logger().error(f'{name} refused: {future.result()}')
            return False

        t0 = time.time()
        # The driver reports segment=idle with an outcome when it is done.
        while rclpy.ok():
            self._mode_pub.publish(String(data='rl'))
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 < 2.0:
                continue                      # let it pick the segment up
            if self._field('segment') == 'idle':
                break
            if time.time() - t0 > timeout:
                self.get_logger().error(f'{name} timed out')
                return False
        outcome = self._field('outcome')
        self.get_logger().info(
            f'{name}: outcome={outcome} after {time.time() - t0:.1f}s '
            f'({self.ramp_status})')
        return outcome == 'goal'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lane', type=float, default=0.75,
                        help='which target lane to approach in (world y)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = TraverseDemo()
    rc = 1
    try:
        if not node.wait_ready():
            sys.exit(1)
        start = node.pose
        print(f'\nstart (world): ({start[0]:.2f}, {start[1]:.2f})\n')

        steps = [
            ('1. nav to the pre-ramp pose',
             lambda: (node.set_mode('nav'),
                      node.nav_to(PRE_RAMP_X, args.lane))[-1]),
            ('2. RL climb',
             lambda: (node.set_mode('rl'),
                      node.ramp_segment(node._climb, 'climb'))[-1]),
            ('3. scripted descent',
             lambda: node.ramp_segment(node._descend, 'descend')),
            ('4. nav home',
             lambda: (node.set_mode('nav'),
                      node.nav_to(*HOME))[-1]),
        ]
        ok = True
        for label, step in steps:
            print(f'--- {label} ---')
            if not step():
                print(f'FAILED at: {label}')
                ok = False
                break
        node.set_mode('idle')
        end = node.pose
        print(f'\nend (world): ({end[0]:.2f}, {end[1]:.2f})')
        if ok:
            print(f'home to within '
                  f'{math.hypot(end[0] - start[0], end[1] - start[1]):.2f} m')
            print('TRAVERSE COMPLETE')
        rc = 0 if ok else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
