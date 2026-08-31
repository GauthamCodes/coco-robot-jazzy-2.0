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
C2-M5.1 localization degradation injector — the C2-M5.0 class-A failure.

Reproduces the injection C2-M5.0 used for ``diverged1`` / ``diverged2``,
written down this time. C2-M5.0 ran it by hand and RESULTS.md describes
it; a recipe that only exists in prose is a recipe that drifts, and
Experiment 2 and Experiment 3 both depend on the two runs being the same
failure.

The recipe, from RESULTS.md "Failure class A":

    hand AMCL an /initialpose 3 m out with a 0.05 m sigma, fired on the
    OBSERVATION that /mission/state reads RETURN_HOME and odometry shows
    three consecutive moving samples

``--preserve-heading`` (the default) is the ``diverged2`` variant: pure
position error with the true yaw intact. RESULTS.md calls it the cleaner
case and the harder one to detect, so it is what the recovery is asked to
handle.

Why this is not cheating
------------------------
The injector reads ``/mission/state`` and ``/amcl_pose`` — the same
topics anything else on the graph can read — and writes ``/initialpose``,
which is the interface RViz's "2D Pose Estimate" button uses. It is an
**operator action**, not a hook into the code under test. Nothing in
``localization_monitor`` or ``mission_states`` knows this node exists,
which is the property that makes the measurement mean anything.

**It publishes to /initialpose and to nothing else.** It adds no
``cmd_vel`` publisher and never commands the robot.

Class B is deliberately NOT reproducible here
---------------------------------------------
``healthy2`` — the C2-M5.0 leg that failed with no injection at all — has
no known trigger, and this file does not pretend to have one. That
failure class remains un-separated and the limitation is recorded in
RESULTS.md rather than papered over with a second injection that would
look like coverage.

Usage
-----
    python3 c2m51_inject.py                      # diverged2 variant
    python3 c2m51_inject.py --dy -3.0 --dyaw 0.4 # diverged1 variant
    python3 c2m51_inject.py --state RETURN_HOME --once
"""

import argparse
import math
import sys

from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import Odometry

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from std_msgs.msg import String

# RESULTS.md, "Failure class A": a 0.05 m sigma. Small on purpose --
# the point of the class is a filter that is CONFIDENT and wrong, which
# is the shape of the M6 run-15 failure family.
INJECT_SIGMA_XY = 0.05
INJECT_SIGMA_YAW = 0.05

# Three consecutive moving samples, as C2-M5.0 fired it. Firing while
# stationary would inject into a robot that has not begun the leg.
MOVING_SAMPLES = 3
MOVING_SPEED = 0.05          # m/s, above odometry noise at rest
MOVING_TURN = 0.05           # rad/s, the same for a spin


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def state_of(line):
    for token in (line or '').split():
        if token.startswith('state='):
            return token[len('state='):] or '--'
    return (line or '--').strip() or '--'


class Injector(Node):

    def __init__(self, args):
        super().__init__('c2m51_inject', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.args = args
        # A comma list, because Experiment 3 has to keep injecting while
        # the mission is in RELOCALIZE as well as RETURN_HOME.
        self.states = tuple(s.strip() for s in args.state.split(',')
                            if s.strip())
        self.amcl = None
        self.state = '--'
        self.moving = 0
        self.fired = 0

        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, latched)
        self.create_subscription(
            String, '/mission/state',
            lambda m: setattr(self, 'state', state_of(m.data)), 10)
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self._on_odom, 10)

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().warn(
            f'injector armed: on {args.state}, dy={args.dy:+.2f} '
            f'dyaw={args.dyaw:+.2f}, sigma={INJECT_SIGMA_XY}')

    def _on_amcl(self, msg):
        p = msg.pose.pose
        self.amcl = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_odom(self, msg):
        # Angular counts as moving. RELOCALIZE spins in place, so a
        # linear-only test would read the recovery rotation as stopped --
        # which matters for --repeat, where the injector has to keep
        # winning while the robot turns.
        speed = abs(msg.twist.twist.linear.x)
        turn = abs(msg.twist.twist.angular.z)
        self.moving = (self.moving + 1
                       if speed > MOVING_SPEED or turn > MOVING_TURN
                       else 0)

    def _tick(self):
        if self.args.once and self.fired:
            return
        if self.state not in self.states:
            return
        if self.moving < MOVING_SAMPLES or self.amcl is None:
            return

        x, y, yaw = self.amcl
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x + self.args.dx
        msg.pose.pose.position.y = y + self.args.dy
        new_yaw = yaw + self.args.dyaw
        msg.pose.pose.orientation.z = math.sin(new_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(new_yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = INJECT_SIGMA_XY ** 2      # xx
        cov[7] = INJECT_SIGMA_XY ** 2      # yy
        cov[35] = INJECT_SIGMA_YAW ** 2    # yaw-yaw
        msg.pose.covariance = cov
        self.pub.publish(msg)
        self.fired += 1
        self.get_logger().error(
            f'INJECTED: ({x:.2f}, {y:.2f}) -> '
            f'({msg.pose.pose.position.x:.2f}, '
            f'{msg.pose.pose.position.y:.2f}) in {self.state}')
        if self.args.once:
            self.get_logger().warn('one-shot: injector will not fire again')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--state', default='RETURN_HOME',
                    help='mission state(s) to fire in, comma separated')
    ap.add_argument('--dx', type=float, default=0.0)
    ap.add_argument('--dy', type=float, default=-3.0)
    ap.add_argument('--dyaw', type=float, default=0.0,
                    help='0.0 is the diverged2 variant: heading preserved')
    ap.add_argument('--once', action='store_true', default=True)
    ap.add_argument('--repeat', dest='once', action='store_false',
                    help='keep re-injecting; used by the failed-recovery '
                         'experiment, where the point is that recovery '
                         'cannot win')
    args = ap.parse_args()

    rclpy.init()
    node = Injector(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.get_logger().info(f'injected {node.fired} time(s)')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
