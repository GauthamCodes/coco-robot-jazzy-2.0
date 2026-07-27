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
magnet_release.py
=================
Detach every magnet target once, at startup, then exit.

**Why this has to exist.** The gz DetachableJoint plugin
(coco_robo2.xacro) attaches its child model the instant that model
appears, not when commanded — there is no SDF option to start detached.
So the moment `full_world_robo.launch.py traverse:=true` spawns the four
platform targets, all four are welded to the gripper palm, six metres
away.

**What that breaks, measured.** Translation still works: commanded 0.3
m/s the robot covered 1.0 m in 4 s, dragging the constraint. Rotation
does not. Commanding -0.3 rad/s for six seconds:

    welded:    yaw  0.000 -> 0.000   (nothing at all)
    detached:  yaw  0.000 -> -1.342

A fixed joint constrains orientation as well as position, and yawing the
palm would have to swing four bodies through an arc six metres out. The
robot simply cannot turn. This is what made map_drive.py abort on its
first waypoint, which is a ~90 degree turn in place, with nothing in the
log but a timeout — the robot was driving, it just could not point
anywhere new.

Nothing about that is obvious from a stationary robot, so it is worth
stating plainly: **if the base will not turn in a traverse world, look
here before you look at the controller.**

Usage:
  ros2 run gazebo_models magnet_release.py --models target_red target_blue

The state topic is edge-triggered — it publishes only on transitions — so
a subscriber has to exist before the command is sent, and silence means
either "already detached" or "nobody is listening". Since the targets are
provably attached at spawn, a detach here must produce a transition, and
not seeing one within the timeout is a real failure worth reporting.
"""
import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Empty, String

DEFAULT_MODELS = ['target_red', 'target_green', 'target_blue', 'target_yellow']
RESEND_PERIOD = 0.5      # s between detach commands
DEFAULT_TIMEOUT = 45.0   # s to get every model detached before giving up


class MagnetRelease(Node):
    """Publishes detach to every named magnet until each confirms."""

    def __init__(self, models, timeout):
        super().__init__('magnet_release')
        self.models = list(models)
        self.timeout = timeout
        self.detached = set()
        self._pubs = {}
        for name in self.models:
            self._pubs[name] = self.create_publisher(
                Empty, f'/magnet/{name}/detach', 10)
            self.create_subscription(
                String, f'/magnet/{name}/state',
                lambda msg, m=name: self._on_state(m, msg), 10)

    def _on_state(self, model, msg):
        if msg.data.strip().lower() == 'detached':
            if model not in self.detached:
                self.get_logger().info(f'{model} detached')
            self.detached.add(model)

    def release(self):
        """Resend until every model reports detached. True if all did."""
        deadline = time.time() + self.timeout
        next_send = 0.0
        while time.time() < deadline and rclpy.ok():
            if len(self.detached) == len(self.models):
                self.get_logger().info(
                    f'all {len(self.models)} targets released')
                return True
            if time.time() >= next_send:
                # Resent rather than sent once: the ros_gz bridge and the
                # target models come up asynchronously with this node, so
                # the first command can land before anyone is listening.
                for name in self.models:
                    if name not in self.detached:
                        self._pubs[name].publish(Empty())
                next_send = time.time() + RESEND_PERIOD
            rclpy.spin_once(self, timeout_sec=0.05)

        missing = [m for m in self.models if m not in self.detached]
        self.get_logger().error(
            f'still attached after {self.timeout:.0f}s: {", ".join(missing)}. '
            f'The base will not be able to turn. Check that the ros_gz_bridge '
            f'has the /magnet entries and that the robot was spawned from the '
            f'xacro carrying the DetachableJoint plugins.')
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models', nargs='+', default=DEFAULT_MODELS)
    parser.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = MagnetRelease(args.models, args.timeout)
    ok = False
    try:
        ok = node.release()
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy invalidates the context from its own SIGINT handler before
        # Python sees the signal, so Ctrl-C arrives as the latter.
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
