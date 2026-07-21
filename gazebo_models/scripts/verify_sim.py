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
verify_sim.py
=============
Health check for a running Coco simulation.

Usage (with full_world_robo.launch.py running):
  python3 verify_sim.py

Subscribes to every load-bearing topic for a few seconds and checks it
against its nominal rate, measured in SIM time from message stamps (so
the check is valid even when the physics real-time factor is unlocked).
Exits 0 if everything passes, 1 otherwise — usable as a smoke test in
scripts and CI images.
"""
import sys
import time

from coco_config.robot import SENSOR_TOPICS, is_best_effort, nominal_hz

from geometry_msgs.msg import TwistStamped  # noqa: F401 (graph parity)
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

WINDOW = 6.0          # wall seconds to listen
BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

# Message type per topic. The nominal rates and QoS live in
# coco_config.robot so this script and coco_config's diagnostics_node
# cannot drift apart when a sensor's <update_rate> changes.
TYPES = {
    '/joint_states': JointState,
    '/scan': LaserScan,
    '/imu': Imu,
    '/camera/image_raw': Image,
    '/diff_drive_controller/odom': Odometry,
    '/model/coco/odometry': Odometry,
}

# Minimum acceptable rate in SIM Hz. These are floors, not targets, and
# they are this script's own policy rather than a property of the robot —
# so they stay here while the nominal rates live in coco_config.robot.
# /joint_states is the loosest because the controller_manager loop is the
# first thing to dip when the machine is loaded, and a slow control loop
# is still a working one.
MIN_HZ = {
    '/joint_states': 20.0,
    '/scan': 8.0,
    '/imu': 40.0,
    '/camera/image_raw': 10.0,
    '/diff_drive_controller/odom': 30.0,
    '/model/coco/odometry': 40.0,
}


def _checks():
    """Build (topic, type, min sim Hz, best_effort); None Hz = presence only."""
    checks = [('/clock', Clock, None, False)]
    for topic in SENSOR_TOPICS:
        floor = MIN_HZ[topic]
        # A floor above the configured rate could never pass; catch that
        # here rather than as a mysterious failure against a healthy sim.
        assert floor <= nominal_hz(topic), (
            f'{topic}: floor {floor} Hz exceeds its nominal '
            f'{nominal_hz(topic)} Hz')
        checks.append((topic, TYPES[topic], floor, is_best_effort(topic)))
    return checks


CHECKS = _checks()


class Probe:
    def __init__(self, node, topic, msg_type, qos):
        self.stamps = []
        self.count = 0

        def cb(msg):
            self.count += 1
            if hasattr(msg, 'header'):
                self.stamps.append(
                    msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            elif hasattr(msg, 'clock'):
                self.stamps.append(
                    msg.clock.sec + msg.clock.nanosec * 1e-9)

        node.create_subscription(msg_type, topic, cb, qos)

    def sim_rate(self):
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / span if span > 0 else 0.0


def run_checks(checks=None, window=WINDOW, node=None, verbose=True):
    """Measure every topic in `checks` and return per-topic results.

    Returns {topic: (passed, rate_or_None, detail)}. Split out of main()
    so the launch_testing integration test can assert on the same checks
    the CLI prints, rather than reimplementing them and drifting.

    Pass an existing `node` to reuse a caller's rclpy context; otherwise
    one is created and torn down here.
    """
    checks = CHECKS if checks is None else checks
    owns_context = node is None
    if owns_context:
        if not rclpy.ok():
            rclpy.init()
        node = Node('verify_sim')

    probes = {
        topic: Probe(node, topic, mtype, BEST_EFFORT if be else 10)
        for topic, mtype, _, be in checks
    }
    end = time.time() + window
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    results = {}
    for topic, _, min_rate, _ in checks:
        probe = probes[topic]
        if probe.count == 0:
            results[topic] = (False, None, 'no messages')
        elif min_rate is None:
            results[topic] = (True, None, f'{probe.count} msgs')
        else:
            rate = probe.sim_rate()
            results[topic] = (rate >= min_rate, rate,
                              f'{rate:6.1f} Hz sim  (min {min_rate})')

    if owns_context:
        node.destroy_node()
        rclpy.shutdown()

    if verbose:
        for topic, (passed, _, detail) in results.items():
            print(f'{"ok  " if passed else "FAIL"}  {topic:35s} {detail}')
    return results


def main():
    results = run_checks()
    failed = sum(1 for passed, _, _ in results.values() if not passed)
    if failed:
        print(f'\n{failed} check(s) FAILED')
        sys.exit(1)
    print('\nall checks passed')


if __name__ == '__main__':
    main()
