#!/usr/bin/env python3
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


def main():
    rclpy.init()
    node = Node('verify_sim')
    probes = {
        topic: Probe(node, topic, mtype, BEST_EFFORT if be else 10)
        for topic, mtype, _, be in CHECKS
    }
    end = time.time() + WINDOW
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    failed = 0
    for topic, _, min_rate, _ in CHECKS:
        p = probes[topic]
        if p.count == 0:
            print(f'FAIL  {topic:35s} no messages')
            failed += 1
        elif min_rate is None:
            print(f'ok    {topic:35s} {p.count} msgs')
        else:
            rate = p.sim_rate()
            status = 'ok  ' if rate >= min_rate else 'FAIL'
            failed += rate < min_rate
            print(f'{status}  {topic:35s} {rate:6.1f} Hz sim'
                  f'  (min {min_rate})')

    node.destroy_node()
    rclpy.shutdown()
    if failed:
        print(f'\n{failed} check(s) FAILED')
        sys.exit(1)
    print('\nall checks passed')


if __name__ == '__main__':
    main()
