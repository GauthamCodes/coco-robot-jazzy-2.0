#!/usr/bin/env python3
"""Measure a live COCO ROS graph: nodes, topic rates, /clock RTF, TF frames.

usage: graph_probe.py DURATION_S [topic ...]   -> JSON on stdout

Subscriptions are best effort (matches reliable and best-effort
publishers alike). Real-time factor = sim-time delta on /clock over the
wall-time delta between the first and last /clock message received.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
import tf2_ros

DEFAULT = ['/clock', '/scan', '/model/coco/odometry',
           '/diff_drive_controller/odom', '/diff_drive_controller/cmd_vel',
           '/camera/image_raw', '/camera/depth/image_raw', '/imu',
           '/joint_states', '/tf', '/robot_description', '/map',
           '/amcl_pose', '/local_costmap/costmap', '/plan',
           '/mission/state', '/cmd_vel_teleop', '/cmd_vel_nav']
PAIRS = [('odom', 'base_footprint'), ('map', 'odom'),
         ('base_footprint', 'base_link'), ('base_link', 'lidar_link'),
         ('base_link', 'camera_link')]


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    topics = sys.argv[2:] or DEFAULT
    rclpy.init()
    n = Node('coco_graph_probe')
    t_end = time.monotonic() + 4.0      # discovery
    while time.monotonic() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
    types = dict(n.get_topic_names_and_types())
    counts, clock = {}, {'first': None, 'last': None}
    subs = []
    for t in topics:
        if t not in types:
            counts[t] = None
            continue
        try:
            msg_t = get_message(types[t][0])
        except Exception as exc:  # noqa: BLE001
            counts[t] = f'type error: {exc}'
            continue
        counts[t] = 0

        def cb(msg, t=t):
            counts[t] += 1
            if t == '/clock':
                s = msg.clock.sec + msg.clock.nanosec * 1e-9
                w = time.monotonic()
                if clock['first'] is None:
                    clock['first'] = (s, w)
                clock['last'] = (s, w)
        subs.append(n.create_subscription(msg_t, t, cb,
                                          qos_profile_sensor_data))
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.05)
    wall = time.monotonic() - t0
    rtf = None
    if clock['first'] and clock['last'] and clock['last'][1] > clock['first'][1]:
        rtf = round((clock['last'][0] - clock['first'][0]) /
                    (clock['last'][1] - clock['first'][1]), 3)
    tf_ok = {}
    for a, b in PAIRS:
        try:
            tf_ok[f'{a}->{b}'] = buf.can_transform(a, b, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001
            tf_ok[f'{a}->{b}'] = f'error: {exc}'
    frames = sorted(ln.split(':')[0] for ln in
                    buf.all_frames_as_yaml().splitlines()
                    if ln and not ln.startswith(' '))
    out = {
        'duration_s': round(wall, 1),
        'nodes': sorted(f'{ns.rstrip("/")}/{nm}' for nm, ns in
                        n.get_node_names_and_namespaces()),
        'topic_count': len(types),
        'rates_hz': {t: (round(c / wall, 2) if isinstance(c, int) else c)
                     for t, c in counts.items()},
        'publishers': {t: n.count_publishers(t) for t in topics},
        'rtf': rtf,
        'sim_time_s': clock['last'][0] if clock['last'] else None,
        'tf_can_transform': tf_ok,
        'tf_frames': frames,
    }
    print(json.dumps(out, indent=1))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
