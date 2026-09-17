#!/usr/bin/env python3
"""C2-NAV.44: record /amcl_pose beside ground truth, to separate two causes.

The executive's pre-ramp gate reads GROUND TRUTH; Nav2's goal checker reads
the pose estimate it is steering by. When they disagree, the difference is
either (a) the controller stopping inside its own xy_goal_tolerance by its
estimate, or (b) that estimate being wrong. Nothing in the existing
instruments records the estimate as a time series, so the two cannot be
separated after the fact. This records both, at 10 Hz, and nothing else.

Ground truth is SCORING ONLY; no node reads this file.

    python3 -P c2n44_poserec.py --out poses.csv [--duration S]
"""
import argparse
import csv
import math
import sys
import threading
import time

WORLD_TO_MAP_X = 2.0     # mission_states.WORLD_TO_MAP_X


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--duration', type=float, default=1800.0)
    ap.add_argument('--hz', type=float, default=10.0)
    args = ap.parse_args(argv)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    rclpy.init()
    node = Node('c2n44_poserec',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = {'amcl': None, 'amcl_t': None, 'gt': None, 'mission': ''}
    lock = threading.Lock()

    latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    def on_amcl(m):
        with lock:
            state['amcl'] = (m.pose.pose.position.x, m.pose.pose.position.y,
                             yaw_of(m.pose.pose.orientation))
            state['amcl_t'] = node.get_clock().now().nanoseconds * 1e-9
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', on_amcl, latched)

    def on_gt(m):
        with lock:
            state['gt'] = (m.pose.pose.position.x, m.pose.pose.position.y,
                           yaw_of(m.pose.pose.orientation))
    node.create_subscription(Odometry, '/model/coco/odometry', on_gt, 20)

    def on_state(m):
        with lock:
            state['mission'] = next((t.split('=', 1)[1] for t in m.data.split()
                                     if t.startswith('state=')), '')
    node.create_subscription(String, '/mission/state', on_state, 10)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    cols = ['t_sim', 'mission_state', 'amcl_map_x', 'amcl_map_y', 'amcl_yaw',
            'amcl_age_s', 'gt_world_x', 'gt_world_y', 'gt_yaw',
            'amcl_world_x', 'err_m']
    end = time.monotonic() + args.duration
    with open(args.out, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        while time.monotonic() < end:
            time.sleep(1.0 / args.hz)
            with lock:
                amcl, gt, ms_, at = state['amcl'], state['gt'], state['mission'], state['amcl_t']
            if gt is None:
                continue
            t = node.get_clock().now().nanoseconds * 1e-9
            row = {'t_sim': round(t, 3), 'mission_state': ms_,
                   'gt_world_x': round(gt[0], 4), 'gt_world_y': round(gt[1], 4),
                   'gt_yaw': round(gt[2], 4)}
            if amcl is not None:
                wx = amcl[0] - WORLD_TO_MAP_X
                row.update({'amcl_map_x': round(amcl[0], 4),
                            'amcl_map_y': round(amcl[1], 4),
                            'amcl_yaw': round(amcl[2], 4),
                            'amcl_age_s': round(t - at, 2) if at else '',
                            'amcl_world_x': round(wx, 4),
                            'err_m': round(math.hypot(wx - gt[0], amcl[1] - gt[1]), 4)})
            w.writerow(row)
            fh.flush()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
