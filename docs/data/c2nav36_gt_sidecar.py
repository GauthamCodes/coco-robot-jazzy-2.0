#!/usr/bin/env python3
"""C2-NAV.36 -- ground-truth sidecar for the AmclNode diagnostic hook in
coco_nav_diag.

Investigation-support tooling only.  Not wired into any launch file, changes
no AMCL/nav2 behaviour.  Run it alongside a `coco_nav_diag` capture (by hand,
or by gazebo_models/scripts/nav_tour_run.sh when an experiment enables the
AMCL diagnostic).

WHY THIS EXISTS
----------------
The AMCL-internal diagnostic hook must stay AMCL-internal (no GT lookup, no
GT wait, no second TF query inside the scan callback -- CODEX_REVIEW.md
SS35.5).  Ground truth is instead captured by a wholly separate process here
and joined OFFLINE by timestamp (`c2nav36_diag.py join`).

It records the Odometry message's own `header.stamp` (ACQUISITION time, the
same /clock domain as amcl_diag's scan stamps) as the correlation key, plus
the receipt time separately for reference -- nav_bench.py stores receipt
time, which is not good enough for a bracket-interpolated join.

INTEGRITY (C2-NAV.39)
----------------------
On SIGINT or SIGTERM the sidecar closes the CSV and writes
`<out>.meta.json`: rows written and dropped, first/last stamp, stamp
regressions and duplicates in arrival order, every (frame_id,
child_frame_id) pair seen, and `clean_shutdown`.  A capture with no meta
file was killed (or predates C2-NAV.39) and its drop count is unknown;
`c2nav36_diag.py join` reports it as such rather than trusting it.

Usage:

    python3 -P docs/data/c2nav36_gt_sidecar.py --out /path/to/gt.csv \\
        [--max-rows 200000]
"""

import argparse
import csv
import json
import sys

CSV_HEADER = [
    'stamp_sec', 'stamp_nanosec',       # message header.stamp (acquisition)
    'recv_wall_sec', 'recv_wall_nanosec',  # this node's own now() at receipt
    'frame_id', 'child_frame_id',
    'x', 'y', 'z', 'yaw', 'qx', 'qy', 'qz', 'qw',
]
META_SCHEMA_VERSION = 1


def format_row(
    stamp_sec, stamp_nanosec, recv_wall_sec, recv_wall_nanosec,
    frame_id, child_frame_id, x, y, z, yaw, qx, qy, qz, qw,
):
    """Pure, rclpy-free row formatter -- one nav_msgs/Odometry sample."""
    return [
        stamp_sec, stamp_nanosec, recv_wall_sec, recv_wall_nanosec,
        frame_id, child_frame_id,
        f'{x:.9f}', f'{y:.9f}', f'{z:.9f}', f'{yaw:.9f}',
        f'{qx:.9f}', f'{qy:.9f}', f'{qz:.9f}', f'{qw:.9f}',
    ]


def quaternion_to_yaw(qx, qy, qz, qw):
    """Standard planar yaw extraction, same convention tf2::getYaw uses."""
    import math
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def meta_path(out_path):
    return out_path + '.meta.json'


class CaptureStats:
    """Pure, rclpy-free accounting behind <out>.meta.json."""

    def __init__(self, topic, max_rows):
        self.topic = topic
        self.max_rows = max_rows
        self.rows_written = 0
        self.rows_dropped = 0
        self.first_stamp = None
        self.last_stamp = None
        self.stamp_regressions = 0
        self.duplicate_stamps = 0
        self.frame_pairs = set()

    def accept(self):
        if self.rows_written >= self.max_rows:
            self.rows_dropped += 1
            return False
        return True

    def observe(self, sec, nanosec, frame_id, child_frame_id):
        stamp = (int(sec), int(nanosec))
        if self.last_stamp is not None:
            if stamp < self.last_stamp:
                self.stamp_regressions += 1
            elif stamp == self.last_stamp:
                self.duplicate_stamps += 1
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.last_stamp = stamp
        self.frame_pairs.add((frame_id, child_frame_id))
        self.rows_written += 1

    def to_dict(self, clean_shutdown):
        return {
            'schema_version': META_SCHEMA_VERSION,
            'topic': self.topic,
            'max_rows': self.max_rows,
            'rows_written': self.rows_written,
            'rows_dropped': self.rows_dropped,
            'first_stamp': list(self.first_stamp) if self.first_stamp else None,
            'last_stamp': list(self.last_stamp) if self.last_stamp else None,
            'stamp_regressions': self.stamp_regressions,
            'duplicate_stamps': self.duplicate_stamps,
            'frame_pairs': sorted([list(p) for p in self.frame_pairs]),
            'clean_shutdown': bool(clean_shutdown),
        }


def write_meta(out_path, stats, clean_shutdown):
    with open(meta_path(out_path), 'w') as f:
        json.dump(stats.to_dict(clean_shutdown), f, indent=1)


class GtSidecarNode:
    """Thin wrapper; rclpy is imported lazily so the pure helpers above stay
    importable by `c2nav36_diag.py selftest` without a ROS runtime."""

    def __init__(self, out_path, max_rows, topic='/model/coco/odometry'):
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node

        self._out_path = out_path
        self._stats = CaptureStats(topic, max_rows)
        self._csv_file = open(out_path, 'w', newline='')  # noqa: SIM115
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(CSV_HEADER)

        class _Node(Node):
            def __init__(self_inner):
                super().__init__('c2nav36_gt_sidecar')
                self_inner.create_subscription(Odometry, topic, self._gt_cb, 20)

        self._rclpy = rclpy
        self._node = _Node()

    def _gt_cb(self, msg):
        if not self._stats.accept():
            return
        now = self._node.get_clock().now().seconds_nanoseconds()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self._writer.writerow(
            format_row(
                msg.header.stamp.sec, msg.header.stamp.nanosec,
                now[0], now[1],
                msg.header.frame_id, msg.child_frame_id,
                p.x, p.y, p.z, yaw, q.x, q.y, q.z, q.w))
        self._stats.observe(msg.header.stamp.sec, msg.header.stamp.nanosec,
                            msg.header.frame_id, msg.child_frame_id)
        if self._stats.rows_written % 500 == 0:
            self._csv_file.flush()

    def spin(self):
        from rclpy.executors import ExternalShutdownException
        orderly = False
        try:
            self._rclpy.spin(self._node)
            orderly = True
        except (KeyboardInterrupt, ExternalShutdownException):
            orderly = True
        finally:
            self._csv_file.flush()
            self._csv_file.close()
            write_meta(self._out_path, self._stats, orderly)
            print(
                f'c2nav36_gt_sidecar: wrote {self._stats.rows_written} rows, '
                f'dropped {self._stats.rows_dropped} (max_rows reached), '
                f'meta {meta_path(self._out_path)}',
                file=sys.stderr)
            self._node.destroy_node()
            self._rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True, help='output CSV path')
    ap.add_argument(
        '--max-rows', type=int, default=200000,
        help='bound on rows written before newer samples are dropped and '
             'counted (default 200000; avoids an unbounded log on a long run)')
    ap.add_argument(
        '--topic', default='/model/coco/odometry',
        help='ground-truth Odometry topic (default: the gz OdometryPublisher, '
             'NOT AMCL output and NOT commanded velocity)')
    args = ap.parse_args()

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    # SIGTERM (ros_clean.sh, the tour runner) must reach the orderly path
    # that writes the meta file, not kill the process mid-row.
    rclpy.init(signal_handler_options=SignalHandlerOptions.ALL)
    node = GtSidecarNode(args.out, args.max_rows, args.topic)
    node.spin()


if __name__ == '__main__':
    main()
