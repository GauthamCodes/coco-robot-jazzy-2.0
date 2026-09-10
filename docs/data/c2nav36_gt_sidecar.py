#!/usr/bin/env python3
"""C2-NAV.36 -- ground-truth sidecar for the AmclNode diagnostic hook in
coco_nav_diag.

Investigation-support tooling only.  Not wired into any launch file, not
started by anything automatically, changes no AMCL/nav2 behaviour.  Run it
by hand, alongside a `coco_nav_diag` `amcl_diag` capture, only for a
dedicated future C2-NAV.36 experiment -- NOT run this session.

WHY THIS EXISTS
----------------
`docs/agents/HANDOFF.md` C2-NAV.35 SS8 and `docs/agents/CODEX_REVIEW.md`
SS35.5 converge independently on the same requirement: the AMCL-internal
diagnostic hook must stay AMCL-internal (no GT lookup, no GT wait, no
second TF query inside the scan callback -- "Do not inject GT into
localization or wait for GT inside AMCL", CODEX_REVIEW.md SS35.5).
Ground truth is instead captured by a wholly separate process here and
joined OFFLINE by timestamp, exactly matching the historical
`docs/data/c2nav34_odom.py` / `nav_bench.py` architecture.

THE ONE THING THIS FIXES THAT nav_bench.py DOES NOT
------------------------------------------------------
`CODEX_REVIEW.md` SS35.1 flags that `nav_bench.py` stores ground truth
using `self.now()` (receipt/callback time), not the `Odometry` message's
own `header.stamp` (acquisition time).  This sidecar records
`header.stamp` (the ACQUISITION stamp -- the same clock domain
`amcl_diag`'s `scan_stamp_sec/nanosec` and `AmclNode::now()` use, since
both come from `/clock` in Gazebo sim) as the correlation key, plus the
wall/receipt time separately for reference.  This is what makes a
bracket-interpolated, non-nearest-receipt-time offline join
(`c2nav36_diag.py join`) possible at all.

Usage:

    python3 -P docs/data/c2nav36_gt_sidecar.py --out /path/to/gt.csv \\
        [--max-rows 200000]

Ctrl-C to stop; the node prints wrote/dropped counts on exit.
"""

import argparse
import csv
import sys

CSV_HEADER = [
    'stamp_sec', 'stamp_nanosec',       # message header.stamp (acquisition)
    'recv_wall_sec', 'recv_wall_nanosec',  # this node's own now() at receipt
    'frame_id', 'child_frame_id',
    'x', 'y', 'z', 'yaw', 'qx', 'qy', 'qz', 'qw',
]


def format_row(
    stamp_sec, stamp_nanosec, recv_wall_sec, recv_wall_nanosec,
    frame_id, child_frame_id, x, y, z, yaw, qx, qy, qz, qw,
):
    """Pure, rclpy-free row formatter -- one nav_msgs/Odometry sample.

    Kept separate from the rclpy callback (GtSidecarNode._gt_cb below) so
    `c2nav36_diag.py selftest` can exercise the exact on-disk format
    without a ROS runtime, mirroring coco_nav_diag's DiagRecorder split
    between pure serialization and the ROS-coupled call site.
    """
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


class GtSidecarNode:
    """Thin wrapper; constructed lazily so importing this module for
    `format_row`/`quaternion_to_yaw` (selftest) never requires rclpy to be
    importable in whatever environment runs the offline analysis."""

    def __init__(self, out_path, max_rows, topic='/model/coco/odometry'):
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node

        self._rows_written = 0
        self._rows_dropped = 0
        self._max_rows = max_rows
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
        if self._rows_written >= self._max_rows:
            self._rows_dropped += 1
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
        self._rows_written += 1
        if self._rows_written % 500 == 0:
            self._csv_file.flush()

    def spin(self):
        try:
            self._rclpy.spin(self._node)
        except KeyboardInterrupt:
            pass
        finally:
            self._csv_file.flush()
            self._csv_file.close()
            print(
                f'c2nav36_gt_sidecar: wrote {self._rows_written} rows, '
                f'dropped {self._rows_dropped} (max_rows reached)',
                file=sys.stderr)
            self._node.destroy_node()
            self._rclpy.shutdown()


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
    rclpy.init()
    node = GtSidecarNode(args.out, args.max_rows, args.topic)
    node.spin()


if __name__ == '__main__':
    main()
