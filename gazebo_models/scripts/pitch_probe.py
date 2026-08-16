#!/usr/bin/env python3
"""
Record every pitch-shaped signal side by side, with timestamps.

Written for C2-M1.5, whose gate is: *is the number the HUD calls
`ROBOT PITCH` actually the robot's pitch?*  Answering that needs the
candidate signal and the ground truth sampled together, through a whole
traverse -- flat, ascent, platform, descent, flat -- because the two agree
on the ramp and can only disagree off it.

Columns, one row per sample:

===============  =====================================================
`wall`           seconds since this probe started (steady clock)
`sim`            newest ground-truth odometry stamp, i.e. `/clock`
`state`          `/mission/state`, the sequencer's step label
`seg`            `segment=` from `/ramp/status`
`ramp_pitch`     `pitch=` from `/ramp/status` -- the field the HUD shows
`ramp_lat`       `lateral=` (cross-track from the target lane)
`ramp_disp`      `disp=` (drift from where the segment began)
`imu_pitch`      pitch from `/imu`, the actual attitude sensor
`gt_pitch`       pitch from ground-truth odometry orientation
`gt_x` `gt_y`    ground-truth position, to say where the robot was
`hud_pitch`      the number parsed back off `/mission/hud`
===============  =====================================================

`imu_pitch` and `gt_pitch` are two independent measurements of the same
quantity; they are both here so that "the IMU is lying" and "the field is
stale" stay separable. `ramp_pitch` next to them is the whole experiment.

Subscribe-only. It publishes nothing, so it cannot alter the run it is
measuring.

Usage, alongside a normal mission (source setup_env.sh first)::

    ros2 run gazebo_models pitch_probe.py --out /tmp/pitch.csv

Sign convention, fixed by `quat_to_rp` and REP-103: pitch is rotation
about +y (left), so **nose-up is NEGATIVE**. On an 18 deg ramp the robot
reads about -0.314 rad going up.
"""

import argparse
import csv
import sys
import time

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu

from std_msgs.msg import String

# The one definition of the quaternion convention, shared with the env and
# the driver. A second copy here is how the sign silently drifts.
from coco_rl.ramp_env import quat_to_rp

COLUMNS = ('wall', 'sim', 'state', 'seg', 'ramp_pitch', 'ramp_lat',
           'ramp_disp', 'imu_pitch', 'gt_pitch', 'gt_x', 'gt_y', 'hud_pitch')


def field_of(line, key):
    """Pull `key=value` out of a space-separated status line."""
    for token in (line or '').split():
        name, _, value = token.partition('=')
        if name == key:
            return value
    return ''


def hud_pitch_of(block):
    """Read the number off the HUD's ROBOT PITCH row, as text."""
    for row in (block or '').splitlines():
        if row.startswith('ROBOT PITCH'):
            return row[len('ROBOT PITCH'):].strip().replace(' rad', '')
    return ''


class PitchProbe(Node):
    """Samples every pitch source at a fixed rate into one CSV."""

    def __init__(self, path, hz):
        """Subscribe every pitch source and open the CSV."""
        super().__init__('pitch_probe')
        # BEST_EFFORT for /imu: the gz bridge publishes it best-effort and a
        # RELIABLE subscriber would never match, leaving the column blank
        # with no error anywhere -- the silent-blindness trap.
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._imu = None
        self._gt = None
        self._ramp = ''
        self._state = ''
        self._hud = ''

        self.create_subscription(Imu, '/imu', self._on_imu, sensor)
        self.create_subscription(
            Odometry, '/model/coco/odometry', self._on_gt, 10)
        self.create_subscription(String, '/ramp/status', self._on_ramp, 10)
        self.create_subscription(String, '/mission/state', self._on_state, 10)
        self.create_subscription(String, '/mission/hud', self._on_hud, 10)

        self._fh = open(path, 'w', newline='')
        self._csv = csv.writer(self._fh)
        self._csv.writerow(COLUMNS)
        # Steady clock, never the wall clock and never /clock: this probe
        # exists to catch a signal that stops updating, and a time base that
        # can itself stop would hide exactly that.
        self._t0 = time.monotonic()
        self._rows = 0
        self.create_timer(1.0 / hz, self._sample)
        self.get_logger().info(f'recording {path} at {hz:.1f} Hz')

    def _on_imu(self, msg):
        self._imu = msg

    def _on_gt(self, msg):
        self._gt = msg

    def _on_ramp(self, msg):
        self._ramp = msg.data

    def _on_state(self, msg):
        self._state = msg.data

    def _on_hud(self, msg):
        self._hud = msg.data

    def _sample(self):
        imu_pitch = gt_pitch = gt_x = gt_y = sim = ''
        if self._imu is not None:
            q = self._imu.orientation
            imu_pitch = f'{quat_to_rp(q.x, q.y, q.z, q.w)[1]:+.4f}'
        if self._gt is not None:
            p = self._gt.pose.pose
            q = p.orientation
            gt_pitch = f'{quat_to_rp(q.x, q.y, q.z, q.w)[1]:+.4f}'
            gt_x, gt_y = f'{p.position.x:+.3f}', f'{p.position.y:+.3f}'
            stamp = self._gt.header.stamp
            sim = f'{stamp.sec + stamp.nanosec * 1e-9:.3f}'
        self._csv.writerow((
            f'{time.monotonic() - self._t0:.2f}', sim, self._state,
            field_of(self._ramp, 'segment'), field_of(self._ramp, 'pitch'),
            field_of(self._ramp, 'lateral'), field_of(self._ramp, 'disp'),
            imu_pitch, gt_pitch, gt_x, gt_y, hud_pitch_of(self._hud)))
        self._rows += 1
        if self._rows % 100 == 0:
            self._fh.flush()

    def close(self):
        """Flush and close the CSV, so a Ctrl-C keeps the last samples."""
        self._fh.flush()
        self._fh.close()


def main(argv=None):
    """Record until --seconds elapses, or until Ctrl-C."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default='/tmp/pitch_probe.csv',
                        help='CSV to write')
    parser.add_argument('--hz', type=float, default=10.0,
                        help='sample rate')
    parser.add_argument('--seconds', type=float, default=0.0,
                        help='stop after this long; 0 runs until Ctrl-C')
    args, _ = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = PitchProbe(args.out, args.hz)
    deadline = None if args.seconds <= 0 else time.monotonic() + args.seconds
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if deadline is not None and time.monotonic() > deadline:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'wrote {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
