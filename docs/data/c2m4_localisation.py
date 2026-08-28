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
c2m4_localisation.py — how far is the estimated target from the real one.

The C2-M4 instrument. It is the sanity test in C2-M4.0 and the four-colour
benchmark in C2-M4.1; the only difference is how many placements it is
asked for. It is deliberately NOT installed by any `CMakeLists.txt` — it
is an instrument, like `map_audit.py` and `c2m2_live_gate.py`, and it
must never end up on a robot.

THE GROUND-TRUTH BOUNDARY, which is the whole point
---------------------------------------------------
Ground truth enters this file in exactly two places and leaves it in
none:

    read   /model/coco/odometry        world -> base_footprint, gz's own
    read   /world/coco_world/pose/info the target's world position

Both are *read for comparison only*. Neither is published, neither
reaches `target_pose_node`, and the node under test subscribes to
nothing this script writes except `/mission/target_colour`, which is the
operator's colour choice and is an input the real mission also provides.

The robot is *placed* with `gz set_pose`. That is experiment setup, not
information: it decides where the robot stands, exactly as driving it
there would, and it tells perception nothing about where the target is.
The alternative — climbing the ramp under Nav2 and the policy for every
placement — measures the whole stack's ability to park, which is a
different experiment and one that M6 already ran.

What is compared, and in which frame
------------------------------------
Both sides are reduced to **the target cylinder's axis, in
`base_footprint`**:

    estimate     the `bbox.center.position` of /perception/target_pose
    ground truth gz's world position of the target model, mapped through
                 gz's own world->base_footprint for the robot

The gz model origin of a target is its geometric centre — the spawner
places it at `z = rise + height/2` — so ground truth is the axis at
mid-height. The estimate's z is the axis at the *visible blob's*
vertical centroid, which is the same point only while the whole cylinder
is in frame. That is why `dz` is reported separately from the horizontal
error and never folded into a single "accuracy": the horizontal error is
what a grasp lives on, and `dz` carries a projection term the grasp does
not consume at all (the arm takes its height from `TARGET_GRASP_Z`).

Nothing here transforms a world-frame truth against a camera-frame
estimate. Both are in `base_footprint` before any subtraction happens.

Usage
-----
    # C2-M4.0 sanity: one colour, three stand-offs
    python3 c2m4_localisation.py --colours blue --standoffs 0.35 0.50 0.70

    # C2-M4.1 benchmark: the full grid
    python3 c2m4_localisation.py --benchmark --out c2m4_benchmark.csv

Requires a running simulator launched with `traverse:=true` and a
running `target_pose_node`. It starts neither: one Gazebo at a time, and
a script that launches its own would make that rule impossible to keep.
"""

import argparse
import csv
import math
import re
import subprocess
import sys
import time

from coco_config.robot import (RAMP_ANGLE_DEG, RAMP_RUN, SPAWN_Z,
                               TARGET_COLOURS, TARGET_ROW_X, target_by_colour)

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String

from vision_msgs.msg import Detection3DArray

WORLD = 'coco_world'
# Platform top, from the same two constants the launch file builds it
# from. Not a literal: if the grade changes this follows it.
PLATFORM_Z = RAMP_RUN * math.tan(math.radians(RAMP_ANGLE_DEG))

# The stand-offs C2-M4.1 sweeps, as the range from base_footprint to the
# target axis. 0.30 is a little inside the point where the whole cylinder
# is still in frame; 0.90 is past the far end of any approach. The
# approach itself is blind under ~0.13 m (perception's min_range), so
# nothing below 0.25 is useful to characterise.
BENCH_STANDOFFS = (0.30, 0.40, 0.55, 0.70, 0.90)
# Lateral offsets of the ROBOT from the target's lane. The arm's whole
# lateral budget is GRASP_MAX_LATERAL = 0.010 m, so these bracket it:
# on-lane, one budget out, and three budgets out.
BENCH_LATERALS = (0.0, -0.010, 0.030)


# ── gz ───────────────────────────────────────────────────────────────────
def gz_service(service, reqtype, reptype, req, timeout_ms=5000, attempts=5):
    """Call a gz transport service; True iff it replied true.

    Retried for the reason `ramp_env.gz_service` documents: each call
    binds an ephemeral transport node and the round trip occasionally
    overruns. set_pose carries an absolute value, so re-sending it is
    harmless.
    """
    cmd = ['gz', 'service', '-s', service, '--reqtype', reqtype,
           '--reptype', reptype, '--timeout', str(timeout_ms), '--req', req]
    for attempt in range(max(1, attempts)):
        if attempt:
            time.sleep(0.5)
        try:
            out = subprocess.run(cmd, capture_output=True, text=True,
                                 timeout=max(15, timeout_ms / 1000 * 5))
        except FileNotFoundError:
            raise RuntimeError(
                '`gz` not found on PATH — source setup_env.sh first.')
        except subprocess.TimeoutExpired:
            continue
        if 'true' in (out.stdout + out.stderr).lower():
            return True
    return False


_POSE_BLOCK = re.compile(r'name:\s*"([^"]+)"(.*?)orientation\s*\{', re.S)
_FIELD = re.compile(r'([xyz]):\s*(-?[0-9.eE+-]+)')


def gz_world_poses(world=WORLD, timeout=15):
    """Every model's world position, by name, from gz's own pose topic.

    GROUND TRUTH. Read for comparison only — see the module docstring.

    Absent fields are zero in the text encoding gz prints (`position {}`
    for a model at the origin), so each component defaults to 0.0 rather
    than being treated as missing.
    """
    try:
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/world/{world}/pose/info', '-n', '1'],
            capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return {}
    poses = {}
    for name, body in _POSE_BLOCK.findall(out):
        block = re.search(r'position\s*\{(.*?)\}', body, re.S)
        values = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        if block:
            for axis, raw in _FIELD.findall(block.group(1)):
                values[axis] = float(raw)
        poses[name] = (values['x'], values['y'], values['z'])
    return poses


# ── geometry ─────────────────────────────────────────────────────────────
def quat_conj_rotate(q, v):
    """Rotate `v` by the inverse of quaternion `q` = (x, y, z, w).

    This is the world->body half of the ground-truth comparison: given
    the robot's world pose, it puts a world point into base_footprint.
    Written out rather than pulled from tf2 because this script must be
    able to run against a recording, with no graph at all.
    """
    qx, qy, qz, qw = q
    # Inverse of a unit quaternion is its conjugate.
    qx, qy, qz = -qx, -qy, -qz
    ux, uy, uz = qx, qy, qz
    vx, vy, vz = v
    # t = 2 * (u x v)
    tx = 2.0 * (uy * vz - uz * vy)
    ty = 2.0 * (uz * vx - ux * vz)
    tz = 2.0 * (ux * vy - uy * vx)
    # v' = v + qw * t + u x t
    return (vx + qw * tx + (uy * tz - uz * ty),
            vy + qw * ty + (uz * tx - ux * tz),
            vz + qw * tz + (ux * ty - uy * tx))


def world_to_base(point_w, robot_position, robot_orientation):
    """A world point expressed in the robot's base_footprint frame."""
    delta = tuple(p - r for p, r in zip(point_w, robot_position))
    return quat_conj_rotate(robot_orientation, delta)


# ── the harness ──────────────────────────────────────────────────────────
class Harness(Node):
    """Places the robot, reads the estimate, reads the truth, subtracts."""

    def __init__(self):
        super().__init__('c2m4_localisation')
        self.detections = []
        self.status = None
        self.odom = None

        self.create_subscription(
            Detection3DArray, '/perception/target_pose',
            self._on_detection, 10)
        self.create_subscription(
            String, '/perception/target_pose/status',
            self._on_status, 10)
        # Ground truth. Bridged from gz's OdometryPublisher, frame_id
        # world, child_frame_id base_footprint — exactly the transform
        # the comparison needs, and gz's own rather than one derived here.
        self.create_subscription(
            Odometry, '/model/coco/odometry', self._on_odom,
            QoSProfile(depth=10,
                       reliability=ReliabilityPolicy.RELIABLE))
        self._colour_pub = self.create_publisher(
            String, '/mission/target_colour', 10)

    def _on_detection(self, msg):
        self.detections.append(msg)

    def _on_status(self, msg):
        self.status = msg.data

    def _on_odom(self, msg):
        self.odom = msg

    # ── helpers ──────────────────────────────────────────────────────────
    def spin(self, seconds):
        """Pump callbacks for `seconds` of wall time."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def select(self, colour, settle=2.0):
        """Ask the node for `colour` and wait for it to say so."""
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and rclpy.ok():
            self._colour_pub.publish(String(data=colour))
            self.spin(0.3)
            if self.status and f'sel={colour} ' in self.status + ' ':
                self.spin(settle)
                return True
        return False

    def place(self, x, y, yaw=0.0):
        """Teleport the robot to a platform pose and let it settle.

        z is the platform top plus the same SPAWN_Z drop the launch file
        uses on flat ground, so the wheels settle onto the deck rather
        than being started interpenetrating it.
        """
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        ok = gz_service(
            f'/world/{WORLD}/set_pose', 'gz.msgs.Pose', 'gz.msgs.Boolean',
            f'name: "coco", position: {{x: {x}, y: {y}, '
            f'z: {PLATFORM_Z + SPAWN_Z}}}, '
            f'orientation: {{z: {qz}, w: {qw}}}')
        if not ok:
            raise RuntimeError('set_pose failed — is the simulator up?')
        self.spin(2.5)
        return ok

    def sample(self, frames=15, timeout=10.0):
        """Collect up to `frames` Detection3DArray messages."""
        self.detections = []
        deadline = time.monotonic() + timeout
        while (len(self.detections) < frames and rclpy.ok()
               and time.monotonic() < deadline):
            rclpy.spin_once(self, timeout_sec=0.1)
        return list(self.detections)


def median(values):
    ordered = sorted(values)
    n = len(ordered)
    if not n:
        return None
    mid = n // 2
    return (ordered[mid] if n % 2 else (ordered[mid - 1] + ordered[mid]) / 2.0)


def measure(harness, colour, standoff, lateral, frames):
    """One placement: place, sample, compare. Returns a result dict."""
    target = target_by_colour(colour)
    robot_x = TARGET_ROW_X - standoff
    robot_y = target.lane_y + lateral
    harness.place(robot_x, robot_y)
    harness.select(colour)

    messages = harness.sample(frames=frames)
    detected = [m for m in messages if m.detections]

    row = {
        'colour': colour,
        'standoff_cmd': standoff,
        'lateral_cmd': lateral,
        'frames': len(messages),
        'detected': len(detected),
        'status': harness.status or '',
    }
    if not detected or harness.odom is None:
        row['result'] = 'NO_DETECTION' if not detected else 'NO_ODOM'
        return row

    # ── the estimate ─────────────────────────────────────────────────
    est_x = median([m.detections[0].bbox.center.position.x for m in detected])
    est_y = median([m.detections[0].bbox.center.position.y for m in detected])
    est_z = median([m.detections[0].bbox.center.position.z for m in detected])
    last = detected[-1].detections[0]
    frame = detected[-1].header.frame_id
    score = last.results[0].hypothesis.score
    ident = last.id

    # ── the truth, in the same frame ─────────────────────────────────
    poses = gz_world_poses()
    if target.model not in poses:
        row['result'] = 'NO_GROUND_TRUTH'
        return row
    pose = harness.odom.pose.pose
    gt = world_to_base(
        poses[target.model],
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w))

    dx, dy, dz = est_x - gt[0], est_y - gt[1], est_z - gt[2]
    row.update({
        'result': 'OK',
        'frame': frame,
        'id': ident,
        'score': round(score, 4),
        'est_x': round(est_x, 5), 'est_y': round(est_y, 5),
        'est_z': round(est_z, 5),
        'gt_x': round(gt[0], 5), 'gt_y': round(gt[1], 5),
        'gt_z': round(gt[2], 5),
        'dx': round(dx, 5), 'dy': round(dy, 5), 'dz': round(dz, 5),
        'err_norm': round(math.sqrt(dx * dx + dy * dy + dz * dz), 5),
        'err_horizontal': round(math.sqrt(dx * dx + dy * dy), 5),
        'err_vertical': round(abs(dz), 5),
        # Frame-to-frame spread at a fixed pose. A stationary robot and a
        # stationary target should give the same answer every frame; this
        # is what says whether the residual is bias or noise.
        'spread_x': round(max(m.detections[0].bbox.center.position.x
                              for m in detected)
                          - min(m.detections[0].bbox.center.position.x
                                for m in detected), 5),
        'spread_y': round(max(m.detections[0].bbox.center.position.y
                              for m in detected)
                          - min(m.detections[0].bbox.center.position.y
                                for m in detected), 5),
    })
    return row


FIELDS = ('colour', 'standoff_cmd', 'lateral_cmd', 'frames', 'detected',
          'result', 'frame', 'id', 'score', 'est_x', 'est_y', 'est_z',
          'gt_x', 'gt_y', 'gt_z', 'dx', 'dy', 'dz', 'err_norm',
          'err_horizontal', 'err_vertical', 'spread_x', 'spread_y',
          'status')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--colours', nargs='+', default=['blue'],
                        choices=list(TARGET_COLOURS))
    parser.add_argument('--standoffs', nargs='+', type=float,
                        default=[0.35, 0.50, 0.70])
    parser.add_argument('--laterals', nargs='+', type=float, default=[0.0])
    parser.add_argument('--frames', type=int, default=15)
    parser.add_argument('--benchmark', action='store_true',
                        help='the full C2-M4.1 grid: four colours x five '
                             'stand-offs x three laterals')
    parser.add_argument('--out', default=None, help='write a CSV here')
    args = parser.parse_args()

    colours = list(TARGET_COLOURS) if args.benchmark else args.colours
    standoffs = BENCH_STANDOFFS if args.benchmark else args.standoffs
    laterals = BENCH_LATERALS if args.benchmark else args.laterals

    rclpy.init()
    harness = Harness()
    harness.spin(2.0)
    rows = []
    try:
        for colour in colours:
            for standoff in standoffs:
                for lateral in laterals:
                    row = measure(harness, colour, standoff, lateral,
                                  args.frames)
                    rows.append(row)
                    print(f"{row['colour']:>6} standoff={standoff:.2f} "
                          f"lateral={lateral:+.3f} -> {row['result']}", end='')
                    if row['result'] == 'OK':
                        print(f"  dx={row['dx']:+.4f} dy={row['dy']:+.4f} "
                              f"dz={row['dz']:+.4f} "
                              f"|h|={row['err_horizontal']:.4f} "
                              f"|e|={row['err_norm']:.4f}")
                    else:
                        print()
                    sys.stdout.flush()
    finally:
        harness.destroy_node()
        rclpy.shutdown()

    ok = [r for r in rows if r['result'] == 'OK']
    print(f'\n{len(ok)}/{len(rows)} placements measured')
    if ok:
        print('horizontal error: '
              f"min {min(r['err_horizontal'] for r in ok):.4f} "
              f"median {median([r['err_horizontal'] for r in ok]):.4f} "
              f"max {max(r['err_horizontal'] for r in ok):.4f} m")
        print('vertical error:   '
              f"min {min(r['err_vertical'] for r in ok):.4f} "
              f"median {median([r['err_vertical'] for r in ok]):.4f} "
              f"max {max(r['err_vertical'] for r in ok):.4f} m")

    if args.out:
        with open(args.out, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS,
                                    extrasaction='ignore')
            writer.writeheader()
            writer.writerows(rows)
        print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
