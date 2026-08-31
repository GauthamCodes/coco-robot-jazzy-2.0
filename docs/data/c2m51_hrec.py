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
C2-M5.1 health recorder — what the monitor said, and what the mission did.

Deliberately much smaller than ``c2m5_locrec.py``, and NOT a replacement
for it. C2-M5.0's recorder computes the scan-vs-map score itself, from a
map YAML, because at that point nothing on the robot published it. C2-M5.1
put that computation in a node, so this only has to **record what the node
said** and line it up against the mission state and the wheel commands.

``c2m5_locrec.py`` is untouched and stays the way the five C2-M5.0 runs
were recorded. Changing its schema would make the committed CSVs and any
new one incomparable, which is the one thing the evidence cannot afford.

Ground truth
------------
``gt_x``/``gt_y`` are recorded and are **scoring only**. Nothing the
monitor or the executive reads comes from them; they exist so a run can
be scored afterwards for how wrong the robot actually was, which is the
only way to tell a true positive from a false one. The health columns
beside them are exactly the bytes the monitor published.

Usage
-----
    python3 c2m51_hrec.py --out run.csv --tag exp1 [--hz 10]
    python3 c2m51_hrec.py --summarise run.csv
"""

import argparse
import csv
import math
import sys
import time

from geometry_msgs.msg import TwistStamped

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from std_msgs.msg import String

COLUMNS = [
    't_wall', 't_sim', 'tag', 'state',
    # exactly what localization_monitor published, split out
    'verdict', 'reason', 'degraded', 'healthy', 'held',
    'd', 'near', 'beams', 'mapped', 'sigma', 'mo_age',
    # what the arbiter is forwarding, and what the wheels got
    'arb_mode', 'arb_active', 'wheel_vx', 'wheel_wz',
    # GROUND TRUTH — scoring only, never an input
    'gt_x', 'gt_y', 'gt_yaw',
]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def parse_kv(line):
    fields = {}
    for part in (line or '').split():
        key, sep, value = part.partition('=')
        if sep and key:
            fields[key] = value
    return fields


def state_of(line):
    for token in (line or '').split():
        if token.startswith('state='):
            return token[len('state='):] or '--'
    return (line or '--').strip() or '--'


class HealthRecorder(Node):

    def __init__(self, args):
        # use_sim_time forced on, for the reason c2m5_locrec records: the
        # node clock is otherwise wall time while every stamp is sim time.
        super().__init__('c2m51_hrec', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.args = args
        self.rows = []
        self.t0 = None
        self.health = ''
        self.state = '--'
        self.arbiter = ''
        self.wheel = (float('nan'), float('nan'))
        self.gt = None
        self.terminal_seen = False

        sensor = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.create_subscription(
            String, '/localization/health',
            lambda m: setattr(self, 'health', m.data), 10)
        self.create_subscription(
            String, '/mission/state',
            lambda m: setattr(self, 'state', state_of(m.data)), 10)
        self.create_subscription(
            String, '/cmd_vel_arbiter/status',
            lambda m: setattr(self, 'arbiter', m.data), 10)
        self.create_subscription(
            Odometry, '/model/coco/odometry', self._on_gt, sensor)
        # TwistStamped, because the wheel topic carries both types and a
        # Twist subscriber is silently blind on it.
        self.create_subscription(
            TwistStamped, '/diff_drive_controller/cmd_vel',
            self._on_wheel, 10)

        self.create_timer(1.0 / args.hz, self._tick)
        self.get_logger().info(f'recording -> {args.out} (tag={args.tag})')

    def _on_gt(self, msg):
        p = msg.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_wheel(self, msg):
        self.wheel = (msg.twist.linear.x, msg.twist.angular.z)

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        t = self.sim_now()
        if self.t0 is None:
            self.t0 = t
        h = parse_kv(self.health)
        a = parse_kv(self.arbiter)
        gt = self.gt or (float('nan'),) * 3
        self.rows.append({
            't_wall': f'{time.monotonic():.3f}',
            't_sim': f'{t - self.t0:.3f}',
            'tag': self.args.tag,
            'state': self.state,
            'verdict': h.get('verdict', '--'),
            'reason': h.get('reason', '--'),
            'degraded': h.get('degraded', '--'),
            'healthy': h.get('healthy', '--'),
            'held': h.get('held', '--'),
            'd': h.get('d', '--'),
            'near': h.get('near', '--'),
            'beams': h.get('beams', '--'),
            'mapped': h.get('mapped', '--'),
            'sigma': h.get('sigma', '--'),
            'mo_age': h.get('mo_age', '--'),
            'arb_mode': a.get('mode', '--'),
            'arb_active': a.get('active', '--'),
            'wheel_vx': f'{self.wheel[0]:.4f}',
            'wheel_wz': f'{self.wheel[1]:.4f}',
            'gt_x': f'{gt[0]:.4f}',
            'gt_y': f'{gt[1]:.4f}',
            'gt_yaw': f'{gt[2]:.4f}',
        })
        if self.state in ('COMPLETE', 'ABORT'):
            if not self.terminal_seen:
                self.terminal_seen = True
                self.get_logger().warn(f'mission reached {self.state}')
            if self.args.stop_on_terminal:
                raise SystemExit(0)

    def write(self):
        with open(self.args.out, 'w', newline='') as fh:
            writer = csv.DictWriter(fh, fieldnames=COLUMNS)
            writer.writeheader()
            writer.writerows(self.rows)
        print(f'wrote {len(self.rows)} rows to {self.args.out}')


# ── offline scoring ──────────────────────────────────────────────────────
def summarise(path):
    """What the monitor did over the run, per mission state."""
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        print('no rows')
        return

    def f(v):
        try:
            x = float(v)
            return None if math.isnan(x) else x
        except (TypeError, ValueError):
            return None

    dt = 1.0
    times = [f(r['t_sim']) for r in rows if f(r['t_sim']) is not None]
    if len(times) > 1:
        dt = (times[-1] - times[0]) / max(1, len(times) - 1)

    print(f'{path}: {len(rows)} rows, {times[-1] - times[0]:.1f} s sim')
    print()
    print(f'{"state":<18}{"n":>6}{"gated":>7}{"d max":>9}{"d med":>9}'
          f'{"INCONS":>8}{"DEGRADED":>10}')
    seen = []
    for row in rows:
        if not seen or seen[-1][0] != row['state']:
            seen.append((row['state'], []))
        seen[-1][1].append(row)
    for state, group in seen:
        gated = [r for r in group if r['mapped'] == '1']
        ds = sorted(x for x in (f(r['d']) for r in gated) if x is not None)
        incons = sum(1 for r in gated if r['verdict'] == 'INCONSISTENT')
        degraded = sum(1 for r in group if r['degraded'] == '1')
        print(f'{state:<18}{len(group):>6}{len(gated):>7}'
              f'{(ds[-1] if ds else float("nan")):>9.4f}'
              f'{(ds[len(ds) // 2] if ds else float("nan")):>9.4f}'
              f'{incons:>8}{degraded:>10}')

    print()
    total_degraded = sum(1 for r in rows if r['degraded'] == '1')
    total_incons = sum(1 for r in rows
                       if r['mapped'] == '1'
                       and r['verdict'] == 'INCONSISTENT')
    # Contiguous runs of a latched degradation: how many times the
    # executive would have been told to recover.
    triggers, run = 0, False
    for row in rows:
        if row['degraded'] == '1' and not run:
            triggers += 1
            run = True
        elif row['degraded'] != '1':
            run = False
    print(f'INCONSISTENT samples on mapped ground : {total_incons}')
    print(f'latched-degraded samples             : {total_degraded} '
          f'({total_degraded * dt:.1f} s)')
    print(f'DISTINCT RECOVERY TRIGGERS           : {triggers}')

    # Ground truth, for scoring only.
    errs = []
    for row in rows:
        gx, gy = f(row['gt_x']), f(row['gt_y'])
        if gx is not None and gy is not None:
            errs.append((gx, gy))
    if errs:
        print(f'ground truth x range                 : '
              f'{min(e[0] for e in errs):+.2f} .. '
              f'{max(e[0] for e in errs):+.2f}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', default='c2m51_run.csv')
    ap.add_argument('--tag', default='run')
    ap.add_argument('--hz', type=float, default=10.0)
    ap.add_argument('--stop-on-terminal', action='store_true')
    ap.add_argument('--summarise', metavar='CSV')
    args = ap.parse_args()

    if args.summarise:
        summarise(args.summarise)
        return

    rclpy.init()
    node = HealthRecorder(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.write()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
