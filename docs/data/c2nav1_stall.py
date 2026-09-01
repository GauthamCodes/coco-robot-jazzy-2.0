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
c2nav1_stall.py — characterise the longest commanded stall in a leg.

C2-NAV.0 established that the enclosure_entry failure is a stall in
TRANSIT, not a terminal-yaw settle: the robot stops with a healthy
global plan, a free band in the local costmap, and most of DWB's
trajectory set still legal. This script pulls the four numbers needed to
say whether that stall survived a change, from the same trace format,
so baseline and post-change legs are described identically.

The stall is the longest contiguous run of COMMANDED crawl on
/cmd_vel_nav (|v_nav| < 0.05 m/s) — the same definition nav_bench.py
uses for its "worst moment" anchor, and the same one C2-NAV.0 reported
47.8 s against.

Usage:
  ./c2nav1_stall.py <trace.csv> [<trace.csv> ...]
"""
import csv
import math
import os
import sys

CRAWL_V = 0.05

# From nav_bench.py TOUR, world coordinates. Keyed by the scenario name
# that starts every trace filename.
GOALS = {
    'open_space': (-2.00, -2.20),
    'wall_adjacent': (-2.00, -3.00),
    'wall_parallel': (0.50, -2.95),
    'obstacle_corner': (0.30, -0.30),
    'corridor_gate': (-2.60, -0.10),
    'enclosure_entry': (-3.45, 2.95),
    'enclosure_exit': (-2.00, 0.00),
}

CM_NAMES = {'0': 'DO_NOTHING', '1': 'STOP', '2': 'SLOWDOWN',
            '3': 'APPROACH', '4': 'LIMIT'}


def num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def goal_for(path):
    base = os.path.basename(path)
    for k in sorted(GOALS, key=len, reverse=True):
        if base.startswith(k):
            return k, GOALS[k]
    return None, None


def describe(path):
    name, goal = goal_for(path)
    rows = [r for r in csv.DictReader(open(path))]
    if not rows or goal is None:
        print(f'{path}: no rows or unknown scenario')
        return
    ts = [num(r['t_rel']) for r in rows]

    # longest contiguous run of commanded crawl
    best_i, best_j, best_len = None, None, 0.0
    i = None
    for k, r in enumerate(rows):
        v = num(r['v_nav'])
        crawl = v is not None and abs(v) < CRAWL_V
        if crawl:
            i = k if i is None else i
            if ts[k] - ts[i] > best_len:
                best_len, best_i, best_j = ts[k] - ts[i], i, k
        else:
            i = None

    print(f'\n=== {os.path.basename(path)}   goal world {goal}')
    print(f'  leg duration            {ts[-1] - ts[0]:.2f} s '
          f'({len(rows)} samples)')
    if best_i is None:
        print('  no commanded stall (|v_nav| < 0.05) of any length')
        return
    sub = rows[best_i:best_j + 1]
    r0 = rows[best_i]
    d0 = math.dist((num(r0['x']), num(r0['y'])), goal)
    dend = math.dist((num(rows[-1]['x']), num(rows[-1]['y'])), goal)
    print(f'  longest commanded stall {best_len:.2f} s '
          f'({100.0 * best_len / (ts[-1] - ts[0]):.1f}% of the leg)')
    print(f'  starts at t_rel         {ts[best_i]:.2f} s')
    print(f'  distance remaining then {d0:.3f} m')
    print(f'  distance remaining end  {dend:.3f} m')

    def col(key):
        return [num(r[key]) for r in sub if num(r[key]) is not None]

    bvx = col('dwb_best_vx')
    if bvx:
        zero = sum(1 for v in bvx if abs(v) < 1e-6)
        print(f'  DWB best vx == 0        {zero}/{len(bvx)} samples '
              f'({100.0 * zero / len(bvx):.1f}%)  mean {sum(bvx)/len(bvx):.4f}')
    ill = col('dwb_illegal')
    n = col('dwb_n')
    if ill and n and sum(n):
        print(f'  DWB trajectories        {sum(n)/len(n):.0f} per cycle, '
              f'{100.0 * sum(ill) / sum(n):.1f}% illegal '
              f'({sum(n)/len(n) - sum(ill)/len(ill):.0f} legal)')
    sm = col('scan_min')
    if sm:
        print(f'  nearest scan return     {min(sm):.3f} m '
              f'(median {sorted(sm)[len(sm)//2]:.3f} m)')
    acts = {}
    for r in sub:
        a = CM_NAMES.get((r.get('cm_action') or '').strip(),
                         r.get('cm_action'))
        acts[a] = acts.get(a, 0) + 1
    print('  collision monitor       ' + ', '.join(
        f'{k} {100.0 * v / len(sub):.0f}%'
        for k, v in sorted(acts.items(), key=lambda kv: -kv[1])))
    for key, label in (('v_nav', 'cmd_vel_nav'),
                       ('v_smoothed', 'cmd_vel_smoothed'),
                       ('v_cmdvel', 'cmd_vel'),
                       ('v_wheel', 'wheels'),
                       ('v_act', 'ground truth')):
        v = col(key)
        if v:
            print(f'  {label:<22}  mean {sum(v)/len(v):+.4f}  '
                  f'max {max(v, key=abs):+.4f}')


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    for p in argv[1:]:
        describe(p)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
