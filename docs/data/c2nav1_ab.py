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
c2nav1_ab.py — the C2-NAV.1 before/after table.

One controlled change (the goal checker: SimpleGoalChecker ->
PositionGoalChecker) against the C2-NAV.0 baseline, same tour, same
topology, same timeout. Every reduction here is the one
`c2nav0_analysis.py table` already used, so the "baseline" column of
this table and the C2-NAV.0 table are the same numbers by construction
rather than by transcription.

The trace directories are optional and supply the final-heading column,
which is the thing the change gives away and so is the one number that
is expected to get WORSE.

Usage:
  ./c2nav1_ab.py c2nav0_baselineA.json c2nav1_navA_goalyaw.json \
      [<baselineA_traces_dir> <navA_goalyaw_traces_dir>]
"""
import csv
import glob
import json
import math
import os
import statistics
import sys
from collections import Counter

ORDER = ['open_space', 'wall_adjacent', 'wall_parallel', 'obstacle_corner',
         'corridor_gate', 'enclosure_entry', 'enclosure_exit']


def med(vals):
    vals = [v for v in vals if v is not None]
    return statistics.median(vals) if vals else None


def load(path):
    d = json.load(open(path))
    return d['tag'], d['legs']


def final_yaws(tracedir):
    """|final heading| per leg, radians.

    The goal was always sent with orientation.w = 1.0, i.e. map yaw 0,
    so |yaw| IS the heading error against the goal actually requested.
    The map frame is a pure translation of the world frame (see
    nav_bench.py WORLD_TO_MAP_*), so world yaw == map yaw.
    """
    out = {}
    if not tracedir or not os.path.isdir(tracedir):
        return out
    for path in sorted(glob.glob(os.path.join(tracedir, '*.csv'))):
        rows = list(csv.DictReader(open(path)))
        if not rows:
            continue
        try:
            y = float(rows[-1]['yaw'])
        except (KeyError, TypeError, ValueError):
            continue
        y = (y + math.pi) % (2 * math.pi) - math.pi
        out[os.path.basename(path)[:-4]] = abs(y)
    return out


def agg(legs, yaws):
    n = len(legs)
    ok = sum(1 for x in legs if x.get('status') == 'SUCCEEDED')
    tr = [x.get('t_transit_s') for x in legs if x.get('t_transit_s')]
    te = [x.get('t_terminal_s') for x in legs if x.get('t_terminal_s')]
    crit = Counter()
    for leg in legs:
        crit.update(leg.get('dwb_illegal_by_critic') or {})
    mt = statistics.median(tr) if tr else None
    mte = statistics.median(te) if te else None
    clear = [x.get('min_clearance_m') for x in legs
             if x.get('min_clearance_m') is not None]
    enc = [x for x in legs if x.get('scenario') == 'enclosure_entry']
    wall = [x for x in legs if x.get('scenario') == 'wall_adjacent']
    a = {
        'success': f'{ok}/{n}',
        'median leg duration (s)': med([x.get('duration_sim_s')
                                        for x in legs]),
        'median transit time (s)': mt,
        'median transit speed (m/s)': med([x.get('transit_speed_mean')
                                           for x in legs]),
        'median terminal time (s)': mte,
        'terminal phase (% of leg)': (100.0 * mte / (mt + mte)
                                      if mt and mte else None),
        'median terminal yaw travel (rad)': med(
            [x.get('terminal_yaw_travel_rad') for x in legs]),
        'median frac actual v < 0.05': med([x.get('frac_actual_below_0.05')
                                            for x in legs]),
        'median frac cmd v < 0.05': med([x.get('frac_cmd_below_0.05')
                                         for x in legs]),
        'median n_stops per leg': med([x.get('n_stops') for x in legs]),
        'total n_stops': sum(x.get('n_stops', 0) for x in legs),
        'median min clearance (m)': med(clear),
        'WORST min clearance (m)': min(clear) if clear else None,
        'median path length (m)': med([x.get('path_len_m') for x in legs]),
        # Ground-truth arrival error. The goal checker judges against
        # AMCL, so this is NOT the number it stopped on, and a leg that
        # ends its terminal settle later gives AMCL longer to converge.
        'median arrival error (m)': med([x.get('final_goal_err_m')
                                         for x in legs]),
        'max arrival error (m)': max(
            [x.get('final_goal_err_m') for x in legs
             if x.get('final_goal_err_m') is not None] or [None],
            key=lambda v: -1 if v is None else v),
        'legs reaching GT 0.25 m': '{}/{}'.format(
            sum(1 for x in legs if x.get('t_transit_s') is not None), n),
        'median cross-track (m)': med([x.get('xtrack_med_m') for x in legs]),
        'RotateToGoal rejections': crit.get('RotateToGoal', 0),
        'BaseObstacle rejections': crit.get('BaseObstacle', 0),
        'Oscillation rejections': crit.get('Oscillation', 0),
        'total rejected trajectories': sum(crit.values()),
        'progress-checker aborts': sum(x.get('n_progress_failures', 0)
                                       for x in legs),
        'median DWB rate (Hz)': med([x.get('dwb_hz') for x in legs]),
        'median DWB illegal frac': med([x.get('dwb_illegal_frac')
                                        for x in legs]),
        'median best-vx zero frac': med([x.get('dwb_best_vx_zero_frac')
                                         for x in legs]),
        'enclosure_entry success': '{}/{}'.format(
            sum(1 for x in enc if x.get('status') == 'SUCCEEDED'), len(enc)),
        'wall_adjacent success': '{}/{}'.format(
            sum(1 for x in wall if x.get('status') == 'SUCCEEDED'),
            len(wall)),
    }
    if yaws:
        vals = list(yaws.values())
        a['median |final heading| (rad)'] = med(vals)
        a['max |final heading| (rad)'] = max(vals) if vals else None
    share = {}
    for leg in legs:
        for k, v in (leg.get('dwb_best_critic_mean') or {}).items():
            share.setdefault(k, []).append(v)
    means = {k: statistics.mean(v) for k, v in share.items()}
    tot = sum(means.values()) or 1
    a['BaseObstacle score share (%)'] = 100.0 * means.get(
        'BaseObstacle', 0.0) / tot
    return a


def fmt(v):
    if v is None:
        return '-'
    if isinstance(v, str):
        return v
    if isinstance(v, float):
        return f'{v:.1f}' if abs(v) >= 100 else f'{v:.3f}'
    return str(v)


def delta(a, b):
    if isinstance(a, str) or isinstance(b, str) or a is None or b is None:
        return ''
    if a == 0:
        return f'{b - a:+.3g}'
    return f'{b - a:+.3g} ({100.0 * (b - a) / abs(a):+.0f}%)'


def main(argv):
    tag_a, legs_a = load(argv[1])
    tag_b, legs_b = load(argv[2])
    ya = final_yaws(argv[3] if len(argv) > 3 else None)
    yb = final_yaws(argv[4] if len(argv) > 4 else None)
    A, B = agg(legs_a, ya), agg(legs_b, yb)

    w = max(len(k) for k in A)
    print(f'\n{"metric":<{w}}  {tag_a:>14}  {tag_b:>14}  change')
    print('-' * (w + 52))
    for k in A:
        print(f'{k:<{w}}  {fmt(A[k]):>14}  {fmt(B.get(k)):>14}  '
              f'{delta(A[k], B.get(k))}')

    print('\nPER SCENARIO       ok | median leg s | transit s | terminal s '
          '| clear m')

    def row(ls):
        ok = sum(1 for x in ls if x.get('status') == 'SUCCEEDED')
        return (f'{ok}/{len(ls)} '
                f'{fmt(med([x.get("duration_sim_s") for x in ls])):>8} '
                f'{fmt(med([x.get("t_transit_s") for x in ls])):>10} '
                f'{fmt(med([x.get("t_terminal_s") for x in ls])):>11} '
                f'{fmt(med([x.get("min_clearance_m") for x in ls])):>8}')

    for s in ORDER:
        la = [x for x in legs_a if x.get('scenario') == s]
        lb = [x for x in legs_b if x.get('scenario') == s]
        if not la and not lb:
            continue
        print(f'  {s:<17} A: {row(la)}')
        print(f'  {"":<17} B: {row(lb)}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
