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
c2nav0_analysis.py — reduce nav_bench output to the C2-NAV.0 tables.

Three separate questions, three subcommands, because they have different
answers and merging them would hide that:

  table   the per-scenario baseline table, median over repeats
  chain   is the collision monitor's output what reaches the wheels?
  arith   the DWB scoring arithmetic, from the params and the Jazzy
          headers, with the measured stall as a cross-check

Usage:
  ./c2nav0_analysis.py table c2nav0_baselineA.json c2nav0_baselineB.json
  ./c2nav0_analysis.py chain '<navbench>/baselineB_traces/*.csv'
  ./c2nav0_analysis.py arith
"""
import csv
import glob
import json
import math
import statistics
import sys
from collections import Counter

# ---------------------------------------------------------------- table

ORDER = ['open_space', 'wall_adjacent', 'wall_parallel', 'obstacle_corner',
         'corridor_gate', 'enclosure_entry', 'enclosure_exit']


def med(vals):
    vals = [v for v in vals if v is not None]
    return round(statistics.median(vals), 3) if vals else None


def table(paths):
    for path in paths:
        d = json.load(open(path))
        legs = d['legs']
        by = {}
        for leg in legs:
            by.setdefault(leg.get('scenario'), []).append(leg)
        print(f"\n######## {d['tag']}  ({len(legs)} legs)")
        hdr = (f'{"scenario":<17}{"ok":>6}{"t_s":>7}{"transit":>8}{"term":>7}'
               f'{"len":>7}{"clear":>7}{"v_tr":>7}{"stop":>5}{"osc":>5}'
               f'{"gate%":>7}{"prog":>5}')
        print(hdr)
        print('-' * len(hdr))
        for s in ORDER:
            ls = by.get(s)
            if not ls:
                continue
            ok = sum(1 for x in ls if x.get('status') == 'SUCCEEDED')
            print(f'{s:<17}{f"{ok}/{len(ls)}":>6}'
                  f'{med([x.get("duration_sim_s") for x in ls]):>7}'
                  f'{str(med([x.get("t_transit_s") for x in ls])):>8}'
                  f'{str(med([x.get("t_terminal_s") for x in ls])):>7}'
                  f'{med([x.get("path_len_m") for x in ls]):>7}'
                  f'{med([x.get("min_clearance_m") for x in ls]):>7}'
                  f'{str(med([x.get("transit_speed_mean") for x in ls])):>7}'
                  f'{med([x.get("n_stops") for x in ls]):>5}'
                  f'{med([x.get("n_osc_cmd") for x in ls]):>5}'
                  # str(): a leg short enough that the collision
                  # monitor never published a state has cm_gated_frac
                  # None, which f-string formatting cannot right-align.
                  # C2-NAV.1 produced the first such legs. Output is
                  # unchanged for every non-None value.
                  f'{str(med([x.get("cm_gated_frac") for x in ls])):>7}'
                  f'{str(med([x.get("n_progress_failures") for x in ls])):>5}')
        tr = [x.get('t_transit_s') for x in legs if x.get('t_transit_s')]
        te = [x.get('t_terminal_s') for x in legs if x.get('t_terminal_s')]
        print(f'\n  succeeded              '
              f'{sum(1 for x in legs if x.get("status") == "SUCCEEDED")}'
              f'/{len(legs)}')
        if tr and te:
            mt, mte = statistics.median(tr), statistics.median(te)
            print(f'  median transit         {mt:.2f} s')
            print(f'  median terminal        {mte:.2f} s  '
                  f'({mte / (mt + mte):.1%} of a leg)')
        print(f'  median transit speed   '
              f'{med([x.get("transit_speed_mean") for x in legs])} m/s '
              f'(max_vel_x 0.30)')
        print(f'  median DWB rate        '
              f'{med([x.get("dwb_hz") for x in legs])} Hz '
              f'(controller_frequency 10.0)')
        for k, label in (('n_progress_failures', 'progress failures'),
                         ('n_loop_rate_misses', 'loop-rate misses'),
                         ('n_stale_cmd_drops', 'stale-cmd drops')):
            print(f'  total {label:<22}'
                  f'{sum(x.get(k, 0) for x in legs)}')
        tot = Counter()
        for leg in legs:
            tot.update(leg.get('dwb_illegal_by_critic') or {})
        n = sum(tot.values()) or 1
        print('  rejections by critic:  ' + ', '.join(
            f'{k} {v} ({100.0 * v / n:.1f}%)'
            for k, v in tot.most_common()))
        crit = {}
        for leg in legs:
            for k, v in (leg.get('dwb_best_critic_mean') or {}).items():
                crit.setdefault(k, []).append(v)
        s = {k: statistics.mean(v) for k, v in crit.items()}
        tt = sum(s.values()) or 1
        print('  score share of the CHOSEN trajectory: ' + ', '.join(
            f'{k} {v:.1f} ({100.0 * v / tt:.1f}%)'
            for k, v in sorted(s.items(), key=lambda kv: -kv[1])))


# ---------------------------------------------------------------- chain

TOL = 1e-4


def _num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def chain(pattern):
    """v_smoothed (monitor IN) -> v_cmdvel (monitor OUT) -> v_wheel.

    A wheel command ABOVE the monitor's output means something other than
    the monitor is deciding what the wheels do.
    """
    files = sorted(glob.glob(pattern))
    if not files:
        print(f'no traces matching {pattern}')
        return
    tot = Counter()
    worst, worst_row = 0.0, None
    for path in files:
        with open(path) as f:
            for row in csv.DictReader(f):
                vin, vout = _num(row['v_smoothed']), _num(row['v_cmdvel'])
                vwh = _num(row['v_wheel'])
                if vin is None or vout is None or vwh is None:
                    continue
                tot['rows'] += 1
                if abs(vin - vout) > TOL:
                    tot['monitor_reduced'] += 1
                if abs(vout - vwh) > TOL:
                    tot['out_ne_wheel'] += 1
                if abs(vwh) > abs(vout) + TOL:
                    tot['wheel_exceeds_gate'] += 1
                    if abs(vwh) - abs(vout) > worst:
                        worst = abs(vwh) - abs(vout)
                        worst_row = (path.split('/')[-1], row['t_rel'],
                                     vin, vout, vwh)
    r = tot['rows'] or 1
    print(f'{len(files)} traces, {tot["rows"]} rows')
    print(f'  monitor reduced the command : {tot["monitor_reduced"]:6} '
          f'({100.0 * tot["monitor_reduced"] / r:5.2f} %)')
    print(f'  monitor out != wheel command: {tot["out_ne_wheel"]:6} '
          f'({100.0 * tot["out_ne_wheel"] / r:5.2f} %)')
    print(f'  wheels EXCEEDED the gate    : '
          f'{tot["wheel_exceeds_gate"]:6} '
          f'({100.0 * tot["wheel_exceeds_gate"] / r:5.2f} %)')
    if worst_row:
        print(f'  worst overshoot: {worst_row[0]} t={worst_row[1]}s  '
              f'in={worst_row[2]} gate={worst_row[3]} wheel={worst_row[4]}  '
              f'gap={worst:.4f} m/s')


# ---------------------------------------------------------------- arith

RES, INSCRIBED, CSF, INFLATION = 0.05, 0.20, 5.0, 0.50
BASE_OBSTACLE_SCALE = 8.0


def _cost(d):
    if d <= 0:
        return 254.0
    if d <= INSCRIBED:
        return 253.0
    return 252.0 * math.exp(-CSF * (d - INSCRIBED))


def arith():
    """Why the robot prefers standing still inside the inflation band.

    dwb_critics/map_grid.hpp:69   getScale() = resolution * 0.5 * scale
    dwb_core/trajectory_critic.hpp:177  getScale() = scale  (BaseObstacle)
    InflationLayer  cost(d) = 252 * exp(-csf * (d - inscribed))

    DWB MINIMISES, and BaseObstacle with sum_scores:false scores only the
    trajectory's FINAL pose. In a cost field that rises along the
    direction of travel the cheapest trajectory is the one that travels
    least, and the goal critics must outbid that.
    """
    print('effective weight per unit scored:')
    print(f'  BaseObstacle   {BASE_OBSTACLE_SCALE:6.2f} per unit COST '
          f'(cost 0..252)')
    for n, s in (('PathDist', 32.0), ('PathAlign', 32.0),
                 ('GoalDist', 24.0), ('GoalAlign', 24.0)):
        e = RES * 0.5 * s
        print(f'  {n:<14} {e:6.2f} per CELL ({e / RES:5.2f} per metre)')
    gain = RES * 0.5 * 24.0 + RES * 0.5 * 32.0
    print(f'\none 0.05 m cell of progress is worth {gain:.2f} '
          f'(GoalDist + PathDist)')
    print(f'{"from d":>8}{"to d":>8}{"dCost":>9}{"dBaseObstacle":>15}'
          f'{"ratio":>9}')
    for d in (0.50, 0.45, 0.40, 0.35, 0.30, 0.25):
        d2 = d - RES
        dc = _cost(d2) - _cost(d)
        print(f'{d:8.3f}{d2:8.3f}{dc:9.1f}{dc * BASE_OBSTACLE_SCALE:15.1f}'
              f'{dc * BASE_OBSTACLE_SCALE / gain:8.0f}x')
    print('\ncross-check against the measured stall '
          '(enclosure_entry rep0, t+13.43 s):')
    c = 456.0 / BASE_OBSTACLE_SCALE
    d = INSCRIBED + math.log(252.0 / c) / CSF
    print(f'  BaseObstacle on the chosen trajectory 456.0 -> cost {c:.1f} '
          f'-> clearance {d:.3f} m')
    print(f'  robot was at world (-2.305, 2.852); box_obstacle_1 NE corner '
          f'(-2.75, 2.65) is '
          f'{math.dist((-2.305, 2.852), (-2.75, 2.65)):.3f} m away')
    print('  the cost field is behaving exactly as the formula says.')
    gap = 3.40 - 2.65
    print(f'\nthe channel it refused to enter is {gap:.2f} m wide:')
    print(f'  non-inscribed band {gap - 2 * INSCRIBED:.2f} m, '
          f'zero-cost band {max(0.0, gap - 2 * INFLATION):.2f} m')
    mid = gap / 2
    print(f'  best clearance at its centre {mid:.3f} m -> cost '
          f'{_cost(mid):.0f} -> BaseObstacle {_cost(mid) * 8:.0f}')
    print(f'  entering costs {_cost(mid) * 8 - 456:.0f}; the 0.5 m of '
          f'progress it buys is worth {10 * gain:.0f}')
    print('\ncollision monitor zones are SQUARES, so their reach on the '
          'diagonal is not their half-width:')
    for name, half in (('PolygonSlow', 0.40), ('PolygonLimit', 0.55)):
        print(f'  {name:<14} half-width {half:.2f} m -> fires out to '
              f'{half * math.sqrt(2):.3f} m')


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    cmd = argv[1]
    if cmd == 'table':
        table(argv[2:])
    elif cmd == 'chain':
        chain(argv[2])
    elif cmd == 'arith':
        arith()
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
