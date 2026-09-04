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
"""C2-NAV.14 result: what actually happened on the ONE fresh seven-leg
tour run with the heading-correcting through-pose HEADING_POSE =
(-3.00, 0.625) added ahead of the existing waypoint (-3.40, 1.35) and
final goal (-3.575, 2.95), all in ONE NavigateThroughPoses request.

Two data sources:
  1. `.navbench/results/c2n14_tour_r1.json` -- the committed-shape
     per-leg summary `nav_bench.py` itself writes (same JSON schema as
     every C2-NAV.5/8/10/11/12 bench). Numbers pulled from here are
     cross-checked against the raw trace below (self-test).
  2. `.navbench/results/c2n14_tour_r1_traces/enclosure_entry_rep0.csv`
     -- the raw 0.1 s ground-truth trace (NOT committed, `.navbench/` is
     scratch per this repo's established convention -- see
     C2-NAV.13/c2nav13_heading.py's own docstring). Every number pulled
     from it is written into the committed JSON this script produces
     (`c2nav14_bench.json`) precisely so the finding survives even
     though the raw trace might not.

Usage:
  python3 c2nav14_report.py selftest
  python3 c2nav14_report.py all
  python3 c2nav14_report.py dump <out.json>
"""
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c2nav12_report import (                                  # noqa: E402
    WAYPOINT, GOAL_SHIFTED, SW_CORNER, DEADLOCK_POSE,
)
from c2nav13_heading import RATE_PERIOD, RPG_RADIUS, TOUR_ENTRY  # noqa: E402
from c2nav14_heading_pose import HEADING_POSE                  # noqa: E402

REPO_ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
TRACE_ROOT = os.path.join(REPO_ROOT, '.navbench', 'results')
TAG = 'c2n14_tour_r1'
CM_NAMES = {0: 'DO_NOTHING', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH',
            4: 'LIMIT'}

# C2-NAV.12 r2's own committed numbers (docs/SESSION_LOG.md "Finding 4"),
# quoted here for direct comparison, not re-derived.
C2NAV12_R2 = dict(
    west_column_entry_t_s=12.70,
    frozen_pose=(-3.249, 1.901),
    dist_to_c2nav8_r1_pose_mm=51.8,
    heading_swing_tick_t_s=9.01,
)


def hdr(t):
    print()
    print('=' * 78)
    print(t)
    print('=' * 78)


def bearing_deg(x, y, tx, ty):
    return math.degrees(math.atan2(ty - y, tx - x))


def load_json():
    return json.load(open(os.path.join(TRACE_ROOT, f'{TAG}.json')))


def load_trace(leg):
    p = os.path.join(TRACE_ROOT, f'{TAG}_traces', f'{leg}_rep0.csv')
    rows = []
    with open(p) as f:
        for r in csv.DictReader(f):
            if r['x'] in (None, ''):
                continue
            rows.append({'t': float(r['t_rel']), 'x': float(r['x']),
                         'y': float(r['y']), 'yaw': float(r['yaw']),
                         'cm': r['cm_action']})
    return rows


def dist(a, r):
    return math.hypot(r['x'] - a[0], r['y'] - a[1])


# ---------------------------------------------------------------------
# 1. SELF-TEST
# ---------------------------------------------------------------------

def self_test():
    hdr('SELF-TEST: raw trace vs. nav_bench.py\'s own committed JSON summary')
    ok = True
    j = load_json()
    leg = next(l for l in j['legs'] if l['scenario'] == 'enclosure_entry')
    rows = load_trace('enclosure_entry')

    last = rows[-1]
    print(f'  last trace row: t={last["t"]:.1f} pose=({last["x"]:+.4f},'
          f'{last["y"]:+.4f})')
    d_freeze = math.hypot(last['x'] - leg['end_world'][0],
                           last['y'] - leg['end_world'][1])
    print(f'  vs JSON end_world {leg["end_world"]}: {d_freeze*1000:.1f} mm')
    p1 = d_freeze < 0.01
    print(f'    {"PASS" if p1 else "FAIL"}')
    ok &= p1

    d_goal = math.dist((last['x'], last['y']), GOAL_SHIFTED)
    print(f'  final_goal_err recomputed: {d_goal:.3f} m  vs JSON '
          f'{leg["final_goal_err_m"]:.3f} m')
    p2 = abs(d_goal - leg['final_goal_err_m']) < 0.01
    print(f'    {"PASS" if p2 else "FAIL"}')
    ok &= p2

    print()
    print(f'  SELF-TEST: {"ALL PASS" if ok else "FAILED -- STOP, fix the tool"}')
    if not ok:
        raise SystemExit(1)
    return ok, j, rows


# ---------------------------------------------------------------------
# 2. HEADING EVOLUTION
# ---------------------------------------------------------------------

def heading_evolution(rows):
    hdr('HEADING EVOLUTION: corridor_gate-exit -> heading pose -> waypoint '
        '-> freeze')
    t0 = rows[0]
    print(f'  t=0 (corridor_gate exit / enclosure_entry leg start):')
    print(f'    pose ({t0["x"]:+.4f}, {t0["y"]:+.4f})  yaw {t0["yaw"]:+.4f} '
          f'rad ({math.degrees(t0["yaw"]):+.1f} deg)')
    tk = next(r for r in TOUR_ENTRY if abs(TOUR_ENTRY[r][0] - t0['x']) < 0.15
              and abs(TOUR_ENTRY[r][1] - t0['y']) < 0.15)
    print(f'    nearest C2-NAV.12 committed exit state for comparison: '
          f'{tk} = {TOUR_ENTRY[tk]}')

    best_hp = min(rows, key=lambda r: dist(HEADING_POSE, r))
    best_wp = min(rows, key=lambda r: dist(WAYPOINT, r))
    b_hp_wp = bearing_deg(*HEADING_POSE, *WAYPOINT)
    print()
    print(f'  desired heading (heading pose -> waypoint, C2-NAV.10\'s own '
          f'bearing): {b_hp_wp:+.2f} deg')
    print()
    print(f'  closest approach to HEADING_POSE {HEADING_POSE}:')
    print(f'    t={best_hp["t"]:.2f}s  pose ({best_hp["x"]:+.4f},'
          f'{best_hp["y"]:+.4f})  dist {dist(HEADING_POSE, best_hp)*1000:.0f} mm  '
          f'yaw {math.degrees(best_hp["yaw"]):+.1f} deg  '
          f'(dev from desired: {math.degrees(best_hp["yaw"])-b_hp_wp:+.1f} deg)')
    print(f'  closest approach to WAYPOINT {WAYPOINT}:')
    print(f'    t={best_wp["t"]:.2f}s  pose ({best_wp["x"]:+.4f},'
          f'{best_wp["y"]:+.4f})  dist {dist(WAYPOINT, best_wp)*1000:.0f} mm  '
          f'yaw {math.degrees(best_wp["yaw"]):+.1f} deg  '
          f'(dev from desired: {math.degrees(best_wp["yaw"])-b_hp_wp:+.1f} deg)')

    last = rows[-1]
    print()
    print(f'  final/frozen pose (t={last["t"]:.1f}s, leg TIMEOUT):')
    print(f'    ({last["x"]:+.4f}, {last["y"]:+.4f})  yaw '
          f'{math.degrees(last["yaw"]):+.1f} deg')
    d_dl = math.dist((last['x'], last['y']), DEADLOCK_POSE)
    print(f'    distance to C2-NAV.8 r1 / C2-NAV.12 canonical SW-corner '
          f'deadlock pose {DEADLOCK_POSE}: {d_dl*1000:.1f} mm')
    print(f'    (C2-NAV.12 r2\'s own frozen pose was '
          f'{C2NAV12_R2["dist_to_c2nav8_r1_pose_mm"]:.1f} mm from the same '
          f'reference pose)')
    return best_hp, best_wp, last, d_dl


# ---------------------------------------------------------------------
# 3. WAYPOINT / HEADING-POSE PERSISTENCE (RemovePassedGoals tick model)
# ---------------------------------------------------------------------

def removal_timeline(rows):
    hdr('REMOVAL-TICK TIMELINE (RateController hz=0.333, period '
        f'{RATE_PERIOD:.3f} s, RemovePassedGoals radius={RPG_RADIUS} m)')
    print('  NOTE: with TWO via-poses ahead of the final goal, whether '
          'RemovePassedGoals pops them strictly front-first or evaluates '
          'the whole remaining list per tick is NOT determined here -- '
          'the installed .deb does not ship the .cpp (same limitation '
          'C2-NAV.13 recorded). The table below reports the measured '
          'DISTANCE to each pose at each tick; it does not assert which '
          'pose(s) were actually removed by name.')
    print()
    header = f'{"tick_t":>8}{"d_HP_mm":>10}{"d_WP_mm":>10}{"HP<0.7":>8}' \
             f'{"WP<0.7":>8}{"x":>9}{"y":>9}{"yaw_deg":>9}'
    print(' ', header)
    import bisect
    ts = [r['t'] for r in rows]
    tick = RATE_PERIOD
    out = []
    tmax = rows[-1]['t']
    n = 0
    while tick < min(tmax, 30.0) and n < 10:
        i = bisect.bisect_left(ts, tick)
        if i >= len(rows):
            break
        r = rows[i]
        dhp, dwp = dist(HEADING_POSE, r) * 1000, dist(WAYPOINT, r) * 1000
        row = dict(tick_t=round(tick, 3), d_hp_mm=round(dhp, 1),
                   d_wp_mm=round(dwp, 1), hp_in_radius=dhp < 700,
                   wp_in_radius=dwp < 700, x=r['x'], y=r['y'],
                   yaw_deg=round(math.degrees(r['yaw']), 1))
        out.append(row)
        print(f'  {tick:8.3f}{dhp:10.1f}{dwp:10.1f}{str(dhp<700):>8}'
              f'{str(dwp<700):>8}{r["x"]:9.3f}{r["y"]:9.3f}'
              f'{math.degrees(r["yaw"]):9.1f}')
        tick += RATE_PERIOD
        n += 1
    return out


def west_column_commit(rows):
    hdr('SW-CORNER (WEST-COLUMN) COMMITMENT TIMING, C2-NAV.13\'s own method '
        '(x < -3.10)')
    west = [r for r in rows if r['x'] < -3.10]
    if not west:
        print('  never entered the west column (x < -3.10)')
        return None
    first = west[0]
    print(f'  first west-column entry: t={first["t"]:.2f}s  pose '
          f'({first["x"]:+.4f},{first["y"]:+.4f})  yaw '
          f'{math.degrees(first["yaw"]):+.1f} deg')
    print(f'  C2-NAV.12 r2\'s own west-column entry: t='
          f'{C2NAV12_R2["west_column_entry_t_s"]:.2f}s')
    return first


def cm_transitions(rows):
    hdr('COLLISION-MONITOR ACTION TRANSITIONS (0=DO_NOTHING 1=STOP '
        '2=SLOWDOWN 3=APPROACH 4=LIMIT)')
    prev = None
    trans = []
    for r in rows:
        v = r['cm']
        if v != prev and v != '':
            trans.append((r['t'], CM_NAMES.get(int(v), v)))
            prev = v
    for t, name in trans:
        print(f'  t={t:7.2f}s  -> {name}')
    return trans


# ---------------------------------------------------------------------
# 4. DUMP -- committed derived record
# ---------------------------------------------------------------------

def dump(out_path):
    ok, j, rows = self_test()
    best_hp, best_wp, last, d_dl = heading_evolution(rows)
    ticks = removal_timeline(rows)
    west = west_column_commit(rows)
    trans = cm_transitions(rows)
    leg = next(l for l in j['legs'] if l['scenario'] == 'enclosure_entry')
    record = dict(
        tag=TAG,
        heading_pose=list(HEADING_POSE),
        waypoint=list(WAYPOINT),
        goal=list(GOAL_SHIFTED),
        leg_status=leg['status'],
        leg_duration_s=leg['duration_sim_s'],
        final_goal_err_m=leg['final_goal_err_m'],
        cm_action_frac=leg['cm_action_frac'],
        cm_polygon_secs=leg['cm_polygon_secs'],
        entry_t0=dict(x=rows[0]['x'], y=rows[0]['y'], yaw=rows[0]['yaw']),
        closest_to_heading_pose=dict(
            t=best_hp['t'], x=best_hp['x'], y=best_hp['y'],
            yaw_deg=math.degrees(best_hp['yaw']),
            dist_m=dist(HEADING_POSE, best_hp)),
        closest_to_waypoint=dict(
            t=best_wp['t'], x=best_wp['x'], y=best_wp['y'],
            yaw_deg=math.degrees(best_wp['yaw']),
            dist_m=dist(WAYPOINT, best_wp)),
        frozen_pose=dict(t=last['t'], x=last['x'], y=last['y'],
                          yaw_deg=math.degrees(last['yaw']),
                          dist_to_deadlock_pose_m=d_dl),
        removal_ticks=ticks,
        west_column_entry=(dict(t=west['t'], x=west['x'], y=west['y'])
                            if west else None),
        cm_transitions=[dict(t=t, name=n) for t, n in trans],
        c2nav12_r2_reference=C2NAV12_R2,
    )
    with open(out_path, 'w') as f:
        json.dump(record, f, indent=2)
    print()
    print(f'wrote {out_path}')
    return record


def main():
    argv = sys.argv[1:]
    if argv and argv[0] == 'selftest':
        ok, _, _ = self_test()
        return 0 if ok else 1
    if argv and argv[0] == 'dump':
        out = argv[1] if len(argv) > 1 else os.path.join(
            HERE, 'c2nav14_bench.json')
        dump(out)
        return 0
    dump(os.path.join(HERE, 'c2nav14_bench.json'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
