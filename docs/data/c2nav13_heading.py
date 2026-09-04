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
"""C2-NAV.13 diagnosis: does the corridor_gate-exit heading reversal
(fresh +0.3..+0.5 rad, C2-NAV.11, vs tour -0.3..-0.5 rad, C2-NAV.12)
actually determine how close the enclosure_entry approach passes the
waypoint (-3.40, 1.35), and is RemovePassedGoals(radius=0.7)'s premature
removal of that waypoint causally connected to the SW-corner failures?

OFFLINE ONLY. No simulator, no Nav2 parameter touched. Reuses C2-NAV.9's
geometry machinery (BOXES, dist_to_box, zone_status_all_yaw,
square_reach) and C2-NAV.12's constants (WAYPOINT, GOAL_SHIFTED,
SW_CORNER, DEADLOCK_POSE) BY IMPORT -- nothing here re-derives what
those scripts already got right.

Two data sources, both read-only:
  1. docs/data/c2nav12_bench.json / c2nav9_corridor.py's own committed
     numbers -- the COMMITTED record, reproduced here as a self-test.
  2. .navbench/results/{c2n11_appr_r*,c2n12_tour_r*}_traces/*.csv -- the
     raw per-0.1s ground-truth traces this session's local checkout
     still has on disk. These are NOT committed (`.navbench/` is
     scratch, per CLAUDE.md's own working style and this repo's
     established convention for c2n*_run.sh/*.log/*.csv). Every number
     pulled from them is cross-checked against a committed figure
     wherever one exists (the self-test), and the full derived timeline
     this script computes is written to a committed JSON
     (c2nav13_bench.json) precisely so the finding survives even though
     the raw traces might not.

Usage:
  python3 c2nav13_heading.py self_test
  python3 c2nav13_heading.py states       # entry pose/yaw/vectors, fresh vs tour
  python3 c2nav13_heading.py heading      # clearance-vs-heading sensitivity
  python3 c2nav13_heading.py timeline     # waypoint-distance timeline + removal tick
  python3 c2nav13_heading.py divergence   # SW-corner-commit time vs removal tick
  python3 c2nav13_heading.py counterfactual
  python3 c2nav13_heading.py dump <out.json>   # write the committed derived record
  python3 c2nav13_heading.py all
"""
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c2nav9_corridor import (                                # noqa: E402
    BOXES, dist_to_box, rect, zone_status_all_yaw, square_reach, in_circle,
    body_frame, POLY_STOP_R, POLY_SLOW_HW, POLY_LIMIT_HW, POLY_SLOW_MAX,
    POLY_LIMIT_MAX, FOOT_CIRC_R, build_clearance_grid, bottleneck,
    CORRIDOR_GATE_GOAL,
)
from c2nav12_report import (                                 # noqa: E402
    WAYPOINT, GOAL_SHIFTED, SW_CORNER, DEADLOCK_POSE,
)

REPO_ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
TRACE_ROOT = os.path.join(REPO_ROOT, '.navbench', 'results')
BOX1 = [b for b in BOXES if b[0] == 'box_obstacle_1'][0]
RATE_HZ = 0.333               # RateController hz=0.333, installed stock
                               # navigate_through_poses_w_replanning_and_recovery.xml,
                               # line 12 -- NOT 1.0. Period ~= 3.003 s.
RATE_PERIOD = 1.0 / RATE_HZ
RPG_RADIUS = 0.7               # RemovePassedGoals radius, same XML line 15

# corridor_gate-end pose (== enclosure_entry leg-start pose), read directly
# off the last row of each run's corridor_gate_rep0.csv trace. Frozen here
# (not re-read every call) because the two source directories
# (c2n11_appr_r*, c2n12_tour_r*) are the entire population this experiment
# has -- there is no larger N to sample from offline.
FRESH_ENTRY = {   # C2-NAV.11, fresh two-leg start (spawn -> corridor_gate
                  # is the FIRST leg)
    'c2n11_appr_r1': (-2.6057, -0.1229, 0.3212),
    'c2n11_appr_r2': (-2.5999, -0.1519, 0.5090),
    'c2n11_appr_r3': (-2.6033, -0.1583, 0.4851),
}
TOUR_ENTRY = {    # C2-NAV.12, full seven-leg tour (obstacle_corner ->
                  # corridor_gate is the FIFTH leg)
    'c2n12_tour_r1': (-2.5636, -0.0942, -0.2853),
    'c2n12_tour_r2': (-2.6132, -0.0423, -0.4404),
    'c2n12_tour_r3': (-2.6312, -0.0823, -0.5036),
}


def hdr(t):
    print()
    print('=' * 78)
    print(t)
    print('=' * 78)


def trace_path(tag, leg):
    return os.path.join(TRACE_ROOT, f'{tag}_traces', f'{leg}_rep0.csv')


def load_trace(tag, leg):
    p = trace_path(tag, leg)
    if not os.path.exists(p):
        return None
    rows = []
    with open(p) as f:
        for r in csv.DictReader(f):
            if r['x'] in (None, ''):
                continue
            rows.append({
                't': float(r['t_rel']), 'x': float(r['x']), 'y': float(r['y']),
                'yaw': float(r['yaw']),
                'cm_action': int(r['cm_action']) if r['cm_action'] not in (None, '') else None,
            })
    return rows


def bearing_deg(x, y, tx, ty):
    return math.degrees(math.atan2(ty - y, tx - x))


def ang_diff_deg(a, b):
    """Smallest signed difference a-b, wrapped to [-180, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


# ---------------------------------------------------------------------
# 1. SELF-TEST
# ---------------------------------------------------------------------

def self_test():
    hdr('SELF-TEST: reproduce committed C2-NAV.9/C2-NAV.12 numbers before '
        'trusting anything new')
    ok = True

    # (a) SW corner geometry, C2-NAV.9's own.
    x0, x1, y0, y1 = rect(BOX1)
    sw = (x0, y0)
    print(f'  box_obstacle_1 SW corner (from BOXES): {sw}  want {SW_CORNER}')
    p1 = abs(sw[0] - SW_CORNER[0]) < 1e-9 and abs(sw[1] - SW_CORNER[1]) < 1e-9
    print(f'    {"PASS" if p1 else "FAIL"}')
    ok &= p1

    # (b) C2-NAV.9's 326.0 mm whole-corridor bottleneck (r1 corridor_gate-end
    # start -> ENCLOSURE_ENTRY_SHIFTED), reused by import of the same
    # build_clearance_grid/bottleneck functions C2-NAV.9 committed.
    xs, ys, clr = build_clearance_grid()
    tau = bottleneck(clr, xs, ys, (-2.6162, 0.0325), GOAL_SHIFTED)
    print(f'  whole-corridor bottleneck (r1 corridor_gate-end start): '
          f'{tau*1000:.1f} mm  want ~326.0 mm')
    p2 = abs(tau * 1000 - 326.0) < 2.0
    print(f'    {"PASS" if p2 else "FAIL"}')
    ok &= p2

    # (c) C2-NAV.12's own reported nearest-waypoint distances, recomputed
    # from the raw GT trace this session still has on disk.
    want = {'c2n12_tour_r1': 0.551, 'c2n12_tour_r2': 0.293, 'c2n12_tour_r3': 0.006}
    print()
    print('  nearest approach to WAYPOINT, recomputed from raw trace vs. '
        'C2-NAV.12\'s committed record:')
    any_trace = False
    for tag, want_d in want.items():
        rows = load_trace(tag, 'enclosure_entry')
        if rows is None:
            print(f'    {tag}: NO TRACE ON DISK (.navbench/ is scratch, may '
                  'not survive a clone) -- skipped, not failed')
            continue
        any_trace = True
        dmin = min(math.dist((r['x'], r['y']), WAYPOINT) for r in rows)
        p = abs(dmin - want_d) < 0.01
        print(f'    {tag}: {dmin:.3f} m  want {want_d:.3f} m  '
              f'{"PASS" if p else "FAIL"}')
        ok &= p
    if not any_trace:
        print('  !! no raw traces found -- (c) not exercised this run')

    print()
    print(f'  SELF-TEST: {"ALL PASS" if ok else "FAILED -- STOP, fix the tool"}')
    if not ok:
        raise SystemExit(1)
    return ok


# ---------------------------------------------------------------------
# 2. APPROACH-STATE RECONSTRUCTION
# ---------------------------------------------------------------------

def _state_row(tag, x, y, yaw):
    b_wp = bearing_deg(x, y, *WAYPOINT)
    b_goal = bearing_deg(x, y, *GOAL_SHIFTED)
    b_sw = bearing_deg(x, y, *SW_CORNER)
    yaw_deg = math.degrees(yaw)
    d_wp = math.dist((x, y), WAYPOINT)
    d_goal = math.dist((x, y), GOAL_SHIFTED)
    d_box, q_box = dist_to_box(x, y, BOX1)
    return dict(
        tag=tag, x=x, y=y, yaw_rad=yaw, yaw_deg=yaw_deg,
        bearing_to_waypoint_deg=b_wp, dev_to_waypoint_deg=ang_diff_deg(b_wp, yaw_deg),
        dist_to_waypoint_m=d_wp,
        bearing_to_goal_deg=b_goal, dev_to_goal_deg=ang_diff_deg(b_goal, yaw_deg),
        dist_to_goal_m=d_goal,
        bearing_to_sw_corner_deg=b_sw, dev_to_sw_corner_deg=ang_diff_deg(b_sw, yaw_deg),
        dist_to_box_m=d_box, box_nearest_point=q_box,
    )


def approach_states():
    hdr('ENTRY-STATE RECONSTRUCTION: pose, heading, and bearings at the '
        'start of enclosure_entry (== end of corridor_gate)')
    print('  bearings: 0 deg = world +X (east), 90 = world +Y (north). '
          '"dev_to_X" = bearing to X minus current yaw, wrapped to '
          '[-180, 180] -- how far the robot would have to turn to point '
          'straight at X.')
    rows = {}
    for group, table in (('FRESH (C2-NAV.11)', FRESH_ENTRY),
                          ('TOUR (C2-NAV.12)', TOUR_ENTRY)):
        hdr(group)
        for tag, (x, y, yaw) in table.items():
            r = _state_row(tag, x, y, yaw)
            rows[tag] = r
            print(f'  {tag}')
            print(f'    pose (x,y)            : ({x:+.4f}, {y:+.4f})')
            print(f'    yaw                    : {yaw:+.4f} rad '
                  f'({r["yaw_deg"]:+.1f} deg)')
            print(f'    -> waypoint {WAYPOINT}: bearing {r["bearing_to_waypoint_deg"]:+.1f} deg, '
                  f'dev {r["dev_to_waypoint_deg"]:+.1f} deg, dist {r["dist_to_waypoint_m"]:.3f} m')
            print(f'    -> final goal {GOAL_SHIFTED}: bearing {r["bearing_to_goal_deg"]:+.1f} deg, '
                  f'dev {r["dev_to_goal_deg"]:+.1f} deg, dist {r["dist_to_goal_m"]:.3f} m')
            print(f'    -> SW corner {SW_CORNER}: bearing {r["bearing_to_sw_corner_deg"]:+.1f} deg, '
                  f'dev {r["dev_to_sw_corner_deg"]:+.1f} deg')
            print(f'    -> box_obstacle_1 clearance: {r["dist_to_box_m"]:.3f} m '
                  f'to {r["box_nearest_point"]}')
            print()

    hdr('FRESH vs TOUR, same corridor_gate-end pose, mirrored yaw sign')
    print(f'  {"pair":<10}{"fresh yaw":>12}{"tour yaw":>12}{"dev_wp fresh":>16}{"dev_wp tour":>16}')
    for i in (1, 2, 3):
        fk, tk = f'c2n11_appr_r{i}', f'c2n12_tour_r{i}'
        fr, tr = rows[fk], rows[tk]
        print(f'  r{i:<9}{fr["yaw_deg"]:>10.1f}d {tr["yaw_deg"]:>10.1f}d '
              f'{fr["dev_to_waypoint_deg"]:>14.1f}d {tr["dev_to_waypoint_deg"]:>14.1f}d')
    print()
    print('  If dev_to_waypoint is small for fresh and large (or the wrong')
    print('  sign) for tour, the reversed heading points the robot away from')
    print('  the waypoint before the leg has moved at all -- a heading effect')
    print('  present at t=0, before any waypoint-removal mechanism can act.')
    return rows


# ---------------------------------------------------------------------
# 3. HEADING SENSITIVITY (geometric, offline)
# ---------------------------------------------------------------------

def heading_sensitivity():
    hdr('CLEARANCE-VS-HEADING SENSITIVITY (fixed base position, sweep yaw)')
    print('  PolygonStop is a CIRCLE centred on base_footprint: its distance')
    print('  to any fixed obstacle point does not depend on robot yaw at all.')
    print('  PolygonSlow/PolygonLimit are SQUARES: their reach along a given')
    print('  bearing varies with yaw via square_reach() (closed-form).')
    print()

    test_points = {
        'r1 corridor_gate-end (fresh)': FRESH_ENTRY['c2n11_appr_r1'][:2],
        'r1 corridor_gate-end (tour)': TOUR_ENTRY['c2n12_tour_r1'][:2],
        'SW corner + 50mm margin pose': (SW_CORNER[0] - 0.05, SW_CORNER[1] - 0.05),
    }
    for name, (px, py) in test_points.items():
        hdr(name + f'  ({px:+.3f}, {py:+.3f})')
        d_box, q_box = dist_to_box(px, py, BOX1)
        print(f'  distance to box_obstacle_1: {d_box:.4f} m to {q_box} '
              '(YAW-INDEPENDENT -- a point-to-rectangle distance has no '
              'heading term)')
        bearing_to_box = math.atan2(q_box[1] - py, q_box[0] - px)
        print(f'  PolygonStop (r={POLY_STOP_R}): triggered whenever d_box < '
              f'{POLY_STOP_R} m, i.e. {"YES" if d_box < POLY_STOP_R else "NO"} '
              'at this point, for EVERY heading (circle).')
        print(f'  PolygonSlow/Limit reach toward the box bearing '
              f'({math.degrees(bearing_to_box):+.1f} deg from this point) as '
              'a function of the SQUARE\'s own facing (robot yaw):')
        for yaw_deg in range(-60, 61, 20):
            yaw = math.radians(yaw_deg)
            # bearing of the box point in the robot's body frame at this yaw
            bx, by = body_frame(q_box[0], q_box[1], px, py, yaw)
            bearing_body = math.atan2(by, bx)
            reach_slow = square_reach(bearing_body, POLY_SLOW_HW)
            reach_limit = square_reach(bearing_body, POLY_LIMIT_HW)
            print(f'    yaw {yaw_deg:+4d} deg: PolygonSlow reach '
                  f'{reach_slow:.4f} m (triggered={d_box < reach_slow}), '
                  f'PolygonLimit reach {reach_limit:.4f} m '
                  f'(triggered={d_box < reach_limit})')
        print()

    hdr('CONCLUSION')
    print('  Heading cannot change whether PolygonStop is triggered AT A GIVEN')
    print('  POINT -- that circle\'s test is d < 0.25 m, full stop, no yaw term.')
    print('  So if a heading effect exists at all, it must act by changing')
    print('  WHICH POINTS the trajectory passes through (DWB path selection),')
    print('  not by changing the clearance test at a fixed point. This is')
    print('  consistent with C2-NAV.9\'s own finding that PolygonStop reads')
    print('  0%/100%, never partial, across a yaw sweep at one point.')


# ---------------------------------------------------------------------
# 4. WAYPOINT-REMOVAL TIMELINE
# ---------------------------------------------------------------------

def _rate_ticks(t_max, period=RATE_PERIOD):
    ticks = []
    t = 0.0
    while t <= t_max + 1e-9:
        ticks.append(t)
        t += period
    return ticks


def _distance_series(rows, target):
    return [(r['t'], math.dist((r['x'], r['y']), target), r['cm_action']) for r in rows]


def waypoint_timeline(tags=None):
    hdr('WAYPOINT-REMOVAL TIMELINE: distance(t) to WAYPOINT '
        f'{WAYPOINT}, RemovePassedGoals radius={RPG_RADIUS} m, '
        f'RateController {RATE_HZ} Hz (period {RATE_PERIOD:.3f} s)')
    tags = tags or list(TOUR_ENTRY.keys()) + list(FRESH_ENTRY.keys())
    out = {}
    for tag in tags:
        rows = load_trace(tag, 'enclosure_entry')
        if rows is None:
            print(f'  {tag}: no trace on disk, skipped')
            continue
        series = _distance_series(rows, WAYPOINT)
        dmin, tmin = min((d, t) for (t, d, _) in series)
        # first sim time the distance is <= radius
        first_cross = next((t for (t, d, _) in series if d <= RPG_RADIUS), None)
        ticks = _rate_ticks(series[-1][0])
        removal_tick = next((tk for tk in ticks if any(
            t <= tk and d <= RPG_RADIUS for (t, d, _) in series if t <= tk)), None)
        # distance AT the removal tick (nearest sample <= tick)
        d_at_removal = None
        if removal_tick is not None:
            before = [(t, d) for (t, d, _) in series if t <= removal_tick]
            if before:
                d_at_removal = before[-1][1]
        reached = dmin <= 0.25   # goal_xy_tolerance, for comparison only
        # count sign crossings of the 0.7 m threshold (monotonic approach?)
        crossings = 0
        prev_above = series[0][1] > RPG_RADIUS
        for (_, d, _c) in series[1:]:
            above = d > RPG_RADIUS
            if above != prev_above:
                crossings += 1
            prev_above = above
        print(f'  {tag}:')
        print(f'    nearest approach   : {dmin:.4f} m at t={tmin:.2f} s '
              f'({"REACHED" if reached else "NOT reached"} the 0.25 m '
              'goal_xy_tolerance)')
        print(f'    first < 0.7 m at   : '
              f'{("t="+format(first_cross, ".2f")+" s") if first_cross is not None else "NEVER"}')
        print(f'    threshold crossings: {crossings} '
              '(1 == monotonic single approach, no bouncing back out)')
        if removal_tick is not None:
            print(f'    quantized removal at RateController tick t={removal_tick:.3f} s '
                  f'(distance there ~= {d_at_removal:.4f} m)')
        else:
            print('    quantized removal  : NEVER (waypoint stays in {goals} '
                  'the whole leg)')
        out[tag] = dict(nearest_m=dmin, nearest_t_s=tmin, first_cross_t_s=first_cross,
                         removal_tick_s=removal_tick, dist_at_removal_m=d_at_removal,
                         crossings=crossings, reached_tolerance=reached)
    return out


# ---------------------------------------------------------------------
# 5. DIVERGENCE TIMING: does the SW-corner-side commitment predate removal?
# ---------------------------------------------------------------------

def divergence_timing(tags=None):
    hdr('SW-CORNER COMMITMENT TIMING vs. WAYPOINT-REMOVAL TICK')
    x0, x1, y0, y1 = rect(BOX1)
    print('  "committed toward the SW-corner side" is operationalised as: the')
    print('  GT track enters the WEST-side approach column (x < x0 + 0.15 == '
          f'{x0 + 0.15:.2f}, west of the box\'s own west face x0={x0:.2f} -- '
          'this is the discriminator that matters: the box\'s EAST face '
          f'(x1={x1:.2f}) is a DIFFERENT, unrelated pocket, as r1 turned out '
          'to prove) while south of the box\'s south edge '
          f'y0={y0:.2f} and within 0.60 m of the box (comfortably inside the '
          'approach funnel, well before PolygonSlow (max reach 0.566 m) or '
          'PolygonStop (0.25 m) could possibly react).')
    print('  NOTE: an earlier version of this check used only "y < y0", which')
    print('  misclassified r1 (frozen EAST of the box, x=-2.486, y=2.274 -- ')
    print('  inside the box\'s own y-span, the opposite side) as an SW-corner')
    print('  approach. Fixed by adding the x < x0+0.15 m west-column test, ')
    print('  cross-checked directly against r1\'s logged frozen pose below.')
    tags = tags or list(TOUR_ENTRY.keys()) + list(FRESH_ENTRY.keys())
    out = {}
    for tag in tags:
        rows = load_trace(tag, 'enclosure_entry')
        if rows is None:
            print(f'  {tag}: no trace on disk, skipped')
            continue
        commit_t = None
        for r in rows:
            d_box, _q = dist_to_box(r['x'], r['y'], BOX1)
            if d_box < 0.60 and r['y'] < y0 and r['x'] < x0 + 0.15:
                commit_t = r['t']
                break
        wp = waypoint_timeline([tag])
        rt = wp.get(tag, {}).get('removal_tick_s')
        print(f'  {tag}: SW-side-commit at '
              f'{("t="+format(commit_t, ".2f")+"s") if commit_t is not None else "NEVER"}'
              f', waypoint-removal tick at '
              f'{("t="+format(rt, ".2f")+"s") if rt is not None else "NEVER"}')
        if commit_t is not None and rt is not None:
            order = 'BEFORE' if commit_t < rt else ('AFTER' if commit_t > rt else 'SAME TICK')
            print(f'    -> SW-side commitment happens {order} waypoint removal')
        out[tag] = dict(sw_commit_t_s=commit_t, removal_tick_s=rt)
    return out


# ---------------------------------------------------------------------
# 6. COUNTERFACTUAL THRESHOLD SENSITIVITY
# ---------------------------------------------------------------------

def counterfactual():
    hdr('COUNTERFACTUAL: RemovePassedGoals.radius SENSITIVITY (NOT a '
        'recommendation -- characterizing intervals only, per C2-NAV.13 '
        'section 6/10/11)')
    wp = waypoint_timeline()
    tour = {k: v for k, v in wp.items() if k.startswith('c2n12_tour')}
    fresh = {k: v for k, v in wp.items() if k.startswith('c2n11_appr')}
    print('  For each run, the waypoint is removed at the first '
          'RateController tick where distance <= radius. A run\'s nearest')
    print('  approach is therefore the exact boundary between "removed at')
    print('  some point in the leg" and "never removed": radius >= nearest')
    print('  removes it eventually; radius < nearest preserves it for the')
    print('  whole leg (given each run\'s own trajectory is unchanged --')
    print('  this does NOT predict what a smaller radius would make the')
    print('  ROBOT do differently, only what it would do to THIS recorded')
    print('  trajectory, which is the only thing offline analysis can say).')
    print()
    for tag, v in {**tour, **fresh}.items():
        d = v['nearest_m']
        print(f'  {tag}: nearest={d:.3f} m -> preserved for the WHOLE leg at '
              f'radius < {d:.3f} m; removed at SOME tick (at 0.7 m, the '
              f'installed radius, that tick is t={v["removal_tick_s"]:.2f}s) '
              f'for radius >= {d:.3f} m. A larger radius only ever removes '
              f'EARLIER, never later, than the 0.7 m tick shown.')
    print()
    print('  Tour r1 (0.551) and r2 (0.293) both fall UNDER the installed '
          '0.7 m -- both would be PRESERVED by any radius in (0, 0.293) and')
    print('  r1 alone would be preserved by any radius in [0.293, 0.551).')
    print('  r3 (0.006) is removed by any radius that is not near-zero -- ')
    print('  its outcome is insensitive to the radius because it genuinely')
    print('  reached the waypoint.')
    print('  All three FRESH (C2-NAV.11) runs are reported in RESULTS.md as')
    print('  0% PolygonStop with early-plan continuity toward the FINAL goal')
    print('  already -- check whether they even come within 0.7 m of the')
    print('  waypoint at all (a fresh two-leg start\'s "waypoint" leg was')
    print('  spliced differently under C2-NAV.10; under C2-NAV.11\'s genuine')
    print('  through-poses the waypoint plays the same corridor-shaping role).')


# ---------------------------------------------------------------------
# main
# ---------------------------------------------------------------------

def dump(out_path):
    doc = {
        'experiment': 'C2-NAV.13 offline diagnosis',
        'waypoint_world': list(WAYPOINT),
        'goal_shifted_world': list(GOAL_SHIFTED),
        'sw_corner_world': list(SW_CORNER),
        'remove_passed_goals_radius_m': RPG_RADIUS,
        'rate_controller_hz': RATE_HZ,
        'fresh_entry_states': {k: _state_row(k, *v) for k, v in FRESH_ENTRY.items()},
        'tour_entry_states': {k: _state_row(k, *v) for k, v in TOUR_ENTRY.items()},
        'waypoint_timeline': waypoint_timeline(),
        'divergence_timing': divergence_timing(),
    }
    with open(out_path, 'w') as f:
        json.dump(doc, f, indent=2, default=str)
    print(f'wrote {out_path}')


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if cmd == 'self_test':
        self_test()
    elif cmd == 'states':
        approach_states()
    elif cmd == 'heading':
        heading_sensitivity()
    elif cmd == 'timeline':
        waypoint_timeline()
    elif cmd == 'divergence':
        divergence_timing()
    elif cmd == 'counterfactual':
        counterfactual()
    elif cmd == 'dump':
        dump(sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, 'c2nav13_bench.json'))
    elif cmd == 'all':
        self_test()
        approach_states()
        heading_sensitivity()
        waypoint_timeline()
        divergence_timing()
        counterfactual()
    else:
        print(__doc__)
        raise SystemExit(1)


if __name__ == '__main__':
    main()
