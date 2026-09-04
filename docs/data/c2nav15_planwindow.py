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
"""C2-NAV.15: mid-leg global-plan geometry diagnosis.

C2-NAV.14 rejected the heading-correcting through-pose and left one
question open, unanswered by every prior C2-NAV.9 through .14 session:
does the SmacPlanner2D global plan itself bend toward box_obstacle_1's
SW corner during the `enclosure_entry` approach, or does the global plan
stay inside C2-NAV.9's own 326 mm feasible corridor while DWB's local
sampling diverges from it? `nav_bench.py`'s `early_plan` capture (C2-NAV.11)
only ever recorded the FIRST `/plan` message's endpoint, never the
geometry of any later message -- this is the gap every C2-NAV.9/.10/.13/.14
session flagged as NOT PROVEN.

This module changes NOTHING navigational. It is pure offline analysis of
a NEW artifact nav_bench.py now writes (`<tag>_planwindow_<leg>_rep<rep>.json`,
C2-NAV.15's own instrumentation, see `send_multi_leg`'s docstring): the
FULL polyline of every `/plan` message across the whole `enclosure_entry`
leg, not just the first one. The installed through-poses BT wraps BOTH
`ComputePathThroughPoses` and `RemovePassedGoals` in the SAME
`RateController hz="0.333"` (confirmed from the installed XML), so every
plan snapshot IS a post-RemovePassedGoals-tick replan -- there is no
separate "before/after" capture to build; the tick boundary and the plan
boundary are the same event by construction.

Reuses BY IMPORT, not restates:
  - c2nav9_corridor.py:  BOXES, STOP_RADIUS/POLY_STOP_R, CIRCUMSCRIBED,
                          build_clearance_grid, bottleneck (the 326 mm
                          widest-path figure), CORRIDOR_GATE_GOAL
  - c2nav8_report.py:    nearest_full (whole-world clearance, not just
                          the 8-box list)
  - c2nav12_report.py:   WAYPOINT, GOAL_SHIFTED, SW_CORNER, DEADLOCK_POSE
  - c2nav13_heading.py:  BOX1, rect(BOX1), RATE_PERIOD, RPG_RADIUS,
                          trace_path/load_trace, _rate_ticks,
                          _distance_series, waypoint_timeline,
                          divergence_timing -- the removal-tick and
                          SW-column-commit machinery, applied to the NEW
                          run by tag rather than reimplemented
  - c2nav14_heading_pose.py: HEADING_POSE

Two data sources for the NEW run (c2n15_tour_r1), both local scratch,
`.navbench/` never tracked in this repo:
  1. `.navbench/results/c2n15_tour_r1_planwindow_enclosure_entry_rep0.json`
     -- the new full-plan-geometry capture.
  2. `.navbench/results/c2n15_tour_r1_traces/enclosure_entry_rep0.csv`
     -- the raw 0.1 s ground-truth trace (same shape every prior session
     read via c2nav13_heading.load_trace).
Every number this script derives is written to the committed
`docs/data/c2nav15_bench.json` so the finding survives even though the
raw artifacts might not.

Usage:
  python3 c2nav15_planwindow.py selftest
  python3 c2nav15_planwindow.py snapshots   # per-/plan-message geometry table
  python3 c2nav15_planwindow.py timeline    # snapshots x GT x removal ticks, merged
  python3 c2nav15_planwindow.py firstbad    # FIRST_BAD_PLAN
  python3 c2nav15_planwindow.py classify    # CASE A vs CASE B
  python3 c2nav15_planwindow.py dump <out.json>
  python3 c2nav15_planwindow.py all
"""
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c2nav9_corridor import (                                # noqa: E402
    BOXES, CIRCUMSCRIBED, STOP_RADIUS, POLY_STOP_R, FOOT_CIRC_R,
    dist_to_box, rect, build_clearance_grid, bottleneck,
    CORRIDOR_GATE_GOAL, GRID_X, GRID_Y,
)
from c2nav8_report import nearest_full                        # noqa: E402
from c2nav12_report import (                                  # noqa: E402
    WAYPOINT, GOAL_SHIFTED, SW_CORNER, DEADLOCK_POSE,
)
from c2nav13_heading import (                                 # noqa: E402
    BOX1, RATE_PERIOD, RPG_RADIUS, trace_path, load_trace,
    _rate_ticks, _distance_series, waypoint_timeline, divergence_timing,
)
from c2nav14_heading_pose import HEADING_POSE                 # noqa: E402

REPO_ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
RESULTS_DIR = os.path.join(REPO_ROOT, '.navbench', 'results')
TAG = 'c2n15_tour_r1'
LEG = 'enclosure_entry'
NW_CORNER = (-3.25, 2.65)   # box_obstacle_1 NW corner, c2nav9_corridor.geometry()
BOX1_X0, BOX1_X1, BOX1_Y0, BOX1_Y1 = rect(BOX1)
assert (BOX1_X0, BOX1_Y0) == SW_CORNER, 'SW_CORNER constant drifted from BOX1 rect'
assert (BOX1_X0, BOX1_Y1) == NW_CORNER, 'NW_CORNER constant drifted from BOX1 rect'

# C2-NAV.14's own committed frozen pose (RESULTS.md C2-NAV.14 section 7),
# quoted verbatim for the self-test and for direct comparison below.
C2NAV14_FROZEN = (-3.332, 1.919)
C2NAV14_FROZEN_DIST_TO_DEADLOCK_M = 0.0328


def hdr(t):
    print()
    print('=' * 78)
    print(t)
    print('=' * 78)


# ---------------------------------------------------------------------
# 0. SELF-TEST -- reproduce known facts before trusting anything new
#    (brief section 20: tool validation).
# ---------------------------------------------------------------------

def self_test():
    hdr('SELF-TEST: reproduce known committed facts before trusting '
        'anything new')
    ok = True

    xs, ys, clr = build_clearance_grid()
    tau = bottleneck(clr, xs, ys, CORRIDOR_GATE_GOAL, GOAL_SHIFTED)
    print(f'  whole-corridor bottleneck (corridor_gate -> goal): '
          f'{tau * 1000:.1f} mm  want ~326.0 mm  '
          f'{"PASS" if abs(tau * 1000 - 326.0) < 1.0 else "FAIL"}')
    ok &= abs(tau * 1000 - 326.0) < 1.0

    print(f'  box_obstacle_1 SW corner: {SW_CORNER}  want (-3.25, 2.15)  '
          f'{"PASS" if SW_CORNER == (-3.25, 2.15) else "FAIL"}')
    ok &= SW_CORNER == (-3.25, 2.15)

    print(f'  box_obstacle_1 NW corner: {NW_CORNER}  want (-3.25, 2.65)  '
          f'{"PASS" if NW_CORNER == (-3.25, 2.65) else "FAIL"}')
    ok &= NW_CORNER == (-3.25, 2.65)

    d = math.dist(C2NAV14_FROZEN, DEADLOCK_POSE)
    print(f'  C2-NAV.14 frozen pose {C2NAV14_FROZEN} vs C2-NAV.8/.12 '
          f'DEADLOCK_POSE {DEADLOCK_POSE}: {d * 1000:.1f} mm  '
          f'want ~32.8 mm  '
          f'{"PASS" if abs(d - C2NAV14_FROZEN_DIST_TO_DEADLOCK_M) < 0.001 else "FAIL"}')
    ok &= abs(d - C2NAV14_FROZEN_DIST_TO_DEADLOCK_M) < 0.001

    print(f'  WAYPOINT: {WAYPOINT}  want (-3.40, 1.35)  '
          f'{"PASS" if WAYPOINT == (-3.40, 1.35) else "FAIL"}')
    ok &= WAYPOINT == (-3.40, 1.35)

    print(f'  HEADING_POSE: {HEADING_POSE}  want (-3.00, 0.625)  '
          f'{"PASS" if HEADING_POSE == (-3.00, 0.625) else "FAIL"}')
    ok &= HEADING_POSE == (-3.00, 0.625)

    print(f'  GOAL_SHIFTED: {GOAL_SHIFTED}  want (-3.575, 2.95)  '
          f'{"PASS" if GOAL_SHIFTED == (-3.575, 2.95) else "FAIL"}')
    ok &= GOAL_SHIFTED == (-3.575, 2.95)

    d_stop, who, _q = nearest_full(*DEADLOCK_POSE)[0]
    print(f'  nearest_full(DEADLOCK_POSE) = {d_stop * 1000:.1f} mm to '
          f'{who}  want < 250 mm (inside PolygonStop)  '
          f'{"PASS" if d_stop < STOP_RADIUS else "FAIL"}')
    ok &= d_stop < STOP_RADIUS

    print()
    print('SELF-TEST: ALL PASS' if ok else 'SELF-TEST: FAILURE -- DO NOT '
          'TRUST ANYTHING BELOW')
    return ok


# ---------------------------------------------------------------------
# 1. LOAD the new plan-window capture + the matching GT trace.
# ---------------------------------------------------------------------

def planwindow_path(tag=TAG, leg=LEG, rep=0):
    return os.path.join(RESULTS_DIR, f'{tag}_planwindow_{leg}_rep{rep}.json')


def load_planwindow(tag=TAG, leg=LEG, rep=0):
    p = planwindow_path(tag, leg, rep)
    if not os.path.exists(p):
        return None
    with open(p) as f:
        return json.load(f)


def bench_json(tag=TAG):
    p = os.path.join(RESULTS_DIR, f'{tag}.json')
    if not os.path.exists(p):
        return None
    with open(p) as f:
        return json.load(f)


# ---------------------------------------------------------------------
# 2. PER-SNAPSHOT GEOMETRY -- the actual /plan polyline analysis.
# ---------------------------------------------------------------------

def analyze_snapshot(snap):
    """One /plan message's full polyline (world frame). Computes, over
    every point on the polyline (not just the endpoint C2-NAV.11/.14
    captured):
      - min clearance to ANY world obstacle (nearest_full, whole world)
      - whether that dips inside PolygonStop's own 0.25 m radius
      - closest approach to the SW / NW corners specifically
      - whether the plan itself ever enters the SW-side approach column
        (x < BOX1_X0+0.15, y < BOX1_Y0, within 0.60 m of the box) --
        EXACTLY C2-NAV.13's own operational test for the ROBOT's GT
        track, applied here to the PLAN's own polyline, so the two are
        directly comparable (this is the Case A/B discriminator).
      - path length and a curvature proxy (sum of absolute heading
        change between consecutive segments).
    """
    pts = snap['poses_world']
    out = dict(ts_sim_s=snap['ts_sim_s'],
               ts_offset_from_t0_s=snap['ts_offset_from_t0_s'],
               n_poses=len(pts))
    if not pts:
        out.update(min_clearance_m=None, enters_polygon_stop=None,
                    closest_to_sw_corner_m=None, closest_to_nw_corner_m=None,
                    plan_enters_sw_column=None, path_length_m=0.0,
                    curvature_rad=0.0, first_pose=None, last_pose=None)
        return out

    out['first_pose'] = [round(pts[0][0], 3), round(pts[0][1], 3)]
    out['last_pose'] = [round(pts[-1][0], 3), round(pts[-1][1], 3)]

    clearances = [nearest_full(x, y)[0] for (x, y) in pts]
    d_min, who, q = min(clearances, key=lambda c: c[0])
    out['min_clearance_m'] = round(d_min, 4)
    out['min_clearance_obstacle'] = who
    out['min_clearance_at'] = [round(q[0], 3), round(q[1], 3)]
    out['enters_polygon_stop'] = d_min < POLY_STOP_R

    out['closest_to_sw_corner_m'] = round(
        min(math.dist((x, y), SW_CORNER) for (x, y) in pts), 4)
    out['closest_to_nw_corner_m'] = round(
        min(math.dist((x, y), NW_CORNER) for (x, y) in pts), 4)

    sw_col = False
    for (x, y) in pts:
        d_box, _q2 = dist_to_box(x, y, BOX1)
        if d_box < 0.60 and y < BOX1_Y0 and x < BOX1_X0 + 0.15:
            sw_col = True
            break
    out['plan_enters_sw_column'] = sw_col

    length = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    out['path_length_m'] = round(length, 4)

    curvature = 0.0
    if len(pts) >= 3:
        bearings = [math.atan2(pts[i + 1][1] - pts[i][1],
                                pts[i + 1][0] - pts[i][0])
                    for i in range(len(pts) - 1)]

        def wrap(a):
            return (a + math.pi) % (2 * math.pi) - math.pi
        curvature = sum(abs(wrap(bearings[i + 1] - bearings[i]))
                         for i in range(len(bearings) - 1))
    out['curvature_rad'] = round(curvature, 3)
    return out


def snapshots(tag=TAG, quiet=False):
    pw = load_planwindow(tag)
    if pw is None:
        if not quiet:
            print(f'no plan-window capture for {tag} ({planwindow_path(tag)})')
        return None, []
    analyzed = [analyze_snapshot(s) for s in pw['snapshots']]
    if not quiet:
        hdr(f'{tag}: {len(analyzed)} /plan snapshots across the whole '
            f'{LEG} leg (t0={pw["t0_sim_s"]}s, t1={pw["t1_sim_s"]}s, '
            f'through_poses={pw["through_poses_world"]}, '
            f'goal={pw["final_goal_world"]})')
        print(f'  {"t_offset":>9} {"n":>4} {"len_m":>7} {"min_clr_m":>9} '
              f'{"obstacle":<16} {"SW_corner_m":>11} {"in_STOP":>8} '
              f'{"SW_column":>9} {"curv_rad":>9}')
        for a in analyzed:
            print(f'  {a["ts_offset_from_t0_s"]:>9.3f} {a["n_poses"]:>4} '
                  f'{a["path_length_m"]:>7.3f} '
                  f'{_fmt(a["min_clearance_m"])!s:>9} '
                  f'{str(a.get("min_clearance_obstacle")):<16} '
                  f'{_fmt(a["closest_to_sw_corner_m"])!s:>11} '
                  f'{str(a["enters_polygon_stop"]):>8} '
                  f'{str(a["plan_enters_sw_column"]):>9} '
                  f'{a["curvature_rad"]:>9.3f}')
    return pw, analyzed


def _fmt(x, nd=4):
    return 'None' if x is None else round(x, nd)


# ---------------------------------------------------------------------
# 3. GT TRACE CORRELATION + REMOVAL-TICK TIMELINE (reused, not restated).
# ---------------------------------------------------------------------

def heading_pose_timeline(tag=TAG):
    """Same shape as c2nav13_heading.waypoint_timeline, but for
    HEADING_POSE -- that function is hardcoded to WAYPOINT, so this is
    the smallest generalisation, built the same way, not a rewrite."""
    rows = load_trace(tag, LEG)
    if rows is None:
        return None
    series = _distance_series(rows, HEADING_POSE)
    dmin, tmin = min((d, t) for (t, d, _c) in series)
    first_cross = next((t for (t, d, _c) in series if d <= RPG_RADIUS), None)
    ticks = _rate_ticks(series[-1][0])
    removal_tick = next((tk for tk in ticks if any(
        t <= tk and d <= RPG_RADIUS for (t, d, _c) in series if t <= tk)),
        None)
    d_at_removal = None
    if removal_tick is not None:
        before = [(t, d) for (t, d, _c) in series if t <= removal_tick]
        if before:
            d_at_removal = before[-1][1]
    return dict(nearest_m=round(dmin, 4), nearest_t_s=round(tmin, 3),
                first_cross_t_s=(round(first_cross, 3)
                                  if first_cross is not None else None),
                removal_tick_s=(round(removal_tick, 3)
                                 if removal_tick is not None else None),
                dist_at_removal_m=(round(d_at_removal, 4)
                                    if d_at_removal is not None else None))


def merged_timeline(tag=TAG):
    """The brief's own required output (section 10): time, robot pose,
    plan version/min-clearance, RemovePassedGoals state, for one table."""
    pw, analyzed = snapshots(tag, quiet=True)
    if pw is None:
        return None
    rows = load_trace(tag, LEG)
    wp_tl = waypoint_timeline([tag])
    hp_tl = heading_pose_timeline(tag)
    div = divergence_timing([tag])

    hdr(f'{tag}: MERGED TIMELINE -- plan snapshot x robot pose x removal ticks')
    print(f'  HEADING_POSE removal tick: '
          f'{hp_tl.get("removal_tick_s") if hp_tl else None} s '
          f'(dist there {hp_tl.get("dist_at_removal_m") if hp_tl else None} m)')
    wp = wp_tl.get(tag, {})
    print(f'  WAYPOINT removal tick    : {wp.get("removal_tick_s")} s '
          f'(dist there {wp.get("dist_at_removal_m")} m)')
    dv = div.get(tag, {})
    print(f'  GT track SW-column commit: {dv.get("sw_commit_t_s")} s')
    print()
    print(f'  {"t_offset":>9} {"robot_xy":<22} {"robot_yaw":>10} '
          f'{"plan_min_clr":>13} {"plan_SW_col":>11} {"cm_action":>10}')
    merged = []
    for a in analyzed:
        t = a['ts_offset_from_t0_s']
        gt_row = None
        if rows:
            best = min(rows, key=lambda r: abs(r['t'] - t))
            if abs(best['t'] - t) < 1.0:
                gt_row = best
        row = dict(a)
        if gt_row:
            row['robot_xy'] = [round(gt_row['x'], 3), round(gt_row['y'], 3)]
            row['robot_yaw_deg'] = round(math.degrees(gt_row['yaw']), 1)
            row['cm_action'] = gt_row['cm_action']
            xy_s = f'({gt_row["x"]:+.3f},{gt_row["y"]:+.3f})'
            print(f'  {t:>9.3f} {xy_s:<22} '
                  f'{math.degrees(gt_row["yaw"]):>10.1f} '
                  f'{_fmt(a["min_clearance_m"])!s:>13} '
                  f'{str(a["plan_enters_sw_column"]):>11} '
                  f'{str(gt_row["cm_action"]):>10}')
        else:
            print(f'  {t:>9.3f} {"(no GT sample)":<22} {"":>10} '
                  f'{_fmt(a["min_clearance_m"])!s:>13} '
                  f'{str(a["plan_enters_sw_column"]):>11} {"":>10}')
        merged.append(row)
    return dict(heading_pose_tick=hp_tl, waypoint_tick=wp, divergence=dv,
                merged=merged)


# ---------------------------------------------------------------------
# 4. FIRST_BAD_PLAN (brief section 8, the single most important output).
# ---------------------------------------------------------------------

def first_bad_plan(tag=TAG):
    pw, analyzed = snapshots(tag, quiet=True)
    if pw is None or not analyzed:
        return None
    prev_good = None
    for a in analyzed:
        bad_reasons = []
        if a['enters_polygon_stop']:
            bad_reasons.append('path enters PolygonStop region '
                                f'(min_clearance={a["min_clearance_m"]}m '
                                f'< {POLY_STOP_R}m)')
        if a['plan_enters_sw_column']:
            bad_reasons.append('path enters the SW-side approach column '
                                '(same test C2-NAV.13 used on the GT track)')
        if bad_reasons:
            return dict(found=True, snapshot=a, reasons=bad_reasons,
                        preceding_good=prev_good)
        prev_good = a
    return dict(found=False, snapshot=None, reasons=[],
                preceding_good=analyzed[-1] if analyzed else None)


def report_first_bad(tag=TAG):
    hdr(f'{tag}: FIRST_BAD_PLAN')
    fb = first_bad_plan(tag)
    if fb is None:
        print('  no plan-window capture to analyze')
        return fb
    if not fb['found']:
        print('  NO plan snapshot ever entered PolygonStop or the SW-side '
              'column -- the global plan itself never threads the bad '
              'region in this run.')
        return fb
    a = fb['snapshot']
    print(f'  t_offset_from_t0 = {a["ts_offset_from_t0_s"]} s '
          f'(ts_sim={a["ts_sim_s"]})')
    print(f'  reasons: {"; ".join(fb["reasons"])}')
    print(f'  min_clearance_m = {a["min_clearance_m"]} to '
          f'{a["min_clearance_obstacle"]} at {a["min_clearance_at"]}')
    print(f'  closest_to_sw_corner_m = {a["closest_to_sw_corner_m"]}')
    print(f'  first_pose = {a["first_pose"]}  last_pose = {a["last_pose"]}')
    if fb['preceding_good'] is not None:
        g = fb['preceding_good']
        print(f'  immediately preceding GOOD plan: t_offset='
              f'{g["ts_offset_from_t0_s"]}s, min_clearance={g["min_clearance_m"]}m, '
              f'last_pose={g["last_pose"]}')
    else:
        print('  this is the FIRST captured plan snapshot -- no preceding '
              'good one exists in this run')
    return fb


# ---------------------------------------------------------------------
# 5. CASE A vs CASE B classification (brief section 9).
# ---------------------------------------------------------------------

def classify(tag=TAG):
    hdr(f'{tag}: GLOBAL PLANNER vs DWB classification')
    pw, analyzed = snapshots(tag, quiet=True)
    if pw is None:
        print('  no plan-window capture to analyze')
        return None
    div = divergence_timing([tag]).get(tag, {})
    sw_commit_t = div.get('sw_commit_t_s')

    plan_bad_ticks = [a['ts_offset_from_t0_s'] for a in analyzed
                      if a['plan_enters_sw_column'] or a['enters_polygon_stop']]
    plan_first_bad_t = plan_bad_ticks[0] if plan_bad_ticks else None

    print(f'  robot GT track SW-column commit time : '
          f'{sw_commit_t if sw_commit_t is not None else "NEVER"} s')
    print(f'  global /plan first enters SW column/STOP: '
          f'{plan_first_bad_t if plan_first_bad_t is not None else "NEVER"} s')

    if plan_first_bad_t is None and sw_commit_t is None:
        verdict = ('NEITHER the global plan NOR the robot GT track ever '
                    'entered the SW-side column/PolygonStop region in this '
                    'run -- no SW-corner failure to classify (this run may '
                    'have failed a different way, or succeeded).')
    elif plan_first_bad_t is None and sw_commit_t is not None:
        verdict = ('CASE B -- GLOBAL PLAN GOOD, LOCAL CONTROLLER BAD. No '
                    'captured /plan snapshot ever threads the SW column or '
                    'PolygonStop region, but the robot GT track committed '
                    f'to it at t={sw_commit_t}s. DWB is not following the '
                    'plan it was given.')
    elif plan_first_bad_t is not None and sw_commit_t is None:
        verdict = ('CASE A (partial) -- the GLOBAL PLAN itself enters the '
                    'SW column/PolygonStop region '
                    f'(first at t={plan_first_bad_t}s), but the robot GT '
                    'track never committed there in this run (leg may have '
                    'ended/timed out/frozen elsewhere before the robot '
                    'physically reached that part of the plan).')
    elif plan_first_bad_t <= sw_commit_t:
        verdict = ('CASE A -- GLOBAL PLAN BAD, and it precedes the robot. '
                    f'The /plan itself first enters the SW column/'
                    f'PolygonStop region at t={plan_first_bad_t}s, '
                    f'{sw_commit_t - plan_first_bad_t:.2f}s BEFORE the '
                    f'robot GT track physically commits there '
                    f'(t={sw_commit_t}s). The global planner is choosing '
                    'the dangerous route; DWB is executing it.')
    else:
        verdict = ('CASE B -- the robot GT track commits to the SW column '
                    f'(t={sw_commit_t}s) BEFORE any captured /plan snapshot '
                    f'does (t={plan_first_bad_t}s), '
                    f'{plan_first_bad_t - sw_commit_t:.2f}s later. DWB '
                    'diverges from a plan that was, at the time, still '
                    'reading safe in this test.')
    print()
    print(f'  VERDICT: {verdict}')
    return dict(sw_commit_t_s=sw_commit_t, plan_first_bad_t_s=plan_first_bad_t,
                verdict=verdict)


# ---------------------------------------------------------------------
# 6. DUMP -- the committed derived record.
# ---------------------------------------------------------------------

def dump(out_path):
    pw, analyzed = snapshots(TAG, quiet=True)
    record = dict(tag=TAG, leg=LEG, self_test_pass=self_test())
    if pw is not None:
        record['plan_window_meta'] = {k: pw[k] for k in
                                      ('t0_sim_s', 't1_sim_s',
                                       'through_poses_world',
                                       'final_goal_world', 'n_snapshots')}
        record['snapshots'] = analyzed
    record['first_bad_plan'] = first_bad_plan(TAG)
    record['classification'] = classify(TAG)
    record['merged_timeline'] = merged_timeline(TAG)
    bj = bench_json(TAG)
    if bj is not None:
        record['bench_legs'] = bj.get('legs')
    with open(out_path, 'w') as f:
        json.dump(record, f, indent=1, default=str)
    print(f'\nwrote {out_path}')
    return record


def visualize(out_path=None):
    """One deterministic PNG (brief section 19): the C2-NAV.9 clearance
    field, box_obstacle_1's two corners, both via-poses, the final goal,
    the robot's actual GT track, and every captured /plan snapshot
    colour-coded by leg time -- so the reader can see, in one image,
    whether the global plan or the robot's own track is what threads
    close to a corner, and which one."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    from matplotlib.cm import ScalarMappable
    from matplotlib.colors import Normalize

    out_path = out_path or os.path.join(
        REPO_ROOT, 'docs', 'images', 'c2nav15_planwindow.png')
    pw, analyzed = snapshots(TAG, quiet=True)
    if pw is None:
        print('no plan-window capture to visualize')
        return
    rows = load_trace(TAG, LEG) or []

    xs, ys, clr = build_clearance_grid()
    fig, ax = plt.subplots(figsize=(9, 10), dpi=150)
    ax.contourf(xs, ys, clr, levels=np.linspace(0, 0.6, 41),
                cmap=plt.get_cmap('RdYlGn'), vmin=0, vmax=0.6, extend='max')
    ax.contour(xs, ys, clr, levels=[POLY_STOP_R], colors='blue',
               linewidths=1.6, linestyles='dashed')

    for b in BOXES:
        x0, x1, y0, y1 = rect(b)
        if x1 < GRID_X[0] or x0 > GRID_X[1] or y1 < GRID_Y[0] or y0 > GRID_Y[1]:
            continue
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor='dimgray',
                                edgecolor='black', zorder=5))
    for (name, pt) in (('SW', SW_CORNER), ('NW', NW_CORNER)):
        ax.plot(*pt, marker='*', color='yellow', markersize=14,
                markeredgecolor='black', zorder=7)
        ax.annotate(f'{name} corner', pt, textcoords='offset points',
                    xytext=(8, 6), fontsize=8, fontweight='bold', zorder=7)

    for (name, pt, color) in (('HEADING_POSE', HEADING_POSE, 'purple'),
                              ('WAYPOINT', WAYPOINT, 'darkorange')):
        ax.plot(*pt, marker='D', color=color, markersize=9, zorder=8,
                markeredgecolor='black')
        ax.annotate(name, pt, textcoords='offset points', xytext=(8, -10),
                    fontsize=8, color=color, fontweight='bold', zorder=8)
    ax.add_patch(Circle(GOAL_SHIFTED, POLY_STOP_R, fill=False,
                        edgecolor='blue', linewidth=1.5, linestyle='dashed',
                        zorder=6))
    ax.plot(*GOAL_SHIFTED, marker='X', color='blue', markersize=11, zorder=9,
            markeredgecolor='white')
    ax.annotate('enclosure_entry goal', GOAL_SHIFTED,
                textcoords='offset points', xytext=(8, 8), fontsize=8,
                color='blue', fontweight='bold', zorder=9)

    if rows:
        px = [r['x'] for r in rows]
        py = [r['y'] for r in rows]
        ax.plot(px, py, color='black', linewidth=2.0, alpha=0.85, zorder=10,
                label='robot GT track')

    tmax = max(a['ts_offset_from_t0_s'] for a in analyzed) or 1.0
    norm = Normalize(vmin=0, vmax=tmax)
    cmap2 = plt.get_cmap('cool')
    fb = first_bad_plan(TAG)
    fb_t = fb['snapshot']['ts_offset_from_t0_s'] if fb and fb['found'] else None
    for snap, a in zip(pw['snapshots'], analyzed):
        pts = snap['poses_world']
        if not pts:
            continue
        xs_p = [p[0] for p in pts]
        ys_p = [p[1] for p in pts]
        is_bad = fb_t is not None and a['ts_offset_from_t0_s'] == fb_t
        ax.plot(xs_p, ys_p, color=('red' if is_bad else
                                    cmap2(norm(a['ts_offset_from_t0_s']))),
                linewidth=2.2 if is_bad else 0.8,
                alpha=1.0 if is_bad else 0.55, zorder=(11 if is_bad else 9))

    sm = ScalarMappable(norm=norm, cmap=cmap2)
    cbar = fig.colorbar(sm, ax=ax, shrink=0.6, pad=0.02)
    cbar.set_label('/plan snapshot: t offset from leg start (s)  '
                   '(red = FIRST_BAD_PLAN)')
    ax.set_xlim(*GRID_X)
    ax.set_ylim(*GRID_Y)
    ax.set_aspect('equal')
    ax.set_xlabel('world x (m)')
    ax.set_ylabel('world y (m)')
    ax.legend(loc='lower left', fontsize=8, framealpha=0.9)
    ax.set_title(f'C2-NAV.15: {TAG} -- every captured /plan snapshot '
                 f'vs. the executed GT track\n(black contour under the '
                 f'field omitted; blue dashed = PolygonStop 0.25 m)',
                 fontsize=10)
    fig.savefig(out_path, bbox_inches='tight')
    print(f'wrote {out_path}')


def all_(argv):
    self_test()
    snapshots(TAG)
    merged_timeline(TAG)
    report_first_bad(TAG)
    classify(TAG)
    return 0


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if cmd == 'selftest':
        ok = self_test()
        return 0 if ok else 1
    if cmd == 'snapshots':
        snapshots(TAG)
        return 0
    if cmd == 'timeline':
        merged_timeline(TAG)
        return 0
    if cmd == 'firstbad':
        report_first_bad(TAG)
        return 0
    if cmd == 'classify':
        classify(TAG)
        return 0
    if cmd == 'dump':
        out = sys.argv[2] if len(sys.argv) > 2 else \
            os.path.join(HERE, 'c2nav15_bench.json')
        dump(out)
        return 0
    if cmd == 'viz':
        visualize()
        return 0
    if cmd == 'all':
        return all_(sys.argv[2:])
    print(f'unknown command: {cmd}')
    return 2


if __name__ == '__main__':
    sys.exit(main())
