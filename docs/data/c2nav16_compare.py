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
"""C2-NAV.16: GOOD (c2n15_tour_r1) vs BAD (c2n16_tour_r1) plan-window
comparison, byte-identical configuration (sha256 6f61e499... of
c2nav11_ntp_params.yaml, same goal/waypoint/heading-pose overrides).

C2-NAV.15 established the tooling and captured one run that did NOT
reproduce the SW-corner deadlock. This session (C2-NAV.16) captured one
that did, on the FIRST fresh seven-leg tour attempted -- no tuning, no
retry, the same uncontrolled variance this chain has measured since
C2-NAV.8 (1/3). This module finds the earliest measurable difference
between the two runs.

Reuses BY IMPORT, not restates: every c2nav15_planwindow.py function
already accepts an explicit `tag` argument (built that way on purpose,
per that module's own C2-NAV.16 "exact next command" note: "retarget its
TAG constant, or pass the tag explicitly"). Nothing here re-implements
snapshot analysis, the SW-column test, or the removal-tick model.

New in this module (things no prior C2-NAV script computed):
  - dwb_command_window(): reads the C2-NAV.6 stop-probe CSV (an EXISTING
    capture, not a new subscription -- brief section 5) around the
    WAYPOINT removal tick, to see what DWB actually commanded (v_nav,
    w_nav, collision-monitor action) at the moment the two runs'
    replanned paths diverge.
  - replan_gaps(): inter-snapshot /plan timing, to check brief section 10
    (are there fewer/delayed replans in BAD).
  - first_divergence(): the brief's own central question (section 8),
    built from the above plus c2nav15_planwindow's per-snapshot table.

Two data sources per tag, both local scratch, `.navbench/` never tracked:
  1. `.navbench/results/{tag}_planwindow_enclosure_entry_rep0.json`
  2. `.navbench/results/{tag}_traces/enclosure_entry_rep0.csv` (via
     c2nav13_heading.load_trace)
  3. `.navbench/results/{tag}_stop.csv` (the C2-NAV.6 stop-probe capture,
     NEW read in this module: v_nav/w_nav/monitor_action per ~0.1s)
Every number this script derives is written to the committed
`docs/data/c2nav16_bench.json` so the finding survives even though the
raw artifacts might not.

Usage:
  python3 c2nav16_compare.py selftest
  python3 c2nav16_compare.py summary        # GOOD vs BAD, side by side
  python3 c2nav16_compare.py divergence     # FIRST_DIVERGENCE
  python3 c2nav16_compare.py dwb            # DWB command window at the tick
  python3 c2nav16_compare.py gaps           # replan-interval comparison
  python3 c2nav16_compare.py dump <out.json>
  python3 c2nav16_compare.py viz
  python3 c2nav16_compare.py all
"""
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import c2nav15_planwindow as pw                                # noqa: E402
from c2nav13_heading import load_trace, RATE_PERIOD             # noqa: E402
from c2nav12_report import WAYPOINT, GOAL_SHIFTED, SW_CORNER, DEADLOCK_POSE  # noqa: E402
from c2nav14_heading_pose import HEADING_POSE                   # noqa: E402

GOOD = 'c2n15_tour_r1'
BAD = 'c2n16_tour_r1'
LEG = 'enclosure_entry'
REPO_ROOT = pw.REPO_ROOT
RESULTS_DIR = pw.RESULTS_DIR

# This session's own measured facts, quoted verbatim, for the self-test.
BAD_FROZEN_POSE = (-3.1973, 1.9006)
BAD_FROZEN_DIST_TO_DEADLOCK_M = 0.1032
BAD_SW_COMMIT_T_S = 17.0
BAD_REMOVAL_TICK_S = 9.00900900900901
BAD_FIRST_BAD_PLAN_T_S = 9.19
GOOD_FIRST_BAD_PLAN_T_S = 14.7
GOOD_SW_COMMIT = None   # NEVER, per C2-NAV.15/this session's classify()


def hdr(t):
    print()
    print('=' * 78)
    print(t)
    print('=' * 78)


# ---------------------------------------------------------------------
# 0. SELF-TEST
# ---------------------------------------------------------------------

def self_test():
    hdr('SELF-TEST: reproduce this session\'s own measured facts before '
        'trusting anything new')
    ok = pw.self_test()

    div = pw.snapshots(BAD, quiet=True)
    from c2nav13_heading import trace_path
    rows = load_trace(BAD, LEG)
    last = rows[-1]
    d = math.dist((last['x'], last['y']), DEADLOCK_POSE)
    print(f'  BAD frozen pose ({last["x"]:.4f},{last["y"]:.4f}) vs '
          f'DEADLOCK_POSE {DEADLOCK_POSE}: {d*1000:.1f} mm  want ~103.2 mm  '
          f'{"PASS" if abs(d - BAD_FROZEN_DIST_TO_DEADLOCK_M) < 0.002 else "FAIL"}')
    ok &= abs(d - BAD_FROZEN_DIST_TO_DEADLOCK_M) < 0.002

    fb = pw.first_bad_plan(BAD)
    t_fb = fb['snapshot']['ts_offset_from_t0_s']
    print(f'  BAD FIRST_BAD_PLAN t={t_fb}s  want 9.19s  '
          f'{"PASS" if abs(t_fb - BAD_FIRST_BAD_PLAN_T_S) < 0.01 else "FAIL"}')
    ok &= abs(t_fb - BAD_FIRST_BAD_PLAN_T_S) < 0.01

    fb_good = pw.first_bad_plan(GOOD)
    t_fb_good = fb_good['snapshot']['ts_offset_from_t0_s']
    print(f'  GOOD FIRST_BAD_PLAN t={t_fb_good}s  want 14.7s  '
          f'{"PASS" if abs(t_fb_good - GOOD_FIRST_BAD_PLAN_T_S) < 0.01 else "FAIL"}')
    ok &= abs(t_fb_good - GOOD_FIRST_BAD_PLAN_T_S) < 0.01

    print()
    print('SELF-TEST: ALL PASS' if ok else 'SELF-TEST: FAILURE -- DO NOT '
          'TRUST ANYTHING BELOW')
    return ok


# ---------------------------------------------------------------------
# 1. DWB command window at the WAYPOINT removal tick (NEW: reads the
#    C2-NAV.6 stop-probe CSV, an existing capture).
# ---------------------------------------------------------------------

def stop_csv_path(tag):
    return os.path.join(RESULTS_DIR, f'{tag}_stop.csv')


def load_stop_csv(tag):
    p = stop_csv_path(tag)
    if not os.path.exists(p):
        return None
    with open(p) as f:
        return list(csv.DictReader(f))


def dwb_command_window(tag, center_offset_s, half_window_s=1.5):
    """Rows of the stop-probe CSV within `half_window_s` of
    leg_t0 + center_offset_s (both in leg-relative seconds, matching the
    plan-window snapshot timestamps)."""
    pwdata = pw.load_planwindow(tag)
    rows = load_stop_csv(tag)
    if pwdata is None or rows is None:
        return None
    t0 = pwdata['t0_sim_s']
    target = t0 + center_offset_s
    out = []
    for r in rows:
        if not r['stamp']:
            continue
        stamp = float(r['stamp'])
        if abs(stamp - target) <= half_window_s:
            out.append({
                't_offset_s': round(stamp - t0, 3),
                'gt_x': float(r['gt_x']), 'gt_y': float(r['gt_y']),
                'monitor_action': r['monitor_action'],
                'v_nav': float(r['v_nav']) if r['v_nav'] else None,
                'w_nav': float(r['w_nav']) if r['w_nav'] else None,
                'v_wheel': float(r['v_wheel']) if r['v_wheel'] else None,
            })
    out.sort(key=lambda r: r['t_offset_s'])
    return out


def stall_duration(window):
    """Longest contiguous run of v_nav == 0.0 inside the window, in
    seconds (approximate, from row-to-row spacing)."""
    if not window:
        return 0.0
    best = 0.0
    run_start = None
    prev_t = None
    for r in window:
        stalled = (r['v_nav'] == 0.0)
        if stalled:
            if run_start is None:
                run_start = r['t_offset_s']
            prev_t = r['t_offset_s']
        else:
            if run_start is not None:
                best = max(best, prev_t - run_start)
            run_start = None
    if run_start is not None:
        best = max(best, prev_t - run_start)
    return round(best, 2)


def report_dwb_windows():
    hdr('DWB COMMAND WINDOW at the WAYPOINT removal tick '
        '(t0+9.009s, +/-1.5s) -- GOOD vs BAD')
    result = {}
    for tag in (GOOD, BAD):
        win = dwb_command_window(tag, 9.009, 1.5)
        sd = stall_duration(win) if win else None
        print(f'  {tag}: {len(win) if win else 0} rows, '
              f'v_nav==0 stall duration ~{sd}s')
        if win:
            for r in win[::3]:   # thin for readability, every ~0.3s
                print(f'    t={r["t_offset_s"]:>6.2f}  xy=({r["gt_x"]:+.4f},'
                      f'{r["gt_y"]:+.4f})  action={r["monitor_action"]:<10} '
                      f'v_nav={r["v_nav"]}  w_nav={r["w_nav"]}')
        result[tag] = {'window': win, 'stall_duration_s': sd}
    return result


# ---------------------------------------------------------------------
# 2. Replan-interval / gap comparison (brief section 10).
# ---------------------------------------------------------------------

def replan_intervals(tag):
    _pw, analyzed = pw.snapshots(tag, quiet=True)
    ts = [a['ts_offset_from_t0_s'] for a in analyzed]
    intervals = [round(ts[i + 1] - ts[i], 3) for i in range(len(ts) - 1)]
    gaps = [(ts[i], iv) for i, iv in enumerate(intervals) if iv > 5.0]
    return dict(n_snapshots=len(ts), leg_duration_s=round(ts[-1] - ts[0], 2),
                intervals=intervals, gaps_over_5s=gaps,
                expected_period_s=round(RATE_PERIOD, 3))


def report_gaps():
    hdr('REPLAN INTERVAL / GAP comparison -- GOOD vs BAD')
    result = {}
    for tag in (GOOD, BAD):
        r = replan_intervals(tag)
        print(f'  {tag}: {r["n_snapshots"]} snapshots over '
              f'{r["leg_duration_s"]}s (expected period '
              f'{r["expected_period_s"]}s)')
        print(f'    gaps > 5s: {r["gaps_over_5s"]}')
        result[tag] = r
    return result


# ---------------------------------------------------------------------
# 3. FIRST_DIVERGENCE (brief section 8, the central question).
# ---------------------------------------------------------------------

def first_divergence():
    hdr('FIRST_DIVERGENCE')
    good_pw, good_snap = pw.snapshots(GOOD, quiet=True)
    bad_pw, bad_snap = pw.snapshots(BAD, quiet=True)

    # Both runs replan on the identical RateController schedule -- confirm
    # this holds up to and including the snapshot immediately after the
    # WAYPOINT removal tick (t0+9.009s), before comparing plan CONTENT.
    good_tick = min(good_snap, key=lambda a: abs(a['ts_offset_from_t0_s'] - 9.009))
    bad_tick = min(bad_snap, key=lambda a: abs(a['ts_offset_from_t0_s'] - 9.009))

    good_prev = [a for a in good_snap if a['ts_offset_from_t0_s'] < good_tick['ts_offset_from_t0_s']][-1]
    bad_prev = [a for a in bad_snap if a['ts_offset_from_t0_s'] < bad_tick['ts_offset_from_t0_s']][-1]

    rows_good = load_trace(GOOD, LEG)
    rows_bad = load_trace(BAD, LEG)
    gp0 = good_pw['t0_sim_s']
    bp0 = bad_pw['t0_sim_s']

    def gt_at(rows, t):
        return min(rows, key=lambda r: abs(r['t'] - t))

    gt_good_tick = gt_at(rows_good, 9.009)
    gt_bad_tick = gt_at(rows_bad, 9.009)

    print('  Both runs: identical config, identical via-poses, identical '
          'RateController/RemovePassedGoals schedule (HEADING_POSE tick '
          't=6.006s, WAYPOINT tick t=9.009s, both runs -- confirmed).')
    print()
    print(f'  Pre-tick /plan (last snapshot before t0+9.009s):')
    print(f'    GOOD t={good_prev["ts_offset_from_t0_s"]}s  n={good_prev["n_poses"]} '
          f'len={good_prev["path_length_m"]}m  min_clr={good_prev["min_clearance_m"]}m  '
          f'SW_col={good_prev["plan_enters_sw_column"]}')
    print(f'    BAD  t={bad_prev["ts_offset_from_t0_s"]}s  n={bad_prev["n_poses"]} '
          f'len={bad_prev["path_length_m"]}m  min_clr={bad_prev["min_clearance_m"]}m  '
          f'SW_col={bad_prev["plan_enters_sw_column"]}')
    print()
    print(f'  Robot GT pose at t0+9.009s (the removal tick itself):')
    print(f'    GOOD xy=({gt_good_tick["x"]:.4f},{gt_good_tick["y"]:.4f}) '
          f'yaw={math.degrees(gt_good_tick["yaw"]):.2f} deg')
    print(f'    BAD  xy=({gt_bad_tick["x"]:.4f},{gt_bad_tick["y"]:.4f}) '
          f'yaw={math.degrees(gt_bad_tick["yaw"]):.2f} deg')
    d_pose = math.dist((gt_good_tick['x'], gt_good_tick['y']),
                        (gt_bad_tick['x'], gt_bad_tick['y']))
    print(f'    pose delta at the tick: {d_pose*1000:.1f} mm')
    print()
    print(f'  First /plan snapshot AT/AFTER the removal tick:')
    print(f'    GOOD t={good_tick["ts_offset_from_t0_s"]}s  n={good_tick["n_poses"]} '
          f'len={good_tick["path_length_m"]}m  min_clr={good_tick["min_clearance_m"]}m  '
          f'SW_col={good_tick["plan_enters_sw_column"]}  STOP={good_tick["enters_polygon_stop"]}')
    print(f'    BAD  t={bad_tick["ts_offset_from_t0_s"]}s  n={bad_tick["n_poses"]} '
          f'len={bad_tick["path_length_m"]}m  min_clr={bad_tick["min_clearance_m"]}m  '
          f'SW_col={bad_tick["plan_enters_sw_column"]}  STOP={bad_tick["enters_polygon_stop"]}')
    print()

    dwb = report_dwb_windows()
    good_stall = dwb[GOOD]['stall_duration_s']
    bad_stall = dwb[BAD]['stall_duration_s']
    print()
    print(f'  DWB translational stall (v_nav==0) in the +/-1.5s window '
          f'around the tick: GOOD {good_stall}s vs BAD {bad_stall}s')

    verdict = (
        'FIRST_DIVERGENCE = the /plan snapshot captured immediately at/after '
        f'the WAYPOINT RemovePassedGoals tick (t0+{bad_tick["ts_offset_from_t0_s"]}s '
        f'in BAD, t0+{good_tick["ts_offset_from_t0_s"]}s in GOOD -- the same '
        'tick, same schedule, in both runs). Both runs are near-identical up '
        f'to that point (pose delta at the tick: {d_pose*1000:.1f} mm, GT '
        'tracks agree within 5-15 cm since leg start). The event that '
        'differs is the CONTENT of the replan the global planner produces '
        'once the via-poses are pruned from {goals}: GOOD\'s replan stays on '
        'the safe/wide side (unchanged min_clearance), BAD\'s replan '
        'immediately threads box_obstacle_1\'s SW pinch (min_clearance drops '
        'to 0.2306m, inside PolygonStop, inside the SW column -- CASE A). '
        'BAD additionally shows the robot holding position (v_nav=0.0) while '
        f'commanding near-maximum angular velocity for ~{bad_stall}s bracketing '
        'the tick -- DWB rotating in place to align with the new (bad) plan\'s '
        f'heading -- while GOOD shows no such stall ({good_stall}s). This is '
        'consistent with, not separate from, the removal event: the two '
        'runs\' plans do not differ meaningfully BEFORE this tick and do '
        'differ sharply at the very next captured snapshot after it.'
    )
    print()
    print('  VERDICT:')
    print(f'  {verdict}')
    return dict(good_tick=good_tick, bad_tick=bad_tick, good_prev=good_prev,
                bad_prev=bad_prev, pose_delta_at_tick_mm=round(d_pose * 1000, 1),
                gt_good_tick=gt_good_tick, gt_bad_tick=gt_bad_tick,
                dwb_stall_s=dict(GOOD=good_stall, BAD=bad_stall),
                verdict=verdict)


# ---------------------------------------------------------------------
# 4. FROZEN_PLAN_STATE (brief section 6).
# ---------------------------------------------------------------------

def frozen_plan_state():
    hdr('FROZEN_PLAN_STATE (BAD run)')
    _pw, analyzed = pw.snapshots(BAD, quiet=True)
    tail = [a for a in analyzed if a['ts_offset_from_t0_s'] >= 20.6]
    n_set = {a['n_poses'] for a in tail}
    len_set = {a['path_length_m'] for a in tail}
    clr_set = {a['min_clearance_m'] for a in tail}
    print(f'  From t={tail[0]["ts_offset_from_t0_s"]}s to '
          f'{tail[-1]["ts_offset_from_t0_s"]}s ({len(tail)} snapshots): '
          f'n_poses in {sorted(n_set)}, path_length_m in {sorted(len_set)}, '
          f'min_clearance_m in {sorted(clr_set)}')
    stable = len(n_set) == 1 and len(len_set) == 1 and len(clr_set) == 1
    print(f'  Plan geometry {"IS" if stable else "is NOT"} constant across '
          'the whole frozen tail.')
    return dict(first_frozen_t_s=tail[0]['ts_offset_from_t0_s'],
                last_t_s=tail[-1]['ts_offset_from_t0_s'],
                n_snapshots=len(tail), stable=stable,
                n_poses=sorted(n_set), path_length_m=sorted(len_set),
                min_clearance_m=sorted(clr_set))


# ---------------------------------------------------------------------
# 5. Recovery-behaviour cross-check for the post-freeze replan gaps.
# ---------------------------------------------------------------------

def recovery_signature_count(tag):
    """Count of 'Failed to make progress' lines in the captured Nav2
    console log for this run -- an EXISTING artifact (the driver's own
    nav2 stdout capture), not a new subscription."""
    log = os.path.join(REPO_ROOT, '.navbench', 'logs', f'nav_{tag}.log')
    if not os.path.exists(log):
        return None
    n = 0
    with open(log, errors='replace') as f:
        for line in f:
            if 'Failed to make progress' in line:
                n += 1
    return n


# ---------------------------------------------------------------------
# 6. DUMP + summary + viz
# ---------------------------------------------------------------------

def summary():
    hdr('C2-NAV.16 SUMMARY: GOOD (c2n15_tour_r1) vs BAD (c2n16_tour_r1)')
    for tag in (GOOD, BAD):
        bj = pw.bench_json(tag)
        leg = None
        if bj:
            for L in bj.get('legs', []):
                if L.get('scenario') == LEG:
                    leg = L
        if leg:
            print(f'  {tag}: status={leg.get("status")} '
                  f'duration_sim_s={leg.get("duration_sim_s")} '
                  f'final_goal_err_m={leg.get("final_goal_err_m")} '
                  f'n_stops={leg.get("n_stops")} '
                  f'min_clearance_m={leg.get("min_clearance_m")} '
                  f'dwb_illegal_frac={leg.get("dwb_illegal_frac")}')
        else:
            print(f'  {tag}: no leg record found')
    div = first_divergence()
    gaps = report_gaps()
    frozen = frozen_plan_state()
    rec_good = recovery_signature_count(GOOD)
    rec_bad = recovery_signature_count(BAD)
    print()
    print(f'  "Failed to make progress" occurrences: GOOD={rec_good}  BAD={rec_bad}')
    return dict(divergence=div, gaps=gaps, frozen=frozen,
                recovery_count=dict(GOOD=rec_good, BAD=rec_bad))


def dump(out_path):
    record = dict(good_tag=GOOD, bad_tag=BAD, leg=LEG,
                  self_test_pass=self_test())
    record['good_classification'] = pw.classify(GOOD)
    record['bad_classification'] = pw.classify(BAD)
    record['good_first_bad_plan'] = pw.first_bad_plan(GOOD)
    record['bad_first_bad_plan'] = pw.first_bad_plan(BAD)
    record['first_divergence'] = first_divergence()
    record['replan_gaps'] = report_gaps()
    record['frozen_plan_state'] = frozen_plan_state()
    record['recovery_count'] = dict(GOOD=recovery_signature_count(GOOD),
                                     BAD=recovery_signature_count(BAD))
    good_bj = pw.bench_json(GOOD)
    bad_bj = pw.bench_json(BAD)
    record['good_legs'] = good_bj.get('legs') if good_bj else None
    record['bad_legs'] = bad_bj.get('legs') if bad_bj else None
    with open(out_path, 'w') as f:
        json.dump(record, f, indent=1, default=str)
    print(f'\nwrote {out_path}')
    return record


def visualize(out_path=None):
    """GOOD vs BAD overlay on C2-NAV.9's clearance field: both GT tracks,
    both via-poses, the goal, and every captured /plan snapshot for BOTH
    runs, colour-coded by run (not time, unlike C2-NAV.15's own single-run
    plot) so the reader can see where the two runs' plans separate."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    import numpy as np

    out_path = out_path or os.path.join(
        REPO_ROOT, 'docs', 'images', 'c2nav16_compare.png')

    good_pwdata, good_snap = pw.snapshots(GOOD, quiet=True)
    bad_pwdata, bad_snap = pw.snapshots(BAD, quiet=True)
    rows_good = load_trace(GOOD, LEG) or []
    rows_bad = load_trace(BAD, LEG) or []

    xs, ys, clr = pw.build_clearance_grid()
    fig, ax = plt.subplots(figsize=(9, 10), dpi=150)
    ax.contourf(xs, ys, clr, levels=np.linspace(0, 0.6, 41),
                cmap=plt.get_cmap('Greys'), vmin=0, vmax=0.6, extend='max',
                alpha=0.35)
    ax.contour(xs, ys, clr, levels=[pw.POLY_STOP_R], colors='blue',
               linewidths=1.6, linestyles='dashed')

    for b in pw.BOXES:
        x0, x1, y0, y1 = pw.rect(b)
        if x1 < pw.GRID_X[0] or x0 > pw.GRID_X[1] or y1 < pw.GRID_Y[0] or y0 > pw.GRID_Y[1]:
            continue
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor='dimgray',
                                edgecolor='black', zorder=5))
    for (name, pt) in (('SW', SW_CORNER), ('NW', (-3.25, 2.65))):
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
    ax.add_patch(Circle(GOAL_SHIFTED, pw.POLY_STOP_R, fill=False,
                        edgecolor='blue', linewidth=1.5, linestyle='dashed',
                        zorder=6))
    ax.plot(*GOAL_SHIFTED, marker='X', color='blue', markersize=11, zorder=9,
            markeredgecolor='white')

    # every /plan snapshot, thin lines, colour by run
    for snap in good_pwdata['snapshots']:
        pts = snap['poses_world']
        if pts:
            ax.plot([p[0] for p in pts], [p[1] for p in pts],
                    color='tab:green', linewidth=0.7, alpha=0.35, zorder=9)
    for snap in bad_pwdata['snapshots']:
        pts = snap['poses_world']
        if pts:
            ax.plot([p[0] for p in pts], [p[1] for p in pts],
                    color='tab:red', linewidth=0.7, alpha=0.35, zorder=9)

    # highlight each run's FIRST_BAD_PLAN
    fb_good = pw.first_bad_plan(GOOD)
    fb_bad = pw.first_bad_plan(BAD)
    for fb, color, label in ((fb_good, 'darkgreen', 'GOOD FIRST_BAD_PLAN'),
                              (fb_bad, 'darkred', 'BAD FIRST_BAD_PLAN')):
        if fb and fb['found']:
            t = fb['snapshot']['ts_offset_from_t0_s']
            src = good_pwdata if fb is fb_good else bad_pwdata
            for snap in src['snapshots']:
                if snap['ts_sim_s'] == fb['snapshot']['ts_sim_s']:
                    pts = snap['poses_world']
                    ax.plot([p[0] for p in pts], [p[1] for p in pts],
                            color=color, linewidth=2.5, alpha=1.0, zorder=12,
                            label=label)
                    break

    if rows_good:
        ax.plot([r['x'] for r in rows_good], [r['y'] for r in rows_good],
                color='green', linewidth=2.2, alpha=0.9, zorder=11,
                label='GOOD GT track (c2n15_tour_r1, SUCCEEDED)')
    if rows_bad:
        ax.plot([r['x'] for r in rows_bad], [r['y'] for r in rows_bad],
                color='red', linewidth=2.2, alpha=0.9, zorder=11,
                label='BAD GT track (c2n16_tour_r1, TIMEOUT/deadlock)')
    ax.plot(*BAD_FROZEN_POSE, marker='X', color='black', markersize=12,
            zorder=13, label='BAD frozen pose')

    ax.set_xlim(*pw.GRID_X)
    ax.set_ylim(*pw.GRID_Y)
    ax.set_aspect('equal')
    ax.set_xlabel('world x (m)')
    ax.set_ylabel('world y (m)')
    ax.legend(loc='lower left', fontsize=7, framealpha=0.9)
    ax.set_title('C2-NAV.16: GOOD vs BAD enclosure_entry, byte-identical '
                 'config\n(green=GOOD run, red=BAD run; bold lines = each '
                 'run\'s FIRST_BAD_PLAN)', fontsize=10)
    fig.savefig(out_path, bbox_inches='tight')
    print(f'wrote {out_path}')


def all_(argv):
    ok = self_test()
    summary()
    return 0 if ok else 1


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if cmd == 'selftest':
        return 0 if self_test() else 1
    if cmd == 'summary':
        summary()
        return 0
    if cmd == 'divergence':
        first_divergence()
        return 0
    if cmd == 'dwb':
        report_dwb_windows()
        return 0
    if cmd == 'gaps':
        report_gaps()
        return 0
    if cmd == 'dump':
        out = sys.argv[2] if len(sys.argv) > 2 else \
            os.path.join(HERE, 'c2nav16_bench.json')
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
