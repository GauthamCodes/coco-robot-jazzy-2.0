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
"""C2-NAV.12 report: does the C2-NAV.11 continuous multi-pose enclosure
approach survive the COMPLETE seven-leg tour, with accumulated
heading/state from the six legs before it, rather than a fresh
corridor_gate-only start?

  c2nav12_report.py collect <results_dir> [out.json]
  c2nav12_report.py legs    [results_dir | collected.json]
  c2nav12_report.py entry   [results_dir | collected.json]
  c2nav12_report.py exit    [results_dir | collected.json]
  c2nav12_report.py stop    [results_dir | collected.json]
  c2nav12_report.py clear   [results_dir | collected.json]
  c2nav12_report.py compare [results_dir | collected.json]
  c2nav12_report.py all     [results_dir | collected.json]

This is C2-NAV.8's report.py (the seven-leg-tour instrument, all its
geometry and segmentation reused BY IMPORT, not restated) with C2-NAV.11's
continuity fields (`early_plan_*`) added to the `entry` table, because
C2-NAV.12 is the experiment that asks whether those two prior results
compose: C2-NAV.8 measured the seven-leg tour at this goal WITHOUT the
through-poses fix (1/3 entry, 269.5 s SW-corner deadlock in r1);
C2-NAV.11 measured the through-poses fix on a FRESH two-leg start
(corridor_gate, enclosure_entry only) that does not carry six legs of
accumulated heading and AMCL drift into the pinch (3/3, 0% PolygonStop).
Neither prior experiment ran the through-poses fix across all seven legs
with the enclosure approach beginning from wherever `corridor_gate`
actually leaves the robot after five preceding legs -- that is the one
new condition here, and everything else (params file, goal, waypoint,
PolygonStop, CSF, timeouts) is held byte-identical to both.
"""
import glob
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c2nav8_report import (                                  # noqa: E402
    BOXES, EXTRA_BOXES, CIRCLES, CIRCUMSCRIBED, STOP_RADIUS,
    LEGS, TOUR_WORLD, WORLD_TO_MAP, LEG_MATCH_M,
    nearest_full, traversed, _fmt)

DEFAULT_DIR = os.path.normpath(os.path.join(
    HERE, '..', '..', '.navbench', 'results'))
COLLECTED = os.path.join(HERE, 'c2nav12_bench.json')
C2NAV8 = os.path.join(HERE, 'c2nav8_bench.json')
C2NAV11 = os.path.join(HERE, 'c2nav11_bench.json')  # may not exist; optional

PARAMS_SHA = '6f61e49912765708e70470df967b23834338723176bcf7ae113f8b8c1e6bb950'
GOAL_SHIFTED = (-3.575, 2.95)
WAYPOINT = (-3.40, 1.35)
SW_CORNER = (-3.25, 2.15)            # box_obstacle_1 SW corner, c2nav9_corridor.py
DEADLOCK_POSE = (-3.3001, 1.9095)    # C2-NAV.8 r1's frozen GT pose, same source

# C2-NAV.12's own TOUR_WORLD differs from c2nav8_report's only in that
# enclosure_entry is driven via a through-pose, not a plain goal -- the
# map-frame goal position used for probe re-segmentation is unaffected,
# so TOUR_WORLD is reused unchanged.


def _closest(rows, target):
    best = None
    for r in rows:
        if r.get('x') in (None, ''):
            continue
        d = math.dist((float(r['x']), float(r['y'])), target)
        if best is None or d < best:
            best = d
    return best


def _runs_from_dir(d):
    """[{tag, legs, stop, stop_csv}] from <dir>/c2n12_tour_r*.json."""
    runs = []
    for p in sorted(glob.glob(os.path.join(d, 'c2n12_tour_r*.json')),
                    key=lambda q: (len(q), q)):
        if p.endswith(('_stop.json', '_geom.json')):
            continue
        tag = os.path.basename(p)[:-5]
        try:
            doc = json.load(open(p))
        except (OSError, ValueError) as e:
            print(f'  !! unreadable {p}: {e}')
            continue
        run = {'tag': tag, 'legs': doc.get('legs', [])}
        sp = os.path.join(d, f'{tag}_stop.json')
        if os.path.exists(sp):
            try:
                run['stop'] = json.load(open(sp))
            except (OSError, ValueError) as e:
                print(f'  !! unreadable {sp}: {e}')
        cp = os.path.join(d, f'{tag}_stop.csv')
        if os.path.exists(cp):
            run['stop_csv'] = cp
        td = os.path.join(d, f'{tag}_traces')
        if os.path.isdir(td):
            run['tracedir'] = td
        runs.append(run)
    return runs


def load(src=None):
    src = src or (COLLECTED if os.path.exists(COLLECTED) else DEFAULT_DIR)
    if os.path.isdir(src):
        return _runs_from_dir(src), src
    doc = json.load(open(src))
    return doc['runs'], src


def collect(argv):
    d = argv[0] if argv else DEFAULT_DIR
    out = argv[1] if len(argv) > 1 else COLLECTED
    runs = _runs_from_dir(d)
    if not runs:
        print(f'no c2n12_tour_r*.json under {d}')
        return 1
    for r in runs:
        r.pop('stop_csv', None)
        r.pop('tracedir', None)
    doc = {
        'experiment': 'C2-NAV.12',
        'question': 'does the C2-NAV.11 continuous multi-pose enclosure '
                    'approach survive the complete seven-leg tour with '
                    'accumulated state from six preceding legs?',
        'params_file': 'docs/data/c2nav11_ntp_params.yaml',
        'params_sha256': PARAMS_SHA,
        'navigation_change_vs_c2nav11': 'none',
        'goal_shifted_world': list(GOAL_SHIFTED),
        'waypoint_world': list(WAYPOINT),
        'through_pose_scenario': 'enclosure_entry',
        'action_used_for_enclosure_entry': 'NavigateThroughPoses',
        'bt_navigator.default_nav_through_poses_bt_xml':
            'navigate_through_poses_w_replanning_and_recovery.xml',
        'PolygonStop': {'radius': STOP_RADIUS, 'min_points': 4},
        'goal_xy_tolerance': 0.25,
        'leg_timeout_s': {n: (200.0 if n == 'enclosure_entry' else 75.0)
                          for n in LEGS},
        'reference_c2nav8': 'docs/data/c2nav8_bench.json (same 7-leg tour, '
                            'same goal, separate legacy legs, no through-pose)',
        'reference_c2nav11': '.navbench/results/c2n11_appr_r{1,2,3}.json '
                             '(same through-pose mechanism, fresh 2-leg '
                             'start: corridor_gate, enclosure_entry only)',
        'n_runs': len(runs),
        'runs': runs,
    }
    with open(out, 'w') as f:
        json.dump(doc, f, indent=1)
    print(f'collected {len(runs)} runs -> {out}')
    for r in runs:
        print(f'  {r["tag"]}: {len(r["legs"])} legs'
              f'{"" if "stop" in r else "   (NO stop sidecar)"}')
    return 0


def _leg(run, name):
    for lg in run['legs']:
        if lg['scenario'] == name:
            return lg
    return None


def legs(argv):
    runs, src = load(argv[0] if argv else None)
    print(f'source: {src}')
    print()
    print('C2-NAV.12 -- seven-leg tour, CSF 65, corrected '
          'NavigateThroughPoses BT, enclosure_entry via waypoint '
          f'{WAYPOINT} to goal {GOAL_SHIFTED}')
    print('TRAV = came within the 0.25 m xy tolerance.  SUCC = the action '
          'status, which needs the goal yaw too.')
    print()
    for run in runs:
        cap = {lg['scenario']: lg.get('timeout_s') for lg in run['legs']}
        print(f'--- {run["tag"]} ---')
        print(f'  {"leg":<16} {"status":<10} {"TRAV":<5} {"cap":>5} '
              f'{"dur_s":>7} {"err_m":>7} {"clear_q":>8} {"path_m":>7} '
              f'{"v_med":>6} {"stops":>5} {"prog":>4}')
        n_succ = n_trav = 0
        for name in LEGS:
            lg = _leg(run, name)
            if lg is None:
                print(f'  {name:<16} {"(not run)":<10}')
                continue
            ok = lg['status'] == 'SUCCEEDED'
            tr = traversed(lg)
            n_succ += ok
            n_trav += tr
            print(f'  {name:<16} {lg["status"]:<10} {"yes" if tr else "NO":<5} '
                  f'{_fmt(cap.get(name), 0, 5)} '
                  f'{_fmt(lg.get("duration_sim_s"), 2, 7)} '
                  f'{_fmt(lg.get("final_goal_err_m"), 3, 7)} '
                  f'{_fmt(lg.get("min_clearance_m"), 3, 8)} '
                  f'{_fmt(lg.get("path_len_m"), 3, 7)} '
                  f'{_fmt(lg.get("v_cmd_med"), 3, 6)} '
                  f'{str(lg.get("n_stops", "-")):>5} '
                  f'{str(lg.get("n_progress_failures", "-")):>4}')
        print(f'  TOTAL  SUCCEEDED {n_succ}/{len(LEGS)}   '
              f'TRAVERSED {n_trav}/{len(LEGS)}')
        print()
    tot_s = sum(1 for r in runs for n in LEGS
                if (_leg(r, n) or {}).get('status') == 'SUCCEEDED')
    tot_t = sum(1 for r in runs for n in LEGS
                if traversed(_leg(r, n) or {}))
    n = len(runs) * len(LEGS)
    print(f'C2-NAV.12 TOTAL: SUCCEEDED {tot_s}/{n}   TRAVERSED {tot_t}/{n}   '
          f'({len(runs)} fresh simulators)')
    print()
    print(f'  {"leg":<16} {"SUCCEEDED":>10} {"TRAVERSED":>10}')
    for name in LEGS:
        s = sum(1 for r in runs
                if (_leg(r, name) or {}).get('status') == 'SUCCEEDED')
        t = sum(1 for r in runs if traversed(_leg(r, name) or {}))
        print(f'  {name:<16} {f"{s}/{len(runs)}":>10} '
              f'{f"{t}/{len(runs)}":>10}')
    return 0


def _entry_row(run):
    lg = _leg(run, 'enclosure_entry')
    if lg is None:
        return None
    tracedir = run.get('tracedir')
    d_sw = d_deadlock = None
    if tracedir:
        import csv
        p = os.path.join(tracedir, 'enclosure_entry_rep0.csv')
        if os.path.exists(p):
            rows = list(csv.DictReader(open(p)))
            d_sw = _closest(rows, SW_CORNER)
            d_deadlock = _closest(rows, DEADLOCK_POSE)
    return lg, d_sw, d_deadlock


def entry(argv):
    runs, src = load(argv[0] if argv else None)
    print(f'source: {src}')
    print()
    print(f'enclosure_entry via ONE NavigateThroughPoses request '
          f'(waypoint {WAYPOINT} -> goal {GOAL_SHIFTED}), cap 200 s, '
          f'AFTER five preceding tour legs (accumulated heading/AMCL '
          f'state, not a fresh spawn).')
    print('C2-NAV.8  (legacy separate legs, tour context): SUCCEEDED 1/3, '
          'TRAVERSED 2/3, 201.42 / 200.22 / 123.67 s, r1 a 269.5 s '
          'PolygonStop deadlock at the SW corner.')
    print('C2-NAV.11 (through-poses fix, FRESH 2-leg start, no tour '
          'context): SUCCEEDED 3/3, TRAVERSED 3/3, 61.64 / 112.38 / '
          '156.37 s, 0% PolygonStop.')
    print()
    print(f'  {"run":<18} {"status":<10} {"TRAV":<5} {"dur_s":>7} '
          f'{"err_m":>7} {"d_sw_m":>7} {"d_dlk_m":>8} {"STOP":>6} '
          f'{"true_clr_m":>10}')
    for run in runs:
        row = _entry_row(run)
        if row is None:
            continue
        lg, d_sw, d_dlk = row
        st = ((run.get('stop') or {}).get('legs') or {}).get(
            'enclosure_entry', {})
        print(f'  {run["tag"]:<18} {lg["status"]:<10} '
              f'{"yes" if traversed(lg) else "NO":<5} '
              f'{_fmt(lg.get("duration_sim_s"), 2, 7)} '
              f'{_fmt(lg.get("final_goal_err_m"), 3, 7)} '
              f'{_fmt(d_sw, 3, 7)} {_fmt(d_dlk, 3, 8)} '
              f'{str(st.get("n_stop_rows", "-")):>6} '
              f'{_fmt(st.get("d_min_base_m_min"), 4, 10)}')
    print()
    print('  CONTINUITY EVIDENCE (early /plan capture, C2-NAV.11 mechanism):')
    print(f'  {"run":<18} {"via":<18} {"early_ts_off_s":>15} '
          f'{"n_poses":>8} {"endpt->goal_m":>14}')
    for run in runs:
        lg = _leg(run, 'enclosure_entry')
        if lg is None:
            continue
        print(f'  {run["tag"]:<18} {str(lg.get("through_poses_world")):<18} '
              f'{_fmt(lg.get("early_plan_ts_offset_from_t0_s"), 3, 15)} '
              f'{str(lg.get("early_plan_n_poses", "-")):>8} '
              f'{_fmt(lg.get("early_plan_endpoint_to_final_goal_m"), 3, 14)}')
    print()
    print('  TERMINAL YAW (still not the subject of this experiment):')
    print(f'  {"run":<18} {"yaw_travel_rad":>15} {"frac_of_leg":>12} '
          f'{"t_terminal_s":>13}')
    for run in runs:
        lg = _leg(run, 'enclosure_entry')
        if lg is None:
            continue
        print(f'  {run["tag"]:<18} '
              f'{_fmt(lg.get("terminal_yaw_travel_rad"), 3, 15)} '
              f'{_fmt(lg.get("terminal_frac_of_leg"), 3, 12)} '
              f'{_fmt(lg.get("t_terminal_s"), 2, 13)}')
    print()
    print('  DWB (chosen-trajectory critic + illegal-fraction):')
    print(f'  {"run":<18} {"BaseObstacle":>13} {"illegal_frac":>13} '
          f'{"vx_zero_frac":>13}')
    for run in runs:
        lg = _leg(run, 'enclosure_entry')
        if lg is None:
            continue
        bo = (lg.get('dwb_best_critic_mean') or {}).get('BaseObstacle')
        print(f'  {run["tag"]:<18} {_fmt(bo, 3, 13)} '
              f'{_fmt(lg.get("dwb_illegal_frac_transit"), 3, 13)} '
              f'{_fmt(lg.get("dwb_best_vx_zero_frac"), 3, 13)}')
    return 0


def exit_(argv):
    runs, src = load(argv[0] if argv else None)
    print(f'source: {src}')
    print()
    print('enclosure_exit -- immediately follows enclosure_entry in the '
          'SAME tour, so a failed/deadlocked entry above costs this leg '
          'exactly as it did in C2-NAV.8.')
    print()
    print(f'  {"run":<18} {"status":<10} {"TRAV":<5} {"dur_s":>7} '
          f'{"err_m":>7} {"driven_m":>8} {"STOP":>6} {"true_clr_m":>10}')
    for run in runs:
        lg = _leg(run, 'enclosure_exit')
        if lg is None:
            print(f'  {run["tag"]:<18} (not run)')
            continue
        st = ((run.get('stop') or {}).get('legs') or {}).get(
            'enclosure_exit', {})
        print(f'  {run["tag"]:<18} {lg["status"]:<10} '
              f'{"yes" if traversed(lg) else "NO":<5} '
              f'{_fmt(lg.get("duration_sim_s"), 2, 7)} '
              f'{_fmt(lg.get("final_goal_err_m"), 3, 7)} '
              f'{_fmt(lg.get("path_len_m"), 3, 8)} '
              f'{str(st.get("n_stop_rows", "-")):>6} '
              f'{_fmt(st.get("d_min_base_m_min"), 4, 10)}')
    print()
    print('  THE COMMAND CHAIN, per leg, from the probe CSV '
          '(median over rows with a command):')
    print(f'  {"run":<18} {"n":>6} {"v_nav":>8} {"v_smoothed":>11} '
          f'{"v_out":>8} {"v_wheel":>8}')
    for run in runs:
        rows = [r for r in _csv_rows(run) if _row_leg(r) == 'enclosure_exit']
        if not rows:
            continue
        med = {}
        for k in ('v_nav', 'v_smoothed', 'v_out', 'v_wheel'):
            vals = [float(r[k]) for r in rows if r.get(k) not in (None, '')]
            med[k] = statistics.median(vals) if vals else None
        print(f'  {run["tag"]:<18} {len(rows):>6} '
              f'{_fmt(med["v_nav"], 4, 8)} {_fmt(med["v_smoothed"], 4, 11)} '
              f'{_fmt(med["v_out"], 4, 8)} {_fmt(med["v_wheel"], 4, 8)}')
    return 0


def _csv_rows(run):
    import csv
    names = [f'{run["tag"]}_stop.csv']
    if run['tag'].startswith('c2n12_'):
        names.append('c2nav12_' + run['tag'][len('c2n12_'):] + '_stop.csv')
    cands = [run.get('stop_csv')] if run.get('stop_csv') else []
    cands += [os.path.join(HERE, n) for n in names]
    cands += [os.path.join(DEFAULT_DIR, n) for n in names]
    for path in cands:
        if path and os.path.exists(path):
            return list(csv.DictReader(open(path)))
    return []


def _row_leg(r):
    if not r.get('goal_map_x'):
        return None
    gx, gy = float(r['goal_map_x']), float(r['goal_map_y'])
    best, bd = None, LEG_MATCH_M
    for name, (wx, wy) in TOUR_WORLD.items():
        mx, my = wx + WORLD_TO_MAP[0], wy + WORLD_TO_MAP[1]
        d = ((gx - mx) ** 2 + (gy - my) ** 2) ** 0.5
        if d < bd:
            best, bd = name, d
    return best


def stop(argv):
    runs, src = load(argv[0] if argv else None)
    print(f'source: {src}')
    print()
    print('PolygonStop, re-segmented over all SEVEN legs from the probe '
          'CSV. radius 0.25 m, min_points 4, applied by '
          'c2nav6_stopprobe.py unchanged.')
    print()
    grand = {'rows': 0, 'stop': 0}
    for run in runs:
        rows = _csv_rows(run)
        if not rows:
            print(f'--- {run["tag"]}: NO CSV FOUND ---')
            continue
        segs = {}
        for r in rows:
            segs.setdefault(_row_leg(r), []).append(r)
        print(f'--- {run["tag"]}  ({len(rows)} recorded frames) ---')
        print(f'  {"leg":<16} {"frames":>7} {"STOP":>6} {"stop_frac":>10} '
              f'{"max_in_circle":>14} {"d_min_probe_m":>14}')
        for name in LEGS + [None]:
            rs = segs.get(name)
            if not rs:
                continue
            st = [r for r in rs if r['monitor_action'] == 'STOP']
            ins = [int(r['n_in_stop']) for r in rs if r['n_in_stop'] != '']
            dm = [float(r['d_min_base_m']) for r in rs
                  if r['d_min_base_m'] != '']
            grand['rows'] += len(rs)
            grand['stop'] += len(st)
            print(f'  {(name or "(no plan)"):<16} {len(rs):>7} {len(st):>6} '
                  f'{len(st)/len(rs):>10.4f} {(max(ins) if ins else 0):>14} '
                  f'{_fmt(min(dm) if dm else None, 4, 14)}')
        print()
    print(f'C2-NAV.12 TOTAL: {grand["stop"]} STOP frames on {grand["rows"]} '
          f'recorded frames across {len(runs)} tours')
    return 0


def clear(argv):
    runs, src = load(argv[0] if argv else None)
    print(f'source: {src}')
    print()
    print('TRUE minimum clearance: distance from the recorded ground-truth '
          'track to the nearest collision face of the whole world '
          f'({len(BOXES)} + {len(EXTRA_BOXES)} boxes, {len(CIRCLES)} '
          'cylinder), reused by import from c2nav8_report.nearest_full.')
    print(f'PolygonStop radius {STOP_RADIUS} m; measured circumscribed '
          f'radius {CIRCUMSCRIBED} m.')
    print()
    for run in runs:
        rows = _csv_rows(run)
        if not rows:
            print(f'--- {run["tag"]}: NO CSV FOUND ---')
            continue
        segs = {}
        for r in rows:
            if r.get('gt_x'):
                segs.setdefault(_row_leg(r), []).append(r)
        print(f'--- {run["tag"]} ---')
        print(f'  {"leg":<16} {"n":>6} {"true_min":>9} {"nearest":<22} '
              f'{"laser":>8} {"quantised":>10}  note')
        for name in LEGS:
            rs = segs.get(name)
            if not rs:
                continue
            pts = [(float(r['gt_x']), float(r['gt_y'])) for r in rs]
            (d, who, _), _p = min(
                (nearest_full(x, y)[0], (x, y)) for x, y in pts)
            las = [float(r['d_min_base_m']) for r in rs
                   if r.get('d_min_base_m') not in (None, '')]
            lmin = min(las) if las else None
            lg = _leg(run, name)
            q = (lg or {}).get('min_clearance_m')
            if d < CIRCUMSCRIBED:
                note = '<<< BELOW THE CIRCUMSCRIBED RADIUS'
            elif d < STOP_RADIUS:
                note = f'inside PolygonStop by {(STOP_RADIUS-d)*1000:.1f} mm'
            else:
                note = f'clear of PolygonStop by {(d-STOP_RADIUS)*1000:.1f} mm'
            if lmin is not None and d - lmin > 0.02:
                note += f'   (!) geometry exceeds laser by ' \
                        f'{(d-lmin)*1000:.0f} mm'
            print(f'  {name:<16} {len(pts):>6} {d:>9.4f} {who:<22} '
                  f'{_fmt(lmin, 4, 8)} {_fmt(q, 3, 10)}  {note}')
        print()
    return 0


def compare(argv):
    """C2-NAV.12 against C2-NAV.8 (same goal, legacy legs) leg-by-leg."""
    runs, src = load(argv[0] if argv else None)
    print(f'source: {src}')
    if not os.path.exists(C2NAV8):
        print(f'  !! {C2NAV8} missing; cannot compare')
        return 1
    ref = json.load(open(C2NAV8))
    ref_runs = [r for r in ref['runs'] if r['tag'].startswith('c2n8_tour')]
    print(f'reference: {C2NAV8}  ({", ".join(r["tag"] for r in ref_runs)})')
    print()
    print('SAME params file family (c2nav11_ntp_params.yaml differs from '
          'C2-NAV.8\'s c2nav4_csf65_params.yaml by exactly the one BT-XML '
          'line), SAME goal, SAME tour. The only variable is whether '
          'enclosure_entry is driven as a legacy single NavigateToPose '
          '(C2-NAV.8) or one continuous NavigateThroughPoses through the '
          'waypoint (C2-NAV.12).')
    print()
    print(f'  {"leg":<16} {"C2-NAV.8 SUCC":>14} {"C2-NAV.12 SUCC":>15} '
          f'{"C2-NAV.8 TRAV":>14} {"C2-NAV.12 TRAV":>15}')
    t8 = t12 = 0
    for name in LEGS:
        s8 = sum(1 for r in ref_runs
                 if (_leg(r, name) or {}).get('status') == 'SUCCEEDED')
        s12 = sum(1 for r in runs
                  if (_leg(r, name) or {}).get('status') == 'SUCCEEDED')
        v8 = sum(1 for r in ref_runs if traversed(_leg(r, name) or {}))
        v12 = sum(1 for r in runs if traversed(_leg(r, name) or {}))
        t8 += s8
        t12 += s12
        print(f'  {name:<16} {f"{s8}/{len(ref_runs)}":>14} '
              f'{f"{s12}/{len(runs)}":>15} {f"{v8}/{len(ref_runs)}":>14} '
              f'{f"{v12}/{len(runs)}":>15}')
    print(f'  {"TOTAL":<16} {f"{t8}/{len(ref_runs)*len(LEGS)}":>14} '
          f'{f"{t12}/{len(runs)*len(LEGS)}":>15}')
    return 0


def all_(argv):
    for fn in (legs, entry, exit_, stop, clear, compare):
        print()
        print('=' * 78)
        print(fn.__name__.rstrip('_').upper())
        print('=' * 78)
        fn(list(argv))
    return 0


MODES = {'collect': collect, 'legs': legs, 'entry': entry, 'exit': exit_,
         'stop': stop, 'clear': clear, 'compare': compare, 'all': all_}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in MODES:
        print(__doc__)
        return 2
    return MODES[sys.argv[1]](sys.argv[2:])


if __name__ == '__main__':
    sys.exit(main())
