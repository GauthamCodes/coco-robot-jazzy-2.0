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
"""C2-NAV.8 report: the seven-leg tour at the shifted enclosure goal.

  c2nav8_report.py collect <results_dir> [out.json]
  c2nav8_report.py legs    [results_dir | collected.json]
  c2nav8_report.py entry   [results_dir | collected.json]
  c2nav8_report.py exit    [results_dir | collected.json]
  c2nav8_report.py stop    [results_dir | collected.json]
  c2nav8_report.py clear   [results_dir | collected.json]
  c2nav8_report.py compare [results_dir | collected.json]
  c2nav8_report.py all     [results_dir | collected.json]

Nothing here computes a navigation result. It reads what `nav_bench.py`
and `c2nav6_stopprobe.py` wrote and tabulates it -- the arrangement
`c2nav5_report.py` and `c2nav6_report.py` use, for the same reason: the
runs are driven out of `.navbench/`, a scratch directory C2-NAV.0 through
C2-NAV.7 deliberately never committed, so `collect` folds the per-run
JSONs into one committed artifact and every other mode reads either and
produces the same tables.

Three things are new here, and each exists because C2-NAV.8 is a
SEVEN-leg tour where C2-NAV.6 and C2-NAV.7 were two-leg runs.

1. SEVEN-LEG SEGMENTATION OF THE PROBE CSV. `c2nav6_stopprobe.py` labels
   a row by the tail of `/plan`, against a `LEG_GOALS` table that holds
   only the two enclosure legs; everything else lands in `other`. The
   probe is deliberately NOT modified -- a count taken by a changed
   instrument is not comparable to C2-NAV.6's and C2-NAV.7's counts by
   construction, and that comparability is the whole evidential value of
   the STOP-frame numbers. So the CSV is re-segmented HERE, offline,
   against all seven map-frame goals. The probe's own JSON stays in the
   artifact as an independent cross-check on the two legs it does know.

2. TRUE CLEARANCE, NOT `min_clearance_m`. C2-NAV.7 measured `nav_bench`'s
   `min_clearance_m` disagreeing with exact world-file geometry by up to
   106 mm IN BOTH DIRECTIONS -- reporting 0.339 m for the one leg that
   genuinely entered the stop circle at 0.2437 m. It is the distance to
   the nearest occupied MAP cell, 360 deg but quantised to the 5 cm grid.
   So `clear` computes the exact distance from the recorded ground-truth
   track to the world file's collision faces, BY IMPORTING
   `c2nav7_geom.py`'s `nearest`, `BOXES`, `STOP_RADIUS` and
   `CIRCUMSCRIBED` rather than restating them -- so these numbers are
   comparable to C2-NAV.7's by construction too. `min_clearance_m` is
   still printed, labelled QUANTISED, because hiding a metric is not the
   same as correcting it.

3. THE C2-NAV.5 COMPARISON IS A FILE, NOT A MEMORY. `compare` reads
   `c2nav5_bench.json`'s three `c2n5_tour_csf65_r*` runs -- the same
   parameter file, the same seven legs, the committed 18/21 -- so the
   only difference on the table is the goal.

TRAVERSED and SUCCEEDED stay in separate columns, C2-NAV.5's definitions
unchanged:

  TRAVERSED := the robot came within goal_xy_tolerance (0.25 m) of the
               goal at any point, i.e. nav_bench recorded a non-null
               `t_transit_s`.
  SUCCEEDED := nav_bench's action status, which needs the goal YAW too.
"""
import glob
import json
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c2nav7_geom import (                                  # noqa: E402
    BOXES, CIRCUMSCRIBED, STOP_RADIUS, dist_to_box, nearest)

# ---------------------------------------------------------------------
# The rest of the world, which C2-NAV.7's eight-box list does not hold.
#
# c2nav7_geom.py's BOXES is the geometry the ENCLOSURE legs can touch,
# and for those two legs it is complete and exact -- C2-NAV.7 only ever
# used `track` there. A SEVEN-leg tour drives past things that list does
# not contain, and a clearance computed from an incomplete world reads as
# MORE clearance than there is, which is the one direction a safety
# number must never be wrong in. Measured on C2-NAV.8 tour r1 before this
# was added: `corridor_gate` scored 0.6254 m against the laser's
# 0.3795 m, a 246 mm overstatement, because the nearest thing on that leg
# is the cylinder and the cylinder was not in the list.
#
# So the missing static collision geometry is added here rather than in
# c2nav7_geom.py, which stays byte-identical so C2-NAV.7's numbers
# reproduce exactly.
#
#   gazebo_models/worlds/coco_world.world -- cylinder_obstacle and the
#   two east-corridor pilasters, verbatim from the <collision> tags.
#   full_world_robo.launch.py -- the ramp wedge and the platform, whose
#   PLAN-VIEW footprints are boxes even though the wedge is not: the
#   robot drives on the ground, so the footprint is what its hull can
#   reach. Constants from coco_config.robot (RAMP_FOOT_X 1.0,
#   RAMP_RUN 2.0 so RAMP_SUMMIT_X 3.0, RAMP_WIDTH 2.5, PLATFORM_LEN 1.5).
EXTRA_BOXES = [
    ('feature_pilaster_north', 7.72, 2.0, 0.3, 0.5),
    ('feature_pilaster_south', 7.72, -1.4, 0.3, 0.5),
    ('ramp_footprint', 2.0, 0.0, 2.0, 2.5),
    ('ramp_platform', 3.75, 0.0, 1.5, 2.5),
]
# (name, centre_x, centre_y, radius)
CIRCLES = [
    ('cylinder_obstacle', -0.2, 0.6, 0.2),
]


def nearest_full(px, py):
    """Nearest collision face over the WHOLE world, not just C2-NAV.7's.

    Same return shape as c2nav7_geom.nearest: a sorted list of
    (distance, name, closest point).
    """
    out = nearest(px, py)
    for b in EXTRA_BOXES:
        d, q = dist_to_box(px, py, b)
        out.append((d, b[0], q))
    for name, cx, cy, r in CIRCLES:
        h = ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5
        f = max(h, 1e-9)
        out.append((max(h - r, 0.0), name,
                    (cx + (px - cx) * r / f, cy + (py - cy) * r / f)))
    out.sort()
    return out

DEFAULT_DIR = os.path.normpath(os.path.join(
    HERE, '..', '..', '.navbench', 'results'))
COLLECTED = os.path.join(HERE, 'c2nav8_bench.json')
C2NAV5 = os.path.join(HERE, 'c2nav5_bench.json')

PARAMS = os.path.join(HERE, 'c2nav4_csf65_params.yaml')
PARAMS_SHA = '3d9623d65edfcc4c40fc2bb2b72f38bea79c261a9b2e6e4304f1f545ba9b07bb'

GOAL_ORIGINAL = (-3.45, 2.95)
GOAL_SHIFTED = (-3.575, 2.95)

# nav_bench.py TOUR order. The report never re-derives it; it is the
# committed ordering and the legs are reported in it.
LEGS = ['open_space', 'wall_adjacent', 'wall_parallel', 'obstacle_corner',
        'corridor_gate', 'enclosure_entry', 'enclosure_exit']

# nav_bench.py: map = world + (2, 0). The probe records the tail of
# /plan in the MAP frame, so the seven-leg segmentation table is built
# from TOUR by the same transform the bench sends goals through, with
# enclosure_entry at the SHIFTED goal this experiment actually ran.
WORLD_TO_MAP = (2.0, 0.0)
TOUR_WORLD = {
    'open_space': (-2.00, -2.20),
    'wall_adjacent': (-2.00, -3.00),
    'wall_parallel': (0.50, -2.95),
    'obstacle_corner': (0.30, -0.30),
    'corridor_gate': (-2.60, -0.10),
    'enclosure_entry': GOAL_SHIFTED,
    'enclosure_exit': (-2.00, 0.00),
}
# Two legs' goals are 0.9 m apart in map x and the probe's own tolerance
# is 0.3 m; 0.25 m here is under half the smallest inter-goal distance,
# so a row cannot be attributed to two legs.
LEG_MATCH_M = 0.25


def traversed(leg):
    """Did the robot reach the goal-checker's xy tolerance at all?"""
    return leg.get('t_transit_s') is not None


def _runs_from_dir(d):
    """[{tag, rep, legs, stop}] from <dir>/c2n8_tour_r*.json."""
    runs = []
    for p in sorted(glob.glob(os.path.join(d, 'c2n8_tour_r*.json')),
                    key=lambda q: (len(q), q)):
        # The glob also catches the probe's sidecar, which is not
        # nav_bench's output and has no 'legs'.
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
        runs.append(run)
    return runs


def load(src=None):
    """Runs from a results directory or from the collected JSON."""
    src = src or (COLLECTED if os.path.exists(COLLECTED) else DEFAULT_DIR)
    if os.path.isdir(src):
        return _runs_from_dir(src), src
    doc = json.load(open(src))
    return doc['runs'], src


def collect(argv):
    """Fold the scratch directory's per-run JSONs into one artifact."""
    d = argv[0] if argv else DEFAULT_DIR
    out = argv[1] if len(argv) > 1 else COLLECTED
    runs = _runs_from_dir(d)
    if not runs:
        print(f'no c2n8_tour_r*.json under {d}')
        return 1
    # The CSV path is a property of this machine, not of the result.
    for r in runs:
        r.pop('stop_csv', None)
    doc = {
        'experiment': 'C2-NAV.8',
        'question': 'does the seven-leg tour hold at the shifted '
                    'enclosure_entry goal, CSF 65?',
        'params_file': 'docs/data/c2nav4_csf65_params.yaml',
        'params_sha256': PARAMS_SHA,
        'navigation_change_vs_c2nav5': 'none',
        'goal_original_world': list(GOAL_ORIGINAL),
        'goal_shifted_world': list(GOAL_SHIFTED),
        'goal_offset_m': [round(GOAL_SHIFTED[0] - GOAL_ORIGINAL[0], 4),
                          round(GOAL_SHIFTED[1] - GOAL_ORIGINAL[1], 4)],
        'PolygonStop': {'radius': STOP_RADIUS, 'min_points': 4},
        'goal_xy_tolerance': 0.25,
        'leg_timeout_s': {n: (200.0 if n == 'enclosure_entry' else 75.0)
                          for n in LEGS},
        'reference': 'docs/data/c2nav5_bench.json (c2n5_tour_csf65_r1..r3), '
                     'same params file, goal -3.45',
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


def _fmt(v, n=3, w=None):
    if v is None:
        return '—'.rjust(w) if w else '—'
    s = f'{v:.{n}f}'
    return s.rjust(w) if w else s


# --------------------------------------------------------------------
# the tables
# --------------------------------------------------------------------
def legs(argv):
    runs, src = load(argv[0] if argv else None)
    print(f'source: {src}')
    print()
    print('C2-NAV.8 -- seven-leg tour, CSF 65, enclosure_entry goal '
          f'{GOAL_SHIFTED[0]} (shifted {(GOAL_SHIFTED[0]-GOAL_ORIGINAL[0])*1000:+.0f} mm)')
    print('TRAV = came within the 0.25 m xy tolerance.  SUCC = the action '
          'status, which needs the goal yaw too.')
    print('clear_q = nav_bench min_clearance_m, QUANTISED to the 5 cm map '
          'grid -- see `clear` for the exact number.')
    print()
    for run in runs:
        cap = {lg['scenario']: lg.get('timeout_s') for lg in run['legs']}
        print(f'--- {run["tag"]} ---')
        print(f'  {"leg":<16} {"status":<10} {"TRAV":<5} {"cap":>5} '
              f'{"dur_s":>7} {"err_m":>7} {"clear_q":>8} {"path_m":>7} '
              f'{"v_med":>6} {"stops":>5} {"prog":>4} {"still":>6}')
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
                  f'{str(lg.get("n_stops", "—")):>5} '
                  f'{str(lg.get("n_progress_failures", "—")):>4} '
                  f'{_fmt(lg.get("frac_cmd_below_0.05"), 3, 6)}')
        print(f'  TOTAL  SUCCEEDED {n_succ}/{len(LEGS)}   '
              f'TRAVERSED {n_trav}/{len(LEGS)}')
        print()
    tot_s = sum(1 for r in runs for n in LEGS
                if (_leg(r, n) or {}).get('status') == 'SUCCEEDED')
    tot_t = sum(1 for r in runs for n in LEGS
                if traversed(_leg(r, n) or {}))
    n = len(runs) * len(LEGS)
    print(f'C2-NAV.8 TOTAL: SUCCEEDED {tot_s}/{n}   TRAVERSED {tot_t}/{n}   '
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


def _detail(runs, name, extra):
    print(f'  {"run":<16} {"status":<10} {"TRAV":<5} {"dur_s":>7} '
          f'{"err_m":>7} {"STOP":>6} {"in_stop":>8} {"d_min_probe":>12}')
    for run in runs:
        lg = _leg(run, name)
        if lg is None:
            continue
        st = ((run.get('stop') or {}).get('legs') or {}).get(name, {})
        print(f'  {run["tag"]:<16} {lg["status"]:<10} '
              f'{"yes" if traversed(lg) else "NO":<5} '
              f'{_fmt(lg.get("duration_sim_s"), 2, 7)} '
              f'{_fmt(lg.get("final_goal_err_m"), 3, 7)} '
              f'{str(st.get("n_stop_rows", "—")):>6} '
              f'{str(st.get("n_in_stop_max", "—")):>8} '
              f'{_fmt(st.get("d_min_base_m_min"), 4, 12)}')
    print()
    for label, key, nd in extra:
        row = ' '.join(
            f'{run["tag"].split("_")[-1]}={_fmt((_leg(run, name) or {}).get(key), nd)}'
            for run in runs if _leg(run, name))
        print(f'  {label:<34} {row}')
    return 0


def entry(argv):
    runs, src = load(argv[0] if argv else None)
    print(f'source: {src}')
    print()
    print(f'enclosure_entry at the SHIFTED goal {GOAL_SHIFTED}, cap 200 s')
    print('C2-NAV.7 (two-leg runs, cap 150 s): SUCCEEDED 1/3, TRAVERSED 3/3, '
          '116.56 / 150.68 / 150.01 s')
    print('C2-NAV.5 (tour, original goal, cap 75 s): SUCCEEDED 2/3, '
          'TRAVERSED 3/3')
    print()
    return _detail(runs, 'enclosure_entry', [
        ('time to xy tolerance   t_transit_s', 't_transit_s', 2),
        ('time settling the yaw  t_terminal_s', 't_terminal_s', 2),
        ('terminal share of leg', 'terminal_frac_of_leg', 3),
        ('mean transit speed  m/s', 'transit_speed_mean', 4),
        ('median commanded vx m/s', 'v_cmd_med', 4),
        ('median wheel vx     m/s', 'v_wheel_med', 4),
        ('DWB best-vx zero fraction', 'dwb_best_vx_zero_frac', 3),
        ('fraction of leg commanded <0.05', 'frac_cmd_below_0.05', 3),
        ('fraction of leg actually <0.05', 'frac_actual_below_0.05', 3),
        ('progress-checker failures', 'n_progress_failures', 0),
        ('stalls (n_stops)', 'n_stops', 0),
        ('collision-monitor gated fraction', 'cm_gated_frac', 3),
        ('path length m', 'path_len_m', 3),
        ('RTF', 'rtf', 3),
    ])


def exit_(argv):
    runs, src = load(argv[0] if argv else None)
    print(f'source: {src}')
    print()
    print('enclosure_exit -- the leg C2-NAV.5 could not pass (1/3) and '
          'C2-NAV.6 could not fix with min_points')
    print('C2-NAV.7 (two-leg runs): SUCCEEDED 3/3, 41.42 / 33.19 / 33.27 s, '
          '0 STOP frames on 5325 frames')
    print()
    rc = _detail(runs, 'enclosure_exit', [
        ('median commanded vx m/s', 'v_cmd_med', 4),
        ('median wheel vx     m/s', 'v_wheel_med', 4),
        ('p95 commanded vx    m/s', 'v_cmd_p95', 4),
        ('p95 wheel vx        m/s', 'v_wheel_p95', 4),
        ('collision-monitor gated fraction', 'cm_gated_frac', 3),
        ('path length m', 'path_len_m', 3),
        ('progress-checker failures', 'n_progress_failures', 0),
    ])
    print()
    print('  THE COMMAND CHAIN, per leg, from the probe CSV '
          '(median over rows with a command):')
    print(f'  {"run":<16} {"n":>6} {"v_nav":>8} {"v_smoothed":>11} '
          f'{"v_out":>8} {"v_wheel":>8}')
    for run in runs:
        rows = [r for r in _csv_rows(run)
                if _row_leg(r) == 'enclosure_exit']
        if not rows:
            continue
        med = {}
        for k in ('v_nav', 'v_smoothed', 'v_out', 'v_wheel'):
            vals = [float(r[k]) for r in rows if r.get(k) not in (None, '')]
            med[k] = statistics.median(vals) if vals else None
        print(f'  {run["tag"]:<16} {len(rows):>6} '
              f'{_fmt(med["v_nav"], 4, 8)} {_fmt(med["v_smoothed"], 4, 11)} '
              f'{_fmt(med["v_out"], 4, 8)} {_fmt(med["v_wheel"], 4, 8)}')
    return rc


# --------------------------------------------------------------------
# the probe CSV, re-segmented across all seven legs
# --------------------------------------------------------------------
def _csv_rows(run):
    """The probe CSV for one run, from scratch or from docs/data.

    The run TAG is `c2n8_tour_rN` -- short, because `ros_clean.sh`'s
    `nav[2]_` pattern matches any command line containing that substring
    and every helper in this series is named to stay clear of it. The
    COMMITTED artifact is `c2navN_...`, matching its siblings in
    docs/data. So the committed name is tried under both spellings and a
    fresh clone with no `.navbench/` finds the data.
    """
    import csv
    names = [f'{run["tag"]}_stop.csv']
    if run['tag'].startswith('c2n8_'):
        names.append('c2nav8_' + run['tag'][len('c2n8_'):] + '_stop.csv')
    # docs/data BEFORE the scratch directory, and every spelling of the
    # committed name before any scratch one. Otherwise a machine that
    # still has `.navbench/` silently reads the scratch CSV, the
    # committed copy is never exercised, and the "reproduces from the
    # repository alone" check passes without ever having tested it.
    cands = [run.get('stop_csv')] if run.get('stop_csv') else []
    cands += [os.path.join(HERE, n) for n in names]
    cands += [os.path.join(DEFAULT_DIR, n) for n in names]
    for path in cands:
        if path and os.path.exists(path):
            return list(csv.DictReader(open(path)))
    return []


def _row_leg(r):
    """Which of the SEVEN legs this row belongs to, from the /plan tail."""
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
    print('PolygonStop, re-segmented over all SEVEN legs from the probe CSV.')
    print(f'radius {STOP_RADIUS} m about the base_footprint origin, '
          'min_points 4, STRICT x^2+y^2 < r^2, applied by '
          'c2nav6_stopprobe.py unchanged.')
    print('C2-NAV.6 baseline for reference: 6 returns inside the circle on '
          '1470 of 1470 STOP frames.')
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
    print(f'C2-NAV.8 TOTAL: {grand["stop"]} STOP frames on {grand["rows"]} '
          f'recorded frames across {len(runs)} tours')
    print()
    print('  probe-JSON cross-check on the two legs its own LEG_GOALS table '
          'knows (unmodified instrument):')
    for run in runs:
        for name, st in ((run.get('stop') or {}).get('legs') or {}).items():
            if name in ('enclosure_entry', 'enclosure_exit'):
                print(f'    {run["tag"]:<16} {name:<16} '
                      f'rows={st["n_rows"]:<6} STOP={st["n_stop_rows"]:<6} '
                      f'max_in_circle={st["n_in_stop_max"]}')
        ctl = (run.get('stop') or {}).get('positive_control')
        if ctl is not None:
            print(f'    {run["tag"]:<16} positive control ok={ctl["ok"]} '
                  f'(monitor msgs={ctl["collision_monitor_state_msgs"]}, '
                  f'rows with a wheel cmd={ctl["rows_with_wheel_cmd"]})')
    return 0


def clear(argv):
    """Exact clearance along every leg, from world-file collision faces."""
    runs, src = load(argv[0] if argv else None)
    print(f'source: {src}')
    print()
    print('TRUE minimum clearance: distance from the recorded ground-truth '
          'track to the nearest collision')
    print(f'face of the whole world -- C2-NAV.7\'s {len(BOXES)} boxes plus '
          f'{len(EXTRA_BOXES)} more and {len(CIRCLES)} cylinder. 360 deg')
    print('and unquantised, unlike nav_bench\'s min_clearance_m (5 cm map '
          'grid; wrong by up to 106 mm')
    print('in BOTH directions in C2-NAV.7) and unlike the probe\'s '
          'd_min_base_m (exact, but the lidar')
    print('is 240 deg and blind behind). laser = that d_min_base_m, printed '
          'as an INDEPENDENT check:')
    print('where geometry and laser agree the number is trustworthy; where '
          'geometry is much LARGER,')
    print('something is missing from the world list and the row is marked '
          '(!).')
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
    """C2-NAV.8 against C2-NAV.5's committed CSF 65 tour."""
    runs, src = load(argv[0] if argv else None)
    print(f'source: {src}')
    if not os.path.exists(C2NAV5):
        print(f'  !! {C2NAV5} missing; cannot compare')
        return 1
    ref = json.load(open(C2NAV5))
    ref_runs = [r for r in ref['runs'] if r['tag'].startswith('c2n5_tour_csf65')]
    print(f'reference: {C2NAV5}  '
          f'({", ".join(r["tag"] for r in ref_runs)})')
    print()
    print('BOTH SIDES ARE CSF 65 ON THE SAME PARAMETER FILE. The only '
          'difference is the enclosure_entry')
    print(f'goal: C2-NAV.5 {GOAL_ORIGINAL}, C2-NAV.8 {GOAL_SHIFTED}. '
          'C2-NAV.5 ran 75 s on every leg;')
    print('C2-NAV.8 ran 75 s on six and 200 s on enclosure_entry -- so its '
          'entry column is NOT')
    print('capped where C2-NAV.5\'s was, and that is stated rather than '
          'normalised away.')
    print()
    print(f'  {"leg":<16} {"C2-NAV.5 SUCC":>14} {"C2-NAV.8 SUCC":>14} '
          f'{"C2-NAV.5 TRAV":>14} {"C2-NAV.8 TRAV":>14}')
    t5 = t8 = 0
    for name in LEGS:
        s5 = sum(1 for r in ref_runs
                 if (_leg(r, name) or {}).get('status') == 'SUCCEEDED')
        s8 = sum(1 for r in runs
                 if (_leg(r, name) or {}).get('status') == 'SUCCEEDED')
        v5 = sum(1 for r in ref_runs if traversed(_leg(r, name) or {}))
        v8 = sum(1 for r in runs if traversed(_leg(r, name) or {}))
        t5 += s5
        t8 += s8
        print(f'  {name:<16} {f"{s5}/{len(ref_runs)}":>14} '
              f'{f"{s8}/{len(runs)}":>14} {f"{v5}/{len(ref_runs)}":>14} '
              f'{f"{v8}/{len(runs)}":>14}')
    print(f'  {"TOTAL":<16} {f"{t5}/{len(ref_runs)*len(LEGS)}":>14} '
          f'{f"{t8}/{len(runs)*len(LEGS)}":>14}')
    print()
    print('  DURATION, median seconds per leg:')
    print(f'  {"leg":<16} {"C2-NAV.5":>10} {"C2-NAV.8":>10} {"delta":>10}')
    for name in LEGS:
        d5 = [(_leg(r, name) or {}).get('duration_sim_s') for r in ref_runs]
        d8 = [(_leg(r, name) or {}).get('duration_sim_s') for r in runs]
        d5 = [x for x in d5 if x is not None]
        d8 = [x for x in d8 if x is not None]
        m5 = statistics.median(d5) if d5 else None
        m8 = statistics.median(d8) if d8 else None
        dd = (m8 - m5) if (m5 is not None and m8 is not None) else None
        print(f'  {name:<16} {_fmt(m5, 2, 10)} {_fmt(m8, 2, 10)} '
              f'{_fmt(dd, 2, 10)}')
    print()
    print('  NOT A CLEAN CONTROL, and C2-NAV.5 said so itself: its '
          'enclosure_exit 3/3 at the')
    print('  BASELINE was measured after an entry that always failed, so the '
          'robot was never')
    print('  inside the pocket. The CSF-65 column above is the like-for-like '
          'one.')
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
