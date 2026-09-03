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
"""C2-NAV.6 report: the PolygonStop trigger on the enclosure exit.

  c2nav6_report.py collect <results_dir> <out.json>
  c2nav6_report.py legs    <results_dir | collected.json>
  c2nav6_report.py stop    <results_dir | collected.json>

Nothing here computes a result. It reads what `nav_bench.py` and
`c2nav6_stopprobe.py` wrote and tabulates it, the same arrangement
`c2nav5_report.py` uses, and for the same reason: the runs are driven out
of `.navbench/`, a scratch directory C2-NAV.0 through C2-NAV.5
deliberately never committed, so `collect` folds the per-run JSONs into
one committed artifact and `legs`/`stop` read either and produce the same
tables.

TRAVERSED and SUCCEEDED stay in separate columns, C2-NAV.5's definitions
unchanged:

  TRAVERSED := the robot came within goal_xy_tolerance (0.25 m) of the
               goal at any point, i.e. nav_bench recorded a non-null
               `t_transit_s`.
  SUCCEEDED := nav_bench's action status, which needs the goal YAW too.

`stop` is the new table and it is the point of the experiment. For every
frame in which the collision monitor actually reported STOP it prints how
many laser returns were inside the 0.25 m circle at that moment, and the
suppression curve: for each candidate `min_points`, how many of those
frames would still have triggered. A candidate value is diagnostic only
where that column reaches zero.
"""
import glob
import json
import os
import statistics
import sys

COND = ['base', 'cand']
COND_LABEL = {'base': 'min_points 4 (C2-NAV.5 candidate config)',
              'cand': 'min_points RAISED'}
LEGS = ['enclosure_entry', 'enclosure_exit']


def traversed(leg):
    """Did the robot reach the goal-checker's xy tolerance at all?"""
    return leg.get('t_transit_s') is not None


def _runs_from_dir(d):
    """[{tag, condition, rep, legs, stop}] from <dir>/c2n6_<cond>_r*."""
    runs = []
    for c in COND:
        for p in sorted(glob.glob(os.path.join(d, f'c2n6_{c}_r*.json'))):
            # The glob also catches this run's two sidecars. They are the
            # probe's output, not nav_bench's, and have no 'legs'.
            if p.endswith(('_stop.json', '_geom.json')):
                continue
            tag = os.path.basename(p)[:-5]
            try:
                doc = json.load(open(p))
            except (OSError, ValueError) as e:
                print(f'  !! unreadable {p}: {e}')
                continue
            side = {}
            for key in ('stop', 'geom'):
                sp = os.path.join(d, f'{tag}_{key}.json')
                if os.path.exists(sp):
                    try:
                        side[key] = json.load(open(sp))
                    except (OSError, ValueError) as e:
                        print(f'  !! unreadable {sp}: {e}')
            runs.append({'tag': tag, 'condition': c,
                         'rep': tag.rsplit('_r', 1)[-1],
                         'legs': doc['legs'],
                         'stop': side.get('stop'),
                         'geom': side.get('geom')})
    return runs


def load(src):
    if os.path.isdir(src):
        return _runs_from_dir(src)
    return json.load(open(src))['runs']


def collect(argv):
    d, out = argv[0], argv[1]
    runs = _runs_from_dir(d)
    doc = {'experiment': 'C2-NAV.6',
           'baseline': {
               'file': 'docs/data/c2nav4_csf65_params.yaml',
               'sha256': '3d9623d65edfcc4c40fc2bb2b72f38bea79c261a'
                         '9b2e6e4304f1f545ba9b07bb',
               'PolygonStop.min_points': 4},
           'candidate': {
               'file': 'docs/data/c2nav6_minpts7_params.yaml',
               'sha256': '437b00b3544a368c87391d43b3ded452405b7400'
                         '6657f99247dd412d38a7870b',
               'PolygonStop.min_points': 7},
           'goal_xy_tolerance': 0.25,
           'n_runs': len(runs),
           'runs': runs}
    with open(out, 'w') as f:
        json.dump(doc, f, indent=1)
    legs = sum(len(r['legs']) for r in runs)
    print(f'collected {len(runs)} runs, {legs} legs -> {out}')
    return 0


def g(leg, k, dv='-'):
    v = leg.get(k)
    return dv if v is None else v


def legs_table(argv):
    runs = load(argv[0])
    print('| run | condition | leg | traversed | status | goal err | '
          'driven | clear | v_cmd med | v_wheel med | dur |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    for r in runs:
        for leg in r['legs']:
            if leg.get('scenario') not in LEGS:
                continue
            print(f'| {r["tag"]} | {r["condition"]} | {leg["scenario"]} '
                  f'| {"yes" if traversed(leg) else "NO"} '
                  f'| {g(leg, "status")} '
                  f'| {g(leg, "final_goal_err_m")} m '
                  f'| {g(leg, "path_len_m")} m '
                  f'| {g(leg, "min_clearance_m")} m '
                  f'| {g(leg, "v_cmd_med")} '
                  f'| {g(leg, "v_wheel_med")} '
                  f'| {g(leg, "duration_sim_s")} s |')
    return 0


def stop_table(argv):
    runs = load(argv[0])
    print('### PolygonStop trigger, per leg')
    print()
    print('| run | condition | leg | frames | STOP frames | STOP frac | '
          'points inside circle when STOP (min/med/max) | nearest return '
          'from base | v_nav med | v_wheel med | driven |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    ladders = {}
    for r in runs:
        s = r.get('stop')
        if not s:
            continue
        for name in LEGS:
            e = s.get('legs', {}).get(name)
            if not e:
                continue
            c = e.get('n_in_stop_when_STOP')
            cc = ('n/a (monitor never reported STOP)' if not c
                  else f'{c["min"]} / {c["median"]} / {c["max"]}')
            print(f'| {r["tag"]} | {r["condition"]} | {name} '
                  f'| {e["n_rows"]} | {e["n_stop_rows"]} '
                  f'| {e["stop_frac"]} | {cc} '
                  f'| {e.get("d_min_base_m_min", "-")} m '
                  f'| {e.get("v_nav_median", "-")} '
                  f'| {e.get("v_wheel_median", "-")} '
                  f'| {e.get("gt_path_len_m", "-")} m |')
            if e.get('suppression_curve'):
                ladders[(r['tag'], name)] = e['suppression_curve']
    if ladders:
        print()
        print('### Suppression curve: STOP frames that would SURVIVE each '
              'min_points')
        print()
        keys = sorted({int(k) for d in ladders.values() for k in d})
        print('| run / leg | ' + ' | '.join(str(k) for k in keys) + ' |')
        print('|---' * (len(keys) + 1) + '|')
        for (tag, name), d in ladders.items():
            row = ' | '.join(str(d.get(str(k), '-')) for k in keys)
            print(f'| {tag} {name} | {row} |')
        print()
        print('A value is diagnostic only where its column reaches 0: that '
              'is the threshold at which the observed STOP would not have '
              'fired.')
    print()
    for r in runs:
        s = r.get('stop')
        if not s:
            continue
        pc = s.get('positive_control', {})
        print(f'{r["tag"]}: control ok={pc.get("ok")} '
              f'(monitor states={pc.get("collision_monitor_state_msgs")}, '
              f'rows with a wheel command={pc.get("rows_with_wheel_cmd")}), '
              f'circle r={s.get("stop_radius_m")} m about '
              f'{s.get("base_frame")}, lidar at '
              f'{s.get("tf_base_from_lidar")} = '
              f'{s.get("lidar_offset_from_base_origin_m")} m off centre')
    return 0


def med(vals):
    vals = [v for v in vals if v is not None]
    return round(statistics.median(vals), 4) if vals else None


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    mode, rest = sys.argv[1], sys.argv[2:]
    if mode == 'collect':
        return collect(rest)
    if mode == 'legs':
        return legs_table(rest)
    if mode == 'stop':
        return stop_table(rest)
    print(f'unknown mode {mode}')
    return 2


if __name__ == '__main__':
    sys.exit(main())
