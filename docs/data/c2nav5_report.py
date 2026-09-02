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
"""C2-NAV.5 report: fresh-simulator validation of cost_scaling_factor 65.

  c2nav5_report.py collect   <results_dir> <out.json>
  c2nav5_report.py enclosure <results_dir | collected.json>
  c2nav5_report.py tour      <results_dir | collected.json>
  c2nav5_report.py cost      <cost.json> [...]

Nothing here computes a navigation result. It reads what `nav_bench.py`
and `c2nav5_costprobe.py` wrote and tabulates it.

`collect` exists so the record is reproducible from the repository
alone. The runs are driven out of `.navbench/`, a scratch directory that
C2-NAV.0 through C2-NAV.4 deliberately never committed; `collect` folds
its per-run `nav_bench` JSONs into one `docs/data/c2nav5_bench.json`,
and `enclosure` and `tour` read either that file or the scratch
directory and produce the same tables from either.

TRAVERSED and SUCCEEDED are kept in separate columns everywhere, and the
distinction is not cosmetic. `nav_bench` reports SUCCEEDED only when
`SimpleGoalChecker` is satisfied, which needs the goal YAW within
0.25 rad as well as the position; C2-NAV.4 recorded a CSF 30 approach
that ended 0.010 m from the goal and still reported TIMEOUT. So

  TRAVERSED := the robot came within `goal_xy_tolerance` (0.25 m) of the
               goal at any point in the leg. nav_bench records that as a
               non-null `t_transit_s`, and writes the note "never reached
               goal xy tolerance" when it did not.
  SUCCEEDED := nav_bench's action status.

A leg can be the first without being the second. Collapsing them hides
which mechanism a failure belongs to -- C2-NAV.4's or C2-NAV.1's.
"""
import glob
import json
import os
import statistics
import sys

COND = ['base', 'csf65']
COND_LABEL = {'base': 'CSF 5.0 (C2-NAV.0)', 'csf65': 'CSF 65.0'}
# nav_bench.py's TOUR order. Listed rather than derived so a leg that
# never ran shows up as a gap instead of vanishing from the table.
TOUR = ['open_space', 'wall_adjacent', 'wall_parallel', 'obstacle_corner',
        'corridor_gate', 'enclosure_entry', 'enclosure_exit']


def traversed(leg):
    """Did the robot reach the goal-checker's xy tolerance at all?"""
    return leg.get('t_transit_s') is not None


def _runs_from_dir(d, prefix):
    """[(tag, cond, rep, [legs])] from <dir>/<prefix>_<cond>_r*.json."""
    runs = []
    for c in COND:
        for p in sorted(glob.glob(os.path.join(d, f'{prefix}_{c}_r*.json'))):
            tag = os.path.basename(p)[:-5]
            rep = tag.rsplit('_r', 1)[-1]
            try:
                doc = json.load(open(p))
            except (OSError, ValueError) as e:
                print(f'  !! unreadable {p}: {e}')
                continue
            runs.append({'tag': tag, 'condition': c, 'rep': rep,
                         'legs': doc['legs']})
    return runs


def load(src, prefix):
    """{condition: [(run_index, leg), ...]}, from a scratch directory or
    from a collected JSON. Both paths must give the same tables, which is
    the whole point of collect()."""
    if os.path.isdir(src):
        runs = _runs_from_dir(src, prefix)
    else:
        runs = [r for r in json.load(open(src))['runs']
                if r['tag'].startswith(prefix)]
    out = {c: [] for c in COND}
    for r in runs:
        for leg in r['legs']:
            out[r['condition']].append((r['rep'], leg))
    return out


def collect(argv):
    """Fold every per-run nav_bench JSON in a scratch directory into one
    committed artifact."""
    d, out = argv[0], argv[1]
    runs = _runs_from_dir(d, 'c2n5_enc') + _runs_from_dir(d, 'c2n5_tour')
    doc = {'experiment': 'C2-NAV.5',
           'baseline': {'file': 'docs/data/c2nav3_baseline_params.yaml',
                        'sha256': 'dbcee9ca5da62677611fb03fc22edf4a26fcef5'
                                  'ccccfefc8e2b89efdb3b5bddb',
                        'local_cost_scaling_factor': 5.0},
           'candidate': {'file': 'docs/data/c2nav4_csf65_params.yaml',
                         'sha256': '3d9623d65edfcc4c40fc2bb2b72f38bea79c261'
                                   'a9b2e6e4304f1f545ba9b07bb',
                         'local_cost_scaling_factor': 65.0},
           'goal_xy_tolerance': 0.25,
           'n_runs': len(runs),
           'runs': runs}
    with open(out, 'w') as f:
        json.dump(doc, f, indent=1)
    enc = sum(1 for r in runs if r['tag'].startswith('c2n5_enc'))
    legs = sum(len(r['legs']) for r in runs)
    print(f'collected {len(runs)} runs ({enc} enclosure, {len(runs) - enc} '
          f'tour), {legs} legs -> {out}')
    return 0


def g(leg, k, dv='-'):
    v = leg.get(k)
    return dv if v is None else v


def med(vals):
    vals = [v for v in vals if v is not None]
    return round(statistics.median(vals), 3) if vals else None


def crawl(leg, k):
    w = leg.get('worst_crawl')
    return '-' if not w else w.get(k, '-')


def enclosure(argv):
    d = argv[0]
    runs = load(d, 'c2n5_enc')
    hdr = ('{:<6} {:>3} {:<10} {:>4} {:>7} {:>8} {:>8} {:>8} {:>7} {:>7} '
           '{:>7} {:>6} {:>6} {:>6} {:>5}')
    print(hdr.format('cond', 'r', 'status', 'trav', 'dur_s', 'goal_err',
                     't_trans', 'crawl_s', 'd@crawl', 'min_clr', 'scan_min',
                     'vx0_fr', 'illeg', 'cm_gat', 'stops'))
    for c in COND:
        for rep, leg in runs[c]:
            print(hdr.format(
                c, rep, str(g(leg, 'status'))[:10],
                'Y' if traversed(leg) else 'n',
                g(leg, 'duration_sim_s'), g(leg, 'final_goal_err_m'),
                g(leg, 't_transit_s'), crawl(leg, 'crawl_len_s'),
                crawl(leg, 'dist_to_goal_m'),
                g(leg, 'min_clearance_m'), g(leg, 'min_scan_range_m'),
                g(leg, 'dwb_best_vx_zero_frac'), g(leg, 'dwb_illegal_frac'),
                g(leg, 'cm_gated_frac'), g(leg, 'n_stops')))
    print()
    print('=== enclosure_entry, fresh simulators, traversed vs succeeded ===')
    print('{:<24} {:>4} {:>10} {:>10} {:>12} {:>12}'.format(
        'condition', 'N', 'traversed', 'SUCCEEDED', 'med dur (s)',
        'med err (m)'))
    for c in COND:
        legs = [leg for _, leg in runs[c]]
        n = len(legs)
        if not n:
            continue
        tr = sum(1 for leg in legs if traversed(leg))
        su = sum(1 for leg in legs if leg.get('status') == 'SUCCEEDED')
        ok = [leg for leg in legs if leg.get('status') == 'SUCCEEDED']
        print('{:<24} {:>4} {:>10} {:>10} {:>12} {:>12}'.format(
            COND_LABEL[c], n, f'{tr}/{n}', f'{su}/{n}',
            str(med([x.get('duration_sim_s') for x in ok]) or '-'),
            str(med([x.get('final_goal_err_m') for x in legs]))))
    print()
    print('=== movement, medians over each condition ===')
    keys = [('final_goal_err_m', 'final goal error m'),
            ('min_clearance_m', 'min clearance m'),
            ('min_scan_range_m', 'min scan range m'),
            ('med_clearance_m', 'median clearance m'),
            ('dwb_best_vx_zero_frac', 'DWB best vx = 0'),
            ('dwb_illegal_frac', 'illegal fraction'),
            ('dwb_illegal_frac_transit', 'illegal, transit'),
            ('v_cmd_med', 'median commanded vx'),
            ('transit_speed_mean', 'transit speed m/s'),
            ('n_stops', 'stops'),
            ('n_osc_cmd', 'angular sign flips'),
            ('n_reversals_cmd', 'linear reversals'),
            ('n_progress_failures', 'progress-checker aborts'),
            ('cm_gated_frac', 'collision monitor gated'),
            ('path_len_m', 'path driven m'),
            ('rtf', 'RTF')]
    print('{:<26} {:>16} {:>16}'.format('metric', COND_LABEL['base'],
                                        COND_LABEL['csf65']))
    for k, label in keys:
        row = []
        for c in COND:
            row.append(med([leg.get(k) for _, leg in runs[c]]))
        print('{:<26} {:>16} {:>16}'.format(
            label, str(row[0]), str(row[1])))
    print()
    print('=== worst crawl per run (the stall, when there is one) ===')
    for c in COND:
        for rep, leg in runs[c]:
            w = leg.get('worst_crawl') or {}
            print(f'  {c:<6} r{rep}  crawl {w.get("crawl_len_s", "-")} s at '
                  f'{w.get("dist_to_goal_m", "-")} m, pose '
                  f'{w.get("pose_world")}, cm={w.get("collision_monitor")}, '
                  f'free_band={w.get("free_band_m")}, '
                  f'cost_at_robot={w.get("cost_at_robot")}, '
                  f'dwb_vx={w.get("dwb_chosen_vx")}')
    return 0


def tour(argv):
    d = argv[0]
    runs = load(d, 'c2n5_tour')
    per = {c: {} for c in COND}
    for c in COND:
        for rep, leg in runs[c]:
            per[c].setdefault(leg.get('scenario'), []).append(leg)
    print('=== the seven tour legs, fresh simulators, 75 s per leg ===')
    hdr = '{:<17} {:<6} {:>3} {:>10} {:>10} {:>9} {:>9} {:>9} {:>8} {:>7}'
    print(hdr.format('scenario', 'cond', 'N', 'traversed', 'SUCCEEDED',
                     'med dur', 'med err', 'med clr', 'worst clr', 'osc/s'))
    for name in TOUR:
        for c in COND:
            legs = per[c].get(name, [])
            n = len(legs)
            if not n:
                print(hdr.format(name, c, 0, '-', '-', '-', '-', '-', '-',
                                 '-'))
                continue
            tr = sum(1 for leg in legs if traversed(leg))
            su = sum(1 for leg in legs if leg.get('status') == 'SUCCEEDED')
            clr = [leg.get('min_clearance_m') for leg in legs
                   if leg.get('min_clearance_m') is not None]
            print(hdr.format(
                name, c, n, f'{tr}/{n}', f'{su}/{n}',
                str(med([x.get('duration_sim_s') for x in legs])),
                str(med([x.get('final_goal_err_m') for x in legs])),
                str(med([x.get('min_clearance_m') for x in legs])),
                str(round(min(clr), 3)) if clr else '-',
                str(med([x.get('osc_per_sec') for x in legs]))))
    print()
    print('=== per-leg detail ===')
    hdr2 = ('{:<17} {:<6} {:>3} {:<9} {:>4} {:>7} {:>8} {:>8} {:>8} {:>6} '
            '{:>6} {:>5} {:>5} {:>5}')
    print(hdr2.format('scenario', 'cond', 'r', 'status', 'trav', 'dur_s',
                      'goal_err', 'min_clr', 'scan_min', 'vx0_fr', 'illeg',
                      'stops', 'osc', 'rev'))
    for name in TOUR:
        for c in COND:
            for rep, leg in runs[c]:
                if leg.get('scenario') != name:
                    continue
                print(hdr2.format(
                    name, c, rep, str(g(leg, 'status'))[:9],
                    'Y' if traversed(leg) else 'n',
                    g(leg, 'duration_sim_s'), g(leg, 'final_goal_err_m'),
                    g(leg, 'min_clearance_m'), g(leg, 'min_scan_range_m'),
                    g(leg, 'dwb_best_vx_zero_frac'),
                    g(leg, 'dwb_illegal_frac'), g(leg, 'n_stops'),
                    g(leg, 'n_osc_cmd'), g(leg, 'n_reversals_cmd')))
    print()
    print('=== leg totals over all tours ===')
    for c in COND:
        legs = [leg for _, leg in runs[c]]
        if not legs:
            continue
        tr = sum(1 for leg in legs if traversed(leg))
        su = sum(1 for leg in legs if leg.get('status') == 'SUCCEEDED')
        clr = [leg.get('min_clearance_m') for leg in legs
               if leg.get('min_clearance_m') is not None]
        print(f'  {COND_LABEL[c]:<20} {su}/{len(legs)} SUCCEEDED, '
              f'{tr}/{len(legs)} traversed, worst clearance '
              f'{min(clr) if clr else "-"} m, median leg '
              f'{med([x.get("duration_sim_s") for x in legs])} s')
    return 0


def cost(paths):
    print('=== transformed-plan cost field at the pinch, by distance rung ==')
    hdr = ('{:<26} {:>5} {:>6} {:>5} {:>4} {:>4} {:>6} {:>4} {:>6} {:>6} '
           '{:>8} {:>8} {:>8} {:>8}')
    print(hdr.format('capture', 'rung', 'd_goal', 'n_pl', 'min', 'med',
                     'max', 'n0', 'pinch', 'atrbt', 'chos_vx', 'BaseOb',
                     'fwd_tot', 'zero_tot'))
    for p in paths:
        d = json.load(open(p))
        tag = os.path.basename(p).replace('_cost.json', '')
        print(f"-- {tag}  closest approach {d.get('closest_approach_m')} m, "
              f"{len(d['snapshots'])} rungs")
        for s in d['snapshots']:
            pl = s['plan']
            ch = s['chosen']
            f = s.get('best_forward') or {}
            z = s.get('zero') or {}
            print(hdr.format(
                '', s['rung_m'], s['dist_to_goal'], pl.get('n', '-'),
                pl.get('min', '-'), pl.get('median', '-'),
                pl.get('max', '-'), pl.get('n_zero', '-'),
                (s.get('pinch') or {}).get('cost', '-'),
                s.get('cost_at_robot', '-'),
                f"{ch['vx']:.4f}",
                ('-' if ch['BaseObstacle_scaled'] is None
                 else f"{ch['BaseObstacle_scaled']:.2f}"),
                ('-' if 'total' not in f else f"{f['total']:.2f}"),
                ('-' if 'total' not in z else f"{z['total']:.2f}")))
    return 0


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    mode, rest = sys.argv[1], sys.argv[2:]
    return {'collect': collect, 'enclosure': enclosure, 'tour': tour,
            'cost': cost}[mode](rest)


if __name__ == '__main__':
    sys.exit(main())
