#!/usr/bin/env python3
"""C2-NAV.39 -- per-leg tour comparison against the established baselines.

Offline.  Reads nav_bench.py outputs and prints, per TOUR leg and per arm,
the metrics the C2-NAV series already reports -- no new metric is invented
here:

  succeeded         legs SUCCEEDED / legs attempted
  median s          median duration_sim_s over every attempt (TIMEOUTs included,
                    as the C2-NAV.8 table does)
  goal err          median final_goal_err_m
  |yaw err|         median |final_yaw_err_rad| (C2-NAV.39 field; absent from
                    older baselines, printed as "--")
  true clear        minimum distance from the ground-truth track to a world-file
                    collision box, 360 deg and unquantised (c2nav7_geom.py) --
                    NOT nav_bench's map-cell min_clearance_m, which C2-NAV.7
                    measured to be wrong by up to 106 mm in both directions
  PolygonStop       total seconds the collision monitor held PolygonStop, and
                    on how many legs it fired at all
  DWB zero-vx       median dwb_best_vx_zero_frac (the zero-velocity crawl/stall
                    signature C2-NAV.20/.21 measured)
  cmd<0.05          median frac_cmd_below_0.05

Arms:
  C2-NAV.5   docs/data/c2nav5_bench.json, c2n5_tour_csf65_r1..3 -- CSF 65,
             BaseObstacle 8.0, the committed TOUR, 1 repeat per fresh sim, 75 s
  C2-NAV.37  run directories of the unfixed default config (BaseObstacle 2.0,
             CSF 5.0), 3 repeats per fresh sim, 75 s
  C2-NAV.39  run directories written by gazebo_models/scripts/nav_tour_run.sh

Run from the worktree root:

  python3 -P docs/data/c2nav39_tour_report.py selftest
  python3 -P docs/data/c2nav39_tour_report.py report \\
      --c2nav5 docs/data/c2nav5_bench.json \\
      --c2nav37 ~/coco_nav_runs/c2nav37_run0 ... \\
      --c2nav39 ~/coco_nav_runs/baseline/baseline_r01 ... [--json out.json]
"""

import argparse
import csv
import glob
import importlib.util
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOUR_ORDER = ['open_space', 'wall_adjacent', 'wall_parallel', 'obstacle_corner',
              'corridor_gate', 'enclosure_entry', 'enclosure_exit']


def _load_geom():
    spec = importlib.util.spec_from_file_location(
        'c2nav7_geom', os.path.join(HERE, 'c2nav7_geom.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GEOM = _load_geom()


# ---------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------

def load_trace_xy(path):
    """Ground-truth world (x, y) samples of one leg's trace CSV."""
    pts = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            if row.get('x') and row.get('y'):
                pts.append((float(row['x']), float(row['y'])))
    return pts


def load_run_dir(run_dir):
    """[(leg record, trace points or None), ...] for one nav_bench run directory."""
    run_dir = os.path.expanduser(run_dir)
    tag = os.path.basename(os.path.normpath(run_dir))
    path = os.path.join(run_dir, f'{tag}.json')
    if not os.path.exists(path):
        found = glob.glob(os.path.join(run_dir, '*.json'))
        found = [p for p in found if 'legs' in json.load(open(p))]
        if len(found) != 1:
            raise SystemExit(f'{run_dir}: expected one nav_bench JSON, found {found}')
        path = found[0]
        tag = os.path.splitext(os.path.basename(path))[0]
    with open(path) as f:
        doc = json.load(f)
    out = []
    for leg in doc['legs']:
        trace = os.path.join(run_dir, f'{tag}_traces', f"{leg['scenario']}_rep{leg.get('rep', 0)}.csv")
        out.append((leg, load_trace_xy(trace) if os.path.exists(trace) else None))
    return out


def load_c2nav5(path):
    with open(path) as f:
        doc = json.load(f)
    out = []
    for run in doc['runs']:
        if run['tag'].startswith('c2n5_tour_csf65'):
            out.extend((leg, None) for leg in run['legs'])
    return out


# ---------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------

def true_min_clearance(points):
    if not points:
        return None
    return min(GEOM.nearest(x, y)[0][0] for x, y in points)


def _median(values):
    values = [v for v in values if v is not None]
    return statistics.median(values) if values else None


def leg_stats(items):
    legs = [leg for leg, _ in items]
    n = len(legs)
    stop_secs = [(leg.get('cm_polygon_secs') or {}).get('PolygonStop', 0.0)
                 if 'cm_polygon_secs' in leg else None for leg in legs]
    clearances = [true_min_clearance(tr) for _, tr in items if tr]
    yaw = [abs(leg['final_yaw_err_rad']) for leg in legs
           if leg.get('final_yaw_err_rad') is not None]
    return {
        'attempts': n,
        'succeeded': sum(1 for leg in legs if leg.get('status') == 'SUCCEEDED'),
        'median_s': _median([leg.get('duration_sim_s') for leg in legs]),
        'median_goal_err_m': _median([leg.get('final_goal_err_m') for leg in legs]),
        'median_abs_yaw_err_rad': _median(yaw),
        'true_min_clearance_m': min(clearances) if clearances else None,
        'polygon_stop_s': (sum(s for s in stop_secs if s is not None)
                           if any(s is not None for s in stop_secs) else None),
        'polygon_stop_legs': sum(1 for s in stop_secs if s),
        'median_dwb_zero_vx': _median([leg.get('dwb_best_vx_zero_frac') for leg in legs]),
        'median_cmd_below_0_05': _median([leg.get('frac_cmd_below_0.05') for leg in legs]),
    }


def arm_table(items):
    by_leg = {}
    for leg, trace in items:
        by_leg.setdefault(leg['scenario'], []).append((leg, trace))
    rows = {name: leg_stats(by_leg[name]) for name in TOUR_ORDER if name in by_leg}
    rows['TOTAL'] = {
        'attempts': sum(r['attempts'] for r in rows.values()),
        'succeeded': sum(r['succeeded'] for r in rows.values()),
    }
    return rows


def _fmt(value, spec):
    return '--' if value is None else format(value, spec)


def render(arms):
    lines = ['| leg | arm | succeeded | median s | goal err m | abs yaw err rad | '
             'true clear m | PolygonStop s (legs) | DWB zero-vx | cmd<0.05 |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for name in TOUR_ORDER + ['TOTAL']:
        for arm, table in arms.items():
            r = table.get(name)
            if r is None:
                continue
            if name == 'TOTAL':
                lines.append(f"| **TOTAL** | {arm} | **{r['succeeded']}/{r['attempts']}** "
                             '| | | | | | | |')
                continue
            stop = ('--' if r['polygon_stop_s'] is None
                    else f"{r['polygon_stop_s']:.2f} ({r['polygon_stop_legs']})")
            lines.append(
                f"| `{name}` | {arm} | {r['succeeded']}/{r['attempts']} | "
                f"{_fmt(r['median_s'], '.2f')} | {_fmt(r['median_goal_err_m'], '.3f')} | "
                f"{_fmt(r['median_abs_yaw_err_rad'], '.3f')} | "
                f"{_fmt(r['true_min_clearance_m'], '.4f')} | {stop} | "
                f"{_fmt(r['median_dwb_zero_vx'], '.3f')} | {_fmt(r['median_cmd_below_0_05'], '.3f')} |")
    return '\n'.join(lines)


# ---------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------

def mode_report(argv):
    ap = argparse.ArgumentParser(prog='c2nav39_tour_report.py report')
    ap.add_argument('--c2nav5', help='docs/data/c2nav5_bench.json')
    ap.add_argument('--c2nav37', nargs='*', default=[], help='C2-NAV.37 run directories')
    ap.add_argument('--c2nav39', nargs='*', default=[], help='nav_tour_run.sh run directories')
    ap.add_argument('--json', help='write the tables as JSON here')
    args = ap.parse_args(argv)

    arms = {}
    if args.c2nav5:
        arms['C2-NAV.5'] = arm_table(load_c2nav5(args.c2nav5))
    if args.c2nav37:
        arms['C2-NAV.37'] = arm_table([x for d in args.c2nav37 for x in load_run_dir(d)])
    if args.c2nav39:
        arms['C2-NAV.39'] = arm_table([x for d in args.c2nav39 for x in load_run_dir(d)])
    if not arms:
        raise SystemExit('nothing to report: pass --c2nav5, --c2nav37 and/or --c2nav39')
    print(render(arms))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(arms, f, indent=1)
        print(f'\nwrote {args.json}')
    return 0


def mode_selftest():
    n = fail = 0

    def chk(name, cond):
        nonlocal n, fail
        n += 1
        if not cond:
            fail += 1
            print(f'  FAILED: {name}')

    # wall_west's east face is x = -3.9 (C2-NAV.7); a point at x = -3.5 on the
    # centre line is 0.4 m from it and farther from everything else.
    chk('true clearance uses world-file box faces',
        abs(true_min_clearance([(-3.5, 0.0)]) - 0.4) < 1e-12)
    chk('true clearance is the minimum over the track',
        abs(true_min_clearance([(-3.5, 0.0), (-3.7, 0.0)]) - 0.2) < 1e-12)
    chk('no trace means no clearance, never zero', true_min_clearance(None) is None)

    legs = [
        ({'scenario': 'open_space', 'status': 'SUCCEEDED', 'duration_sim_s': 10.0,
          'final_goal_err_m': 0.1, 'final_yaw_err_rad': -0.2,
          'cm_polygon_secs': {'PolygonSlow': 1.0}, 'dwb_best_vx_zero_frac': 0.1,
          'frac_cmd_below_0.05': 0.2}, [(-3.5, 0.0)]),
        ({'scenario': 'open_space', 'status': 'TIMEOUT', 'duration_sim_s': 75.0,
          'final_goal_err_m': 0.5, 'final_yaw_err_rad': 0.4,
          'cm_polygon_secs': {'PolygonStop': 3.5}, 'dwb_best_vx_zero_frac': 0.3,
          'frac_cmd_below_0.05': 0.6}, [(-3.7, 0.0)]),
        ({'scenario': 'open_space', 'status': 'SUCCEEDED', 'duration_sim_s': 20.0,
          'final_goal_err_m': 0.2, 'cm_polygon_secs': {},
          'dwb_best_vx_zero_frac': 0.2, 'frac_cmd_below_0.05': 0.4}, None),
    ]
    t = arm_table(legs)
    r = t['open_space']
    chk('succeeded counts SUCCEEDED only', r['succeeded'] == 2 and r['attempts'] == 3)
    chk('median duration includes the TIMEOUT attempt', r['median_s'] == 20.0)
    chk('median |yaw err| skips legs without the field',
        abs(r['median_abs_yaw_err_rad'] - 0.3) < 1e-12)
    chk('PolygonStop seconds and firing legs',
        r['polygon_stop_s'] == 3.5 and r['polygon_stop_legs'] == 1)
    chk('true clearance is the worst leg', abs(r['true_min_clearance_m'] - 0.2) < 1e-12)
    chk('totals', t['TOTAL'] == {'attempts': 3, 'succeeded': 2})
    chk('render has a TOTAL row', '**2/3**' in render({'arm': t}))

    c2n5 = os.path.join(HERE, 'c2nav5_bench.json')
    if os.path.exists(c2n5):
        t5 = arm_table(load_c2nav5(c2n5))
        chk(f"C2-NAV.5 CSF 65 total reproduces the committed 18/21 (got {t5['TOTAL']})",
            t5['TOTAL'] == {'attempts': 21, 'succeeded': 18})
        chk(f"C2-NAV.5 open_space median reproduces 14.89 s (got {t5['open_space']['median_s']})",
            t5['open_space']['median_s'] == 14.89)
        chk('C2-NAV.5 enclosure_exit reproduces 1/3',
            t5['enclosure_exit']['succeeded'] == 1)
    else:
        chk('docs/data/c2nav5_bench.json present', False)

    print(f'\n{n - fail} passed, {fail} FAILED')
    return 1 if fail else 0


if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] not in ('selftest', 'report'):
        sys.exit(f'usage: {sys.argv[0]} {{selftest|report}} [args...]')
    sys.exit(mode_selftest() if sys.argv[1] == 'selftest' else mode_report(sys.argv[2:]))
