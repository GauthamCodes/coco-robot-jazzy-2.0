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


# C2-NAV.40. A STOP hold is a maximal run of consecutive trace rows (0.1 s
# apart) with cm_action == 1 (PolygonStop). C2-NAV.8's deadlock was one hold
# of 269.5 s in which the robot moved 0.8 mm; a hold is called a deadlock
# when it lasts at least DEADLOCK_HOLD_S, the robot moves less than
# DEADLOCK_MOVE_M during it, and the leg did not succeed.
DEADLOCK_HOLD_S = 30.0
DEADLOCK_MOVE_M = 0.05


def load_trace_rows(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def stop_holds(rows):
    """[{t0, secs, x, y, moved_m, ended_leg}, ...], one per PolygonStop hold."""
    holds, cur = [], None
    for i, row in enumerate(rows):
        if row.get('cm_action') == '1':
            if cur is None:
                cur = {'first': i, 'last': i}
            cur['last'] = i
        elif cur is not None:
            holds.append(cur)
            cur = None
    if cur is not None:
        holds.append(cur)
    out = []
    for h in holds:
        seg = rows[h['first']:h['last'] + 1]
        pts = [(float(r['x']), float(r['y'])) for r in seg if r.get('x') and r.get('y')]
        t0, t1 = float(seg[0]['t_rel']), float(seg[-1]['t_rel'])
        out.append({
            't0': t0,
            'secs': round(t1 - t0 + 0.1, 2),
            'x': pts[0][0] if pts else None,
            'y': pts[0][1] if pts else None,
            'moved_m': (max(math.dist(pts[0], p) for p in pts) if pts else None),
            'ended_leg': h['last'] == len(rows) - 1,
        })
    return out


def immobile_from_start_s(rows, move_m=DEADLOCK_MOVE_M):
    """Seconds from the first trace row until the robot has moved move_m."""
    pts = [(float(r['t_rel']), float(r['x']), float(r['y']))
           for r in rows or [] if r.get('x') and r.get('y')]
    if not pts:
        return None
    t0, x0, y0 = pts[0]
    for t, x, y in pts:
        if math.dist((x0, y0), (x, y)) >= move_m:
            return round(t - t0, 2)
    return round(pts[-1][0] - t0 + 0.1, 2)


def inherits_stop_hold(prev, rows):
    """True when the previous leg ENDED inside a PolygonStop hold and this
    leg's trace records no collision-monitor state other than STOP. The
    monitor publishes its state on CHANGE, so a hold that outlives the leg
    boundary leaves the next trace's cm_action column blank -- C2-NAV.40 r03's
    exit held 65.95 s with not one state row."""
    if not prev or not (prev.get('longest_stop_hold') or {}).get('ended_leg'):
        return False
    states = {r.get('cm_action') for r in rows or []} - {'', None}
    return states <= {'1'}


def leg_detail(leg, rows, prev=None):
    """The per-leg fields C2-NAV.40 reports, from the record and its trace.
    `prev` is the previous leg's detail in the same tour, for a hold that
    carries across the leg boundary."""
    pts = [(float(r['x']), float(r['y'])) for r in rows or [] if r.get('x') and r.get('y')]
    holds = stop_holds(rows or [])
    longest = max(holds, key=lambda h: h['secs']) if holds else None
    failed = leg.get('status') != 'SUCCEEDED'
    inherited = inherits_stop_hold(prev, rows)
    still = immobile_from_start_s(rows)
    deadlock = bool(failed and (
        (longest and longest['secs'] >= DEADLOCK_HOLD_S
         and longest['moved_m'] is not None and longest['moved_m'] < DEADLOCK_MOVE_M)
        or (inherited and still is not None and still >= DEADLOCK_HOLD_S)))
    near = None
    if longest and longest['x'] is not None:
        d, box, _ = GEOM.nearest(longest['x'], longest['y'])[0]
        near = {'box': box, 'dist_m': round(d, 4)}
    return {
        'scenario': leg['scenario'],
        'status': leg.get('status'),
        'goal_world': leg.get('goal_world'),
        'duration_sim_s': leg.get('duration_sim_s'),
        'timeout_s': leg.get('timeout_s'),
        'final_goal_err_m': leg.get('final_goal_err_m'),
        'final_yaw_err_rad': leg.get('final_yaw_err_rad'),
        'path_len_m': leg.get('path_len_m'),
        'true_min_clearance_m': (round(true_min_clearance(pts), 4) if pts else None),
        'polygon_stop_s': (leg.get('cm_polygon_secs') or {}).get('PolygonStop', 0.0),
        'stop_activations': len(holds),
        'longest_stop_hold': longest,
        'longest_stop_near': near,
        'inherited_stop_hold': inherited,
        'immobile_from_start_s': still,
        'start_xy': list(pts[0]) if pts else None,
        'stop_deadlock': deadlock,
    }


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
    ap.add_argument('--arm', nargs='+', action='append', default=[], metavar=('LABEL', 'DIR'),
                    help='another arm: a label, then nav_tour_run.sh run directories')
    ap.add_argument('--json', help='write the tables as JSON here')
    args = ap.parse_args(argv)

    arms = {}
    if args.c2nav5:
        arms['C2-NAV.5'] = arm_table(load_c2nav5(args.c2nav5))
    if args.c2nav37:
        arms['C2-NAV.37'] = arm_table([x for d in args.c2nav37 for x in load_run_dir(d)])
    if args.c2nav39:
        arms['C2-NAV.39'] = arm_table([x for d in args.c2nav39 for x in load_run_dir(d)])
    for label, *dirs in args.arm:
        arms[label] = arm_table([x for d in dirs for x in load_run_dir(d)])
    if not arms:
        raise SystemExit('nothing to report: pass --c2nav5, --c2nav37 and/or --c2nav39')
    print(render(arms))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(arms, f, indent=1)
        print(f'\nwrote {args.json}')
    return 0


def mode_legs(argv):
    """Per run, per leg: the C2-NAV.40 fields, STOP holds and deadlocks."""
    ap = argparse.ArgumentParser(prog='c2nav39_tour_report.py legs')
    ap.add_argument('runs', nargs='+', help='nav_tour_run.sh run directories')
    ap.add_argument('--json', help='write the per-leg records as JSON here')
    args = ap.parse_args(argv)
    ordinary = set(TOUR_ORDER) - {'enclosure_entry', 'enclosure_exit'}
    doc = []
    for run_dir in args.runs:
        run_dir = os.path.expanduser(run_dir)
        tag = os.path.basename(os.path.normpath(run_dir))
        with open(os.path.join(run_dir, f'{tag}.json')) as f:
            legs = json.load(f)['legs']
        details = []
        for leg in legs:
            trace = os.path.join(run_dir, f'{tag}_traces',
                                 f"{leg['scenario']}_rep{leg.get('rep', 0)}.csv")
            rows = load_trace_rows(trace) if os.path.exists(trace) else None
            details.append(leg_detail(leg, rows, details[-1] if details else None))
        ok = [d for d in details if d['status'] == 'SUCCEEDED']
        summary = {
            'run': tag,
            'legs': f'{len(ok)}/{len(details)}',
            'ordinary': f"{sum(d['scenario'] in ordinary for d in ok)}/"
                        f"{sum(d['scenario'] in ordinary for d in details)}",
            'enclosure_entry': [d['status'] for d in details
                                if d['scenario'] == 'enclosure_entry'],
            'enclosure_exit': [d['status'] for d in details
                               if d['scenario'] == 'enclosure_exit'],
            'total_sim_s': round(sum(d['duration_sim_s'] or 0.0 for d in details), 2),
            'min_true_clearance_m': min((d['true_min_clearance_m'] for d in details
                                         if d['true_min_clearance_m'] is not None),
                                        default=None),
            'stop_activations': sum(d['stop_activations'] for d in details),
            'polygon_stop_s': round(sum(d['polygon_stop_s'] for d in details), 2),
            'deadlocks': [d['scenario'] for d in details if d['stop_deadlock']],
        }
        doc.append({'summary': summary, 'legs': details})
        print(f"\n## {tag}: {summary['legs']} legs, ordinary {summary['ordinary']}, "
              f"entry {summary['enclosure_entry']}, exit {summary['enclosure_exit']}, "
              f"{summary['total_sim_s']} sim s, min true clearance "
              f"{summary['min_true_clearance_m']} m, {summary['stop_activations']} STOP "
              f"activations / {summary['polygon_stop_s']} s, deadlocks {summary['deadlocks']}")
        print('| leg | status | goal | s | goal err m | yaw err rad | true clear m | '
              'STOP s (n) | longest hold s @ (x, y), moved m, nearest | deadlock |')
        print('|---|---|---|---|---|---|---|---|---|---|')
        for d in details:
            h, near = d['longest_stop_hold'], d['longest_stop_near']
            hold = ('--' if h is None else
                    f"{h['secs']:.1f} @ ({h['x']:.4f}, {h['y']:.4f}), {h['moved_m']:.4f}, "
                    f"{near['box']} {near['dist_m']:.4f}"
                    + (' [held to leg end]' if h['ended_leg'] else ''))
            if d['inherited_stop_hold']:
                hold = (f"INHERITED from previous leg, no release recorded; immobile "
                        f"{d['immobile_from_start_s']} s from start "
                        f"({d['start_xy'][0]:.4f}, {d['start_xy'][1]:.4f})")
            print(f"| `{d['scenario']}` | {d['status']} | {d['goal_world']} | "
                  f"{d['duration_sim_s']} | {d['final_goal_err_m']} | {d['final_yaw_err_rad']} | "
                  f"{d['true_min_clearance_m']} | {d['polygon_stop_s']} ({d['stop_activations']}) | "
                  f"{hold} | {'YES' if d['stop_deadlock'] else 'no'} |")
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(doc, f, indent=1)
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

    def row(t, action, x, y):
        return {'t_rel': str(t), 'cm_action': action, 'x': str(x), 'y': str(y)}

    rows = ([row(0.0, '0', -3.3, 1.9)]
            + [row(round(0.1 + 0.1 * i, 1), '1', -3.3, 1.9 + (0.0008 if i else 0.0))
               for i in range(400)])
    holds = stop_holds(rows)
    chk('one hold, 40 s, held to the end of the leg',
        len(holds) == 1 and holds[0]['secs'] == 40.0 and holds[0]['ended_leg'])
    chk('hold movement is measured', abs(holds[0]['moved_m'] - 0.0008) < 1e-9)
    chk('two separate holds are two activations',
        len(stop_holds([row(0, '1', 0, 0), row(0.1, '2', 0, 0), row(0.2, '1', 0, 0)])) == 2)
    d = leg_detail({'scenario': 'enclosure_entry', 'status': 'TIMEOUT'}, rows)
    chk('an immobile 40 s hold on a failed leg is a deadlock', d['stop_deadlock'])
    chk('the same hold on a SUCCEEDED leg is not',
        not leg_detail({'scenario': 'x', 'status': 'SUCCEEDED'}, rows)['stop_deadlock'])
    chk('nearest geometry named for the hold',
        d['longest_stop_near']['box'] == GEOM.nearest(-3.3, 1.9)[0][1])
    moving = [row(round(0.1 * i, 1), '1', -3.3 + 0.001 * i, 1.9) for i in range(400)]
    chk('a hold the robot moves 0.4 m through is not a deadlock',
        not leg_detail({'scenario': 'x', 'status': 'TIMEOUT'}, moving)['stop_deadlock'])
    chk('no trace: no activations, no deadlock',
        leg_detail({'scenario': 'x', 'status': 'TIMEOUT'}, None)['stop_activations'] == 0)

    # A short hold that ends the entry and carries into an exit whose trace
    # has no monitor state at all (C2-NAV.40 r03): a deadlock on the exit.
    entry_rows = ([row(round(0.1 * i, 1), '2', -2.49, 2.65 + 0.0001 * i) for i in range(100)]
                  + [row(round(10.0 + 0.1 * i, 1), '1', -2.5052, 2.6831) for i in range(90)])
    entry = leg_detail({'scenario': 'enclosure_entry', 'status': 'TIMEOUT'}, entry_rows)
    chk('a 9 s hold alone is not a deadlock', not entry['stop_deadlock'])
    exit_rows = [row(round(0.1 * i, 1), '', -2.5052, 2.6831) for i in range(660)]
    ex = leg_detail({'scenario': 'enclosure_exit', 'status': 'TIMEOUT'}, exit_rows, entry)
    chk('a blank-monitor exit after a held entry inherits the hold', ex['inherited_stop_hold'])
    chk('immobile time is measured from the leg start', ex['immobile_from_start_s'] == 66.0)
    chk('an inherited 66 s immobile hold on a failed leg is a deadlock', ex['stop_deadlock'])
    released = [row(0.0, '0', -2.5052, 2.6831)] + exit_rows[1:]
    chk('a recorded release breaks the inheritance',
        not leg_detail({'scenario': 'enclosure_exit', 'status': 'TIMEOUT'}, released,
                       entry)['inherited_stop_hold'])
    chk('no inheritance when the previous leg did not end held',
        not leg_detail({'scenario': 'x', 'status': 'TIMEOUT'}, exit_rows,
                       leg_detail({'scenario': 'y', 'status': 'TIMEOUT'},
                                  [row(0.0, '0', 0, 0)]))['inherited_stop_hold'])

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
    modes = {'selftest': lambda _: mode_selftest(), 'report': mode_report, 'legs': mode_legs}
    if len(sys.argv) < 2 or sys.argv[1] not in modes:
        sys.exit(f'usage: {sys.argv[0]} {{selftest|report|legs}} [args...]')
    sys.exit(modes[sys.argv[1]](sys.argv[2:]))
