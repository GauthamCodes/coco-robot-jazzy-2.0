#!/usr/bin/env python3
"""C2-NAV.49: did the C2-NAV.48 deadlock mechanism recur anywhere in the matrix?

C2-NAV.46's one failure was `r2_blue`: a 595.5 s PolygonStop hold on the
RETURN leg, 0.2486 m from `cylinder_obstacle`. C2-NAV.48 attributed it to a
44 mm band that was free to the local planner and fatal to the collision
monitor, and closed the band by raising `local_costmap.robot_radius` to 0.25.

This reads the recorded trace and answers, per run, the four questions
Phase 8 of the C2-NAV.49 brief asks -- PolygonStop activation, its duration,
the closest approach to `cylinder_obstacle`, and how the local costmap
classifies that closest pose.

NOTHING HERE IS A NEW METRIC. `cm_action == '1'` is the collision monitor's
STOP state, exactly as `c2nav41_topology.stop_breach` defines it and
`c2nav39_tour_report.stop_holds` used it before that. The trace is the
runner's own 10 Hz resample. The geometry is read from the world file rather
than restated, so a world edit cannot silently invalidate the answer.

No ROS. Reads only files the runner wrote, plus the shipped world.

    python3 -P c2nav49_clearance.py RUN_DIR [RUN_DIR ...] [--json OUT]
"""
import argparse
import csv
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

# this file is <repo>/docs/data/c2nav49_clearance.py, so the repo root is
# THREE directories up, not two.
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WORLD = os.path.join(REPO, 'gazebo_models', 'worlds', 'coco_world.world')
TRACE_HZ = 10.0                     # c2nav42_cmdpath.TRACE_HZ
STOP_RADIUS = 0.25                  # nav2_params.yaml collision_monitor PolygonStop
LOCAL_ROBOT_RADIUS = 0.25           # nav2_params.yaml local_costmap (C2-NAV.48)
FOOTPRINT_PADDING = 0.01            # Nav2 Costmap2DROS default
# Costmap2DROS builds a 16-gon of circumradius robot_radius, pads it, and
# LayeredCostmap takes the apothem. C2-NAV.0 measured 0.205879 m for 0.20.
INSCRIBED = (LOCAL_ROBOT_RADIUS + FOOTPRINT_PADDING) * math.cos(math.pi / 16)
# The leg the C2-NAV.46 deadlock happened on.
RETURN_STATES = ('DESCEND', 'RETURN_HOME', 'PLACE', 'VERIFY_PLACEMENT')


def cylinder_from_world(path=WORLD):
    """(x, y, radius) of cylinder_obstacle, read from the shipped world.

    The world is NOT well-formed XML: its prose comments contain `--`
    (`--randomize`, `--target`), which XML forbids inside a comment, so
    ElementTree refuses the raw file. gz parses it happily. Comments are
    stripped before parsing rather than the world being "fixed" -- the world
    is frozen (`world_v1`), and this reader must not be a reason to edit it.
    C2-NAV.48 hit the same thing and kept a hand-cleaned copy in a scratch
    directory, which is not reproducible; this is.
    """
    with open(path, errors='replace') as f:
        text = re.sub(r'<!--.*?-->', '', f.read(), flags=re.S)
    root = ET.fromstring(text)
    for model in root.iter('model'):
        if model.get('name') != 'cylinder_obstacle':
            continue
        pose = [float(v) for v in (model.findtext('pose') or '0 0 0 0 0 0').split()]
        rad = model.find('.//collision/geometry/cylinder/radius')
        return pose[0], pose[1], float(rad.text)
    raise SystemExit(f'cylinder_obstacle not found in {path}')


def _f(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return None


def episodes(flags):
    """Number of contiguous True runs, and the longest, in rows."""
    count = longest = cur = 0
    for flag in flags:
        if flag:
            cur += 1
            if cur == 1:
                count += 1
            longest = max(longest, cur)
        else:
            cur = 0
    return count, longest


def one_run(run_dir, cyl):
    cx, cy, cr = cyl
    name = os.path.basename(run_dir.rstrip('/'))
    trace = os.path.join(run_dir, 'cmdpath', 'trace.csv')
    out = {'run': name, 'trace': os.path.exists(trace)}
    meta = os.path.join(run_dir, 'meta.txt')
    if os.path.exists(meta):
        text = open(meta, errors='replace').read()
        hit = re.search(r'^colour=(\w+)', text, re.M)
        out['colour'] = hit.group(1) if hit else None
    if not out['trace']:
        return out

    with open(trace, newline='') as f:
        rows = list(csv.DictReader(f))
    out['rows'] = len(rows)

    stop_flags, near_all, near_return = [], [], []
    stop_driven = 0
    stop_by_state = {}
    for row in rows:
        stop = row.get('cm_action') == '1'
        stop_flags.append(stop)
        if stop:
            state = row.get('mission_state') or ''
            stop_by_state[state] = stop_by_state.get(state, 0) + 1
            wheel = _f(row, 'v_wheel')
            if wheel is not None and abs(wheel) > 0.01:
                stop_driven += 1
        x, y = _f(row, 'x'), _f(row, 'y')
        if x is None or y is None:
            continue
        # distance from base_footprint to the cylinder's SURFACE
        surf = math.hypot(x - cx, y - cy) - cr
        near_all.append(surf)
        if (row.get('mission_state') or '') in RETURN_STATES:
            near_return.append(surf)

    count, longest = episodes(stop_flags)
    out.update({
        'stop_rows': sum(stop_flags),
        'stop_seconds': round(sum(stop_flags) / TRACE_HZ, 1),
        'stop_episodes': count,
        'longest_stop_seconds': round(longest / TRACE_HZ, 1),
        'stop_rows_wheels_driven': stop_driven,
        'stop_rows_by_state': stop_by_state,
    })
    if near_all:
        closest = min(near_all)
        out['closest_cylinder_m'] = round(closest, 4)
        # How the LOCAL COSTMAP classifies that pose. Below the inscribed
        # radius the cell is INSCRIBED_INFLATED_OBSTACLE (cost 253) and
        # BaseObstacleCritic::isValidCost rejects it, so the planner will not
        # steer there -- which is the whole point of C2-NAV.48.
        out['closest_inside_stop_circle'] = bool(closest < STOP_RADIUS)
        out['closest_inside_inscribed'] = bool(closest < INSCRIBED)
        out['closest_margin_past_stop_mm'] = round((closest - STOP_RADIUS) * 1000, 1)
    if near_return:
        out['closest_cylinder_return_m'] = round(min(near_return), 4)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--json')
    args = ap.parse_args()

    cyl = cylinder_from_world()
    print(f'cylinder_obstacle from {os.path.relpath(WORLD, REPO)}: '
          f'centre ({cyl[0]}, {cyl[1]}) radius {cyl[2]} m')
    print(f'PolygonStop radius {STOP_RADIUS} m; local inscribed radius '
          f'{INSCRIBED:.6f} m (robot_radius {LOCAL_ROBOT_RADIUS} '
          f'+ padding {FOOTPRINT_PADDING}, 16-gon apothem)\n')

    rows = [one_run(r, cyl) for r in args.runs]
    head = (f'{"run":<12} {"col":<7} {"stopRows":>8} {"stop_s":>7} {"eps":>4} '
            f'{"longest_s":>9} {"driven":>7} {"nearest_m":>9} {"return_m":>9} '
            f'{"inStop":>7} {"inInscr":>8}')
    print(head)
    print('-' * len(head))
    for r in rows:
        if not r.get('trace'):
            print(f'{r["run"]:<12} {r.get("colour") or "--":<7} '
                  f'{"NO TRACE -- run produced no cmdpath/trace.csv":>8}')
            continue
        def g(k, fmt='{}', dash='--'):
            v = r.get(k)
            return dash if v is None else fmt.format(v)
        print(f'{r["run"]:<12} {r.get("colour") or "--":<7} '
              f'{g("stop_rows"):>8} {g("stop_seconds"):>7} '
              f'{g("stop_episodes"):>4} {g("longest_stop_seconds"):>9} '
              f'{g("stop_rows_wheels_driven"):>7} '
              f'{g("closest_cylinder_m", "{:.4f}"):>9} '
              f'{g("closest_cylinder_return_m", "{:.4f}"):>9} '
              f'{g("closest_inside_stop_circle"):>7} '
              f'{g("closest_inside_inscribed"):>8}')

    traced = [r for r in rows if r.get('trace')]
    print()
    print(f'runs with a trace                     : {len(traced)} of {len(rows)}')
    if not traced:
        # CLAUDE.md: any check whose success condition is "we saw nothing"
        # must first prove it can see something. With no trace read, every
        # count below is vacuous and must not be reported as a clean result.
        print('NOTHING WAS READ -- the counts below are vacuous, not evidence.')
    print(f'runs with any PolygonStop row         : '
          f'{sum(1 for r in traced if r.get("stop_rows"))}')
    print(f'PolygonStop rows with wheels driven   : '
          f'{sum(r.get("stop_rows_wheels_driven") or 0 for r in traced)}')
    nearest = [r['closest_cylinder_m'] for r in traced if 'closest_cylinder_m' in r]
    if nearest:
        print(f'closest approach to cylinder, any run : {min(nearest):.4f} m '
              f'({(min(nearest) - STOP_RADIUS) * 1000:+.1f} mm vs the stop circle)')
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(rows, f, indent=1)
        print(f'\nwrote {args.json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
