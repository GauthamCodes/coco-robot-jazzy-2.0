#!/usr/bin/env python3
"""C2-NAV.48 validation: clearance + PolygonStop behaviour in the new runs,
against the three C2-NAV.46 historical blue runs."""
import collections
import csv
import math
import os
import sys

OBST = {
    'cylinder_obstacle': ('cyl', -0.2, 0.6, 0.2),
    'gate_cube_north': ('box', -1.1, 1.05, 0.25, 0.25),
    'gate_cube_south': ('box', -1.1, -0.75, 0.25, 0.25),
    'box_obstacle_2': ('box', 0.8, -1.4, 0.25, 0.25),
    'box_obstacle_1': ('box', -3.0, 2.4, 0.25, 0.25),
    'wall_west': ('plane_x', -3.9),
    'wall_east': ('plane_x', 7.9),
    'wall_north': ('plane_y', 3.9),
    'wall_south': ('plane_y', -3.9),
}
STOP_R = 0.25
INS_OLD = (0.20 + 0.01) * math.cos(math.pi / 16)
INS_NEW = (0.25 + 0.01) * math.cos(math.pi / 16)


def clear_to(name, x, y):
    o = OBST[name]
    if o[0] == 'cyl':
        return math.hypot(x - o[1], y - o[2]) - o[3]
    if o[0] == 'box':
        return math.hypot(max(abs(x - o[1]) - o[3], 0.0),
                          max(abs(y - o[2]) - o[4], 0.0))
    if o[0] == 'plane_x':
        return abs(x - o[1])
    return abs(y - o[1])


def report(label, path):
    tp = os.path.join(path, 'cmdpath', 'trace.csv')
    if not os.path.exists(tp):
        print(f'{label:<28} (no trace)')
        return
    with open(tp) as f:
        rows = list(csv.DictReader(f))
    pol = collections.Counter(r['cm_polygon'] for r in rows)
    stop_rows = pol.get('PolygonStop', 0)
    ret = [r for r in rows if r['mission_state'] == 'RETURN_HOME']
    best = (9.9, '')
    for r in ret:
        try:
            x, y = float(r['x']), float(r['y'])
        except (TypeError, ValueError):
            continue
        for n in OBST:
            c = clear_to(n, x, y)
            if c < best[0]:
                best = (c, n)
    cyl = min((clear_to('cylinder_obstacle', float(r['x']), float(r['y']))
               for r in ret if r['x'] and r['y']), default=float('nan'))
    dur = float(rows[-1]['t_rel'])
    print(f'{label:<28} dur {dur:7.1f}s  stop_rows {stop_rows:>5}  '
          f'slow {pol.get("PolygonSlow", 0):>4}  '
          f'closest {best[0]:.4f} ({best[1]})  cyl {cyl:.4f}  '
          f'{"INSIDE stop" if best[0] < STOP_R else "clear of stop"}')


print(f'PolygonStop circle          {STOP_R:.4f} m')
print(f'inscribed @ robot_radius 0.20 (old) {INS_OLD:.6f} m')
print(f'inscribed @ robot_radius 0.25 (new) {INS_NEW:.6f} m')
print()
print('--- C2-NAV.46 historical (robot_radius 0.20) ---')
H = os.path.expanduser('~/coco_nav_runs/c2nav46_m6')
for r in ('r1_blue', 'r2_blue', 'r3_blue'):
    report(f'  {r}', os.path.join(H, r))
print()
print('--- C2-NAV.48 (robot_radius 0.25) ---')
N = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1
                       else '~/coco_nav_runs/c2nav48_repro')
for d in sorted(os.listdir(N)) if os.path.isdir(N) else []:
    report(f'  {d}', os.path.join(N, d))
