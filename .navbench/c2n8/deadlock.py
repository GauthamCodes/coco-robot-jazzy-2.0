#!/usr/bin/env python3
"""C2-NAV.8 tour r1: the entry deadlock, characterised from the CSV.

Where the robot stopped, for how long, what it was commanding, what
reached the wheels, and how far it was from the geometry that gated it.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, '..', '..', 'docs', 'data'))
sys.path.insert(0, DATA)
import c2nav8_report as R                                   # noqa: E402
from c2nav7_geom import CIRCUMSCRIBED, STOP_RADIUS          # noqa: E402


def med(v):
    v = sorted(v)
    return v[len(v) // 2] if v else None


for tag in ('c2n8_tour_r1', 'c2n8_tour_r2', 'c2n8_tour_r3'):
    rows = R._csv_rows({'tag': tag})
    print(f'=== {tag} ===')
    for leg in ('enclosure_entry', 'enclosure_exit'):
        rs = [r for r in rows if R._row_leg(r) == leg]
        if not rs:
            continue
        st = [r for r in rs if r['monitor_action'] == 'STOP']
        print(f'  {leg}: {len(rs)} frames, {len(st)} STOP')
        if not st:
            continue
        xs = {(round(float(r['gt_x']), 4), round(float(r['gt_y']), 4))
              for r in st}
        t0 = min(float(r['t_s']) for r in st)
        t1 = max(float(r['t_s']) for r in st)
        print(f'    distinct poses while in STOP: {len(xs)}')
        if len(xs) <= 3:
            for x, y in sorted(xs):
                d, who, q = R.nearest_full(x, y)[0]
                print(f'      ({x:+.4f}, {y:+.4f})  {d:.4f} m to {who} '
                      f'at ({q[0]:+.4f}, {q[1]:+.4f})')
                print(f'        inside PolygonStop by '
                      f'{(STOP_RADIUS-d)*1000:.1f} mm; '
                      f'{(d-CIRCUMSCRIBED)*1000:.1f} mm above the '
                      f'circumscribed radius')
        print(f'    held from t={t0:.1f}s to t={t1:.1f}s '
              f'({t1-t0:.1f} s continuous)')
        ins = {int(r['n_in_stop']) for r in st}
        print(f'    returns inside the circle: {sorted(ins)}')
        vn = [float(r['v_nav']) for r in st if r['v_nav'] not in ('', None)]
        vw = [float(r['v_wheel']) for r in st
              if r['v_wheel'] not in ('', None)]
        print(f'    v_nav median {med(vn)}, min {min(vn)}, max {max(vn)}')
        print(f'    v_wheel: {sorted(set(vw))[:5]} (n={len(vw)})')
    print()

print('=== minimum TRUE clearance over the whole experiment ===')
worst = None
for tag in ('c2n8_tour_r1', 'c2n8_tour_r2', 'c2n8_tour_r3'):
    rows = R._csv_rows({'tag': tag})
    for r in rows:
        if not r.get('gt_x'):
            continue
        x, y = float(r['gt_x']), float(r['gt_y'])
        d, who, _ = R.nearest_full(x, y)[0]
        if worst is None or d < worst[0]:
            worst = (d, who, tag, R._row_leg(r), x, y)
d, who, tag, leg, x, y = worst
print(f'  {d:.4f} m to {who}, on {tag} {leg}, at ({x:+.4f}, {y:+.4f})')
print(f'  circumscribed radius {CIRCUMSCRIBED} m -> margin '
      f'{(d-CIRCUMSCRIBED)*1000:.1f} mm')
print(f'  PolygonStop radius {STOP_RADIUS} m -> '
      f'{"inside by" if d < STOP_RADIUS else "clear by"} '
      f'{abs(d-STOP_RADIUS)*1000:.1f} mm')
