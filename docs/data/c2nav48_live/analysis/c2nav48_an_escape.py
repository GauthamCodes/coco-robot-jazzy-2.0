#!/usr/bin/env python3
"""C2-NAV.48: prove the deadlock vetoed ESCAPE commands, not merely forward
ones. During r2_blue's PolygonStop hold, what did Nav2 command and what
reached the wheels?"""
import collections
import csv
import os

p = os.path.expanduser('~/coco_nav_runs/c2nav46_m6/r2_blue/cmdpath/trace.csv')
with open(p) as f:
    rows = list(csv.DictReader(f))

held = [r for r in rows if r['cm_polygon'] == 'PolygonStop']
print(f'PolygonStop rows: {len(held)}  ({len(held)/10:.1f} s at 10 Hz)\n')


def num(r, k):
    try:
        return float(r[k])
    except (TypeError, ValueError):
        return None


buckets = collections.Counter()
wheel_nonzero = 0
for r in held:
    vn, wn = num(r, 'v_nav'), num(r, 'w_nav')
    vg, vw = num(r, 'v_gated'), num(r, 'v_wheel')
    ww = num(r, 'w_wheel')
    if vn is None:
        continue
    if vn > 1e-6:
        buckets['nav commanded FORWARD (v>0)'] += 1
    elif vn < -1e-6:
        buckets['nav commanded REVERSE (v<0)  <-- escape'] += 1
    elif wn is not None and abs(wn) > 1e-6:
        buckets['nav commanded PURE ROTATION (w!=0) <-- escape'] += 1
    else:
        buckets['nav commanded zero'] += 1
    if (vw is not None and abs(vw) > 1e-6) or (ww is not None and abs(ww) > 1e-6):
        wheel_nonzero += 1

for k in sorted(buckets, key=lambda k: -buckets[k]):
    print(f'  {k:<48} {buckets[k]:>5} rows')
print()
print(f'  rows where ANY wheel command was non-zero        {wheel_nonzero:>5}')
print()
nz = [r for r in held if (num(r, 'v_nav') or 0) != 0 or (num(r, 'w_nav') or 0) != 0]
print(f'Nav2 asked for motion in {len(nz)} of {len(held)} held rows; '
      f'the wheels moved in {wheel_nonzero}.')
print()
print('distinct (v_nav, w_nav) the monitor zeroed, most common first:')
cmds = collections.Counter((r['v_nav'], r['w_nav']) for r in held)
for (v, w), n in cmds.most_common(8):
    print(f'  v_nav={v:>8}  w_nav={w:>8}   {n:>5} rows')
