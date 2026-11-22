#!/usr/bin/env python3
"""C2-NAV.27 -- are the "accepted plan starts" really starts? (offline)

threshold.py found 946 returned plans whose FIRST pose sat on a cell of
published cost 99 (= raw 253, INSCRIBED). live_at_refusal.py found the
two refusals fired on cells of published cost 99 as well. Both cannot be
true of a plain `cost >= 253` start test, so one of the two readings is
measuring the wrong thing. This checks which.

For c2n26_slow_r3 -- 46 returned plans over a 200 s stalled
enclosure_entry -- it prints, per plan: the plan's own timestamp, its
first pose, the robot's GROUND TRUTH pose at that timestamp, and the
live published cost at both.
"""
import bisect
import csv
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geom  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, '..'))
RES = os.path.join(ROOT, 'results')
TAG = sys.argv[1] if len(sys.argv) > 1 else 'c2n26_slow_r3'
LEG = 'enclosure_entry'


def main():
    meta = json.load(open(os.path.join(
        RES, '%s_costmapwindow_%s_rep0_meta.json' % (TAG, LEG))))
    z = np.load(os.path.join(
        RES, '%s_costmapwindow_%s_rep0.npz' % (TAG, LEG)))
    grids = z[list(z.keys())[0]]
    s0 = meta['snapshots'][0]
    h, w, res = s0['height'], s0['width'], s0['resolution']
    ox, oy = s0['origin_x'], s0['origin_y']
    if grids.ndim == 2:
        grids = grids.reshape(-1, h, w)
    cts = [s['ts_sim_s'] for s in meta['snapshots']]

    plans = json.load(open(os.path.join(
        RES, '%s_planwindow_%s_rep0.json' % (TAG, LEG))))
    t0 = plans['t0_sim_s']

    # ground-truth track, t_rel -> world xy
    tr = []
    p = os.path.join(RES, '%s_traces' % TAG, '%s_rep0.csv' % LEG)
    with open(p) as f:
        for r in csv.DictReader(f):
            try:
                tr.append((float(r['t_rel']), float(r['x']),
                           float(r['y'])))
            except (TypeError, ValueError):
                pass
    trt = [a[0] for a in tr]

    def cost(mx, my, ts):
        i = min(max(bisect.bisect_left(cts, ts), 0), len(cts) - 1)
        cx, cy = int((mx - ox) / res), int((my - oy) / res)
        if not (0 <= cx < w and 0 <= cy < h):
            return None
        return int(grids[i, cy, cx])

    print('%s / %s -- %d returned plans, %d costmap snapshots'
          % (TAG, LEG, len(plans['snapshots']), len(grids)))
    print('leg t0_sim = %.3f s' % t0)
    print()
    print('%8s%9s %-22s%6s   %-22s%6s%9s'
          % ('t_rel', 'n_poses', 'plan first pose (map)', 'cost',
             'ground truth (map)', 'cost', 'gap_m'))
    print('-' * 92)
    for pl in plans['snapshots']:
        if not pl.get('poses_world'):
            continue
        ts = pl['ts_sim_s']
        rel = ts - t0
        sx, sy = pl['poses_world'][0]
        mx, my = sx + 2.0, sy
        j = min(max(bisect.bisect_left(trt, rel), 0), len(tr) - 1)
        gx, gy = tr[j][1] + 2.0, tr[j][2]
        gap = float(np.hypot(mx - gx, my - gy))
        print('%8.1f%9d (%7.3f,%7.3f)      %5s   (%7.3f,%7.3f)      %5s%9.3f'
              % (rel, pl['n_poses'], mx, my, cost(mx, my, ts),
                 gx, gy, cost(gx, gy, ts), gap))
    return 0


if __name__ == '__main__':
    sys.exit(main())
