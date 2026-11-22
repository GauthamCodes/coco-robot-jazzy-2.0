#!/usr/bin/env python3
"""C2-NAV.27 -- check the offline costmap reconstruction against a LIVE one.

OFFLINE ONLY.

geom.py rebuilds the global costmap from the static map + the nav2
InflationLayer formula. `nav_bench` recorded the REAL global costmap for
enclosure_entry legs. Comparing them decides whether the offline model
may be trusted for the legs where no live costmap was recorded --
wall_adjacent above all.
"""
import csv
import json
import os
import sys
import bisect

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geom  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, '..'))
RES = os.path.join(ROOT, 'results')
TAG = 'c2n26_slow_r3'          # 46 accepted plans, 200 s deep in the pinch


def main():
    meta = json.load(open(os.path.join(
        RES, '%s_costmapwindow_enclosure_entry_rep0_meta.json' % TAG)))
    z = np.load(os.path.join(
        RES, '%s_costmapwindow_enclosure_entry_rep0.npz' % TAG))
    grids = z[list(z.keys())[0]]
    s0 = meta['snapshots'][0]
    res, ox, oy = s0['resolution'], s0['origin_x'], s0['origin_y']
    w, h = s0['width'], s0['height']
    if grids.ndim == 2:
        grids = grids.reshape(-1, h, w)
    g = grids[0]
    print('live global costmap: %d snapshots, grid %s, res %.4f, '
          'origin (%.3f, %.3f)' % (len(grids), g.shape, res, ox, oy))
    print('value histogram of one live snapshot:')
    vals, cnt = np.unique(g, return_counts=True)
    for v, c in zip(vals.tolist(), cnt.tolist()):
        print('   cost %3d : %6d cells' % (v, c))
    print()

    print('=== live cost profile straight down through the '
          'wall_adjacent goal (map x = 0.00) ===')
    cx = int((0.00 - ox) / res)
    print('%9s%9s%12s%12s' % ('map_y', 'live', 'offline', 'd_to_lethal'))
    for my in np.arange(-2.90, -3.45, -0.05):
        cy = int((my - oy) / res)
        if not (0 <= cy < h):
            continue
        v = geom.verdict(0.00, float(my))
        print('%9.3f%9d%12d%12.3f'
              % (my, int(g[cy, cx]), v['cost'], v['d_cell']))
    print()

    print('=== agreement over the WHOLE grid (live snapshot 0 vs '
          'offline static reconstruction) ===')
    ys = oy + (np.arange(h) + 0.5) * res
    xs = ox + (np.arange(w) + 0.5) * res
    XX, YY = np.meshgrid(xs, ys)
    pts = np.column_stack([XX.ravel(), YY.ravel()])
    d = np.empty(len(pts))
    occ = geom.OCC
    for i in range(0, len(pts), 4096):
        sl = slice(i, i + 4096)
        dx = pts[sl, 0][:, None] - occ[None, :, 0]
        dy = pts[sl, 1][:, None] - occ[None, :, 1]
        d[sl] = np.sqrt(dx * dx + dy * dy).min(axis=1)
    off = np.array([geom.inflation_cost(v) for v in d]).reshape(h, w)
    live = g.astype(int)
    same = (off == live)
    print('exact match on %.2f %% of %d cells' % (100 * same.mean(), off.size))
    diff = off.astype(int) - live
    print('offline - live: mean %.2f, p5 %.0f, p95 %.0f, max |%d|'
          % (diff.mean(), np.percentile(diff, 5), np.percentile(diff, 95),
             np.abs(diff).max()))
    print()
    print('cells where LIVE is inscribed-or-lethal (>=253): %d'
          % int((live >= 253).sum()))
    print('cells where OFFLINE is inscribed-or-lethal (>=253): %d'
          % int((off >= 253).sum()))
    print('live cells >=253 that offline also calls >=253: %d'
          % int(((live >= 253) & (off >= 253)).sum()))
    print()

    print('=== what cost does the live map actually give at the two '
          '"Start occupied" poses? ===')
    for lbl, mx, my in [('c2n26_slow_r2 AMCL (0.02,-3.19)', 0.02, -3.19),
                        ('c2n26_slow_r2 g-truth (0.048,-3.025)',
                         0.048, -3.025),
                        ('c2n26_bslow_r3 AMCL (-1.12, 2.02)', -1.12, 2.02)]:
        cxi, cyi = int((mx - ox) / res), int((my - oy) / res)
        v = geom.verdict(mx, my)
        print('%-40s live=%3d  offline=%3d  d=%.3f'
              % (lbl, int(g[cyi, cxi]), v['cost'], v['d_cell']))
    print()
    print('NOTE: this live snapshot is from a DIFFERENT run/leg, so it')
    print('shares only the static layer. It is used here to validate the')
    print('static reconstruction, not to date-stamp those poses.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
