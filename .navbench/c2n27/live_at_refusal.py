#!/usr/bin/env python3
"""C2-NAV.27 -- the LIVE global costmap at the moment of the refusals.

OFFLINE ONLY.

The recorded grids are nav2's PUBLISHED costmap, i.e. the rescaled
nav_msgs/OccupancyGrid, not raw costs. Costmap2DPublisher's translation
table is fixed:

    raw 0   -> 0        raw 253 (INSCRIBED) -> 99
    raw 255 -> -1       raw 254 (LETHAL)    -> 100
    raw 1..252          -> 1 + 97*(raw-1)/251     (so 1..98)

DERIVED from the table's construction; CONFIRMED here by the fact that
the enclosure snapshots contain exactly the values {-1, 0..98, 99, 100}
and that 99/100 form contiguous bands around the mapped walls.

c2n26_slow_r2 aborted `wall_adjacent` on "Start occupied" and then sat
motionless at ground truth (-1.952, -3.025) while five further goals were
refused. Its `enclosure_entry` costmap window was recorded DURING that
motionless period, so those snapshots are the very costmap the planner
refused on.
"""
import bisect
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geom  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, '..'))
RES = os.path.join(ROOT, 'results')

INSCRIBED_PUB = 99
LETHAL_PUB = 100


def load(tag):
    meta = json.load(open(os.path.join(
        RES, '%s_costmapwindow_enclosure_entry_rep0_meta.json' % tag)))
    z = np.load(os.path.join(
        RES, '%s_costmapwindow_enclosure_entry_rep0.npz' % tag))
    grids = z[list(z.keys())[0]]
    s0 = meta['snapshots'][0]
    h, w = s0['height'], s0['width']
    if grids.ndim == 2:
        grids = grids.reshape(-1, h, w)
    return meta, grids, s0


def cell(s0, mx, my):
    return (int((my - s0['origin_y']) / s0['resolution']),
            int((mx - s0['origin_x']) / s0['resolution']))


def main():
    for tag, poses in [
        ('c2n26_slow_r2',
         [('AMCL pose the planner refused', 0.02, -3.19),
          ('AMCL pose, first refusal', 0.01, -3.18),
          ('GROUND TRUTH resting pose', 0.048, -3.025),
          ('the goal itself', 0.00, -3.00)]),
        ('c2n26_bslow_r3',
         [('AMCL pose the planner refused', -1.12, 2.02),
          ('GROUND TRUTH end pose', -1.167, 1.897)])]:
        meta, grids, s0 = load(tag)
        print('=' * 78)
        print('%s : %d live snapshots, sim t %.1f -> %.1f s'
              % (tag, len(grids), meta['t0_sim_s'], meta['t1_sim_s']))
        g = grids[0]
        nl = int((g == LETHAL_PUB).sum())
        ni = int((g == INSCRIBED_PUB).sum())
        print('   snapshot 0: %d LETHAL(100) cells, %d INSCRIBED(99) cells;'
              ' static map has %d occupied cells'
              % (nl, ni, len(geom.OCC)))
        print()
        print('   %-34s%8s%10s%10s%10s'
              % ('pose (map frame)', 'live', 'meaning', 'offline',
                 'd_lethal'))
        for lbl, mx, my in poses:
            r, c = cell(s0, mx, my)
            vals = grids[:, r, c]
            v = geom.verdict(mx, my)
            mean = ('LETHAL' if vals.max() == LETHAL_PUB else
                    'INSCRIBED' if vals.max() == INSCRIBED_PUB else 'free')
            print('   %-34s%5d..%-3d%10s%10d%10.3f'
                  % (lbl + ' (%.3f,%.3f)' % (mx, my),
                     int(vals.min()), int(vals.max()), mean,
                     v['cost'], v['d_cell']))
        print()

    print('=' * 78)
    print('=== does the live map add LETHAL cells the static map lacks? ===')
    meta, grids, s0 = load('c2n26_slow_r2')
    g = grids[0]
    res, ox, oy = s0['resolution'], s0['origin_x'], s0['origin_y']
    rows, cols = np.nonzero(g == LETHAL_PUB)
    live_xy = np.column_stack([ox + (cols + 0.5) * res,
                               oy + (rows + 0.5) * res])
    d = np.empty(len(live_xy))
    for i in range(0, len(live_xy), 2048):
        sl = slice(i, i + 2048)
        dx = live_xy[sl, 0][:, None] - geom.OCC[None, :, 0]
        dy = live_xy[sl, 1][:, None] - geom.OCC[None, :, 1]
        d[sl] = np.sqrt(dx * dx + dy * dy).min(axis=1)
    extra = live_xy[d > 1e-6]
    print('live LETHAL cells: %d ; static occupied cells: %d ; live '
          'lethal cells NOT in the static map: %d'
          % (len(live_xy), len(geom.OCC), len(extra)))
    if len(extra):
        near = extra[np.hypot(extra[:, 0] - 0.02,
                              extra[:, 1] + 3.19) < 0.60]
        print('   of those, %d lie within 0.60 m of the refused start '
              '(0.02, -3.19):' % len(near))
        for p in near[:20]:
            print('      map(%.3f, %.3f)  d_from_refused_start=%.3f'
                  % (p[0], p[1], float(np.hypot(p[0] - 0.02, p[1] + 3.19))))
    print()

    print('=== live cost profile down through the wall_adjacent goal, '
          'in c2n26_slow_r2\'s OWN costmap ===')
    print('   %8s%8s%12s%12s' % ('map_y', 'live', 'raw(est)', 'offline_raw'))
    for my in np.arange(-2.90, -3.45, -0.05):
        r, c = cell(s0, 0.02, float(my))
        v = int(grids[:, r, c].max())
        raw = (254 if v == 100 else 253 if v == 99 else
               -1 if v < 0 else int(1 + (v - 1) * 251 / 97) if v else 0)
        print('   %8.3f%8d%12d%12d'
              % (my, v, raw, geom.verdict(0.02, float(my))['cost']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
