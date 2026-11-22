#!/usr/bin/env python3
"""C2-NAV.27 -- what cost does SmacPlanner2D actually ACCEPT as a start?

OFFLINE ONLY.

The Nav2 .cpp sources are not installed here, so the start-validity
threshold cannot be read from the function body. It CAN be measured.

`nav_bench` recorded, for the enclosure_entry leg of several runs, both
- the LIVE global costmap (243x175 uint8, ~500 snapshots per leg), and
- every plan the planner RETURNED, with its full pose list.

A returned plan's first pose is a start the planner ACCEPTED. Reading
the live costmap at that cell gives an OBSERVED upper bound on the
accepted cost. Doing it over hundreds of accepted plans pins the
threshold without reading a line of C++.
"""
import bisect
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
RES = os.path.join(ROOT, 'results')
W2M = (2.0, 0.0)


def load(tag):
    mp = os.path.join(
        RES, '%s_costmapwindow_enclosure_entry_rep0_meta.json' % tag)
    np_ = os.path.join(
        RES, '%s_costmapwindow_enclosure_entry_rep0.npz' % tag)
    pp = os.path.join(RES, '%s_planwindow_enclosure_entry_rep0.json' % tag)
    if not (os.path.exists(mp) and os.path.exists(np_)
            and os.path.exists(pp)):
        return None
    meta = json.load(open(mp))
    z = np.load(np_)
    key = list(z.keys())[0]
    grids = z[key]
    plans = json.load(open(pp))
    return meta, grids, plans


def main():
    tags = sorted({os.path.basename(p).split('_costmapwindow')[0]
                   for p in glob.glob(os.path.join(
                       RES, '*_costmapwindow_enclosure_entry_rep0.npz'))})
    print('runs with a recorded LIVE global costmap for enclosure_entry:')
    print('  ' + ', '.join(tags))
    print()

    allcosts = []
    for tag in tags:
        d = load(tag)
        if d is None:
            continue
        meta, grids, plans = d
        snaps = meta['snapshots']
        ts = [s['ts_sim_s'] for s in snaps]
        res = snaps[0]['resolution']
        ox, oy = snaps[0]['origin_x'], snaps[0]['origin_y']
        w, h = snaps[0]['width'], snaps[0]['height']
        if grids.ndim == 2:
            grids = grids.reshape(-1, h, w)
        costs = []
        for pl in plans['snapshots']:
            if not pl.get('poses_world'):
                continue
            sx, sy = pl['poses_world'][0]      # ALREADY map frame? check
            # planwindow poses_world are WORLD; convert to map.
            mx, my = sx + W2M[0], sy + W2M[1]
            i = bisect.bisect_left(ts, pl['ts_sim_s'])
            i = min(max(i, 0), len(ts) - 1)
            cx = int((mx - ox) / res)
            cy = int((my - oy) / res)
            if not (0 <= cx < w and 0 <= cy < h):
                continue
            costs.append(int(grids[i, cy, cx]))
        if costs:
            a = np.array(costs)
            print('%-17s %3d accepted plans   cost of start cell: '
                  'min %3d  median %3d  p95 %3d  MAX %3d'
                  % (tag, len(a), a.min(), int(np.median(a)),
                     int(np.percentile(a, 95)), a.max()))
            allcosts.append(a)
    if allcosts:
        a = np.concatenate(allcosts)
        print()
        print('ALL RUNS: %d accepted plan starts, MAX observed start cost '
              '= %d' % (len(a), a.max()))
        import collections
        c = collections.Counter(a.tolist())
        print('counts at the interesting values: '
              '252=%d  253=%d  254=%d'
              % (c.get(252, 0), c.get(253, 0), c.get(254, 0)))
        print('n at cost >= 253: %d ; n at cost == 254: %d'
              % (int((a >= 253).sum()), int((a == 254).sum())))
        print()
        if (a >= 253).sum() > 0:
            print('==> The planner ACCEPTS starts at cost 253 '
                  '(INSCRIBED_INFLATED_OBSTACLE).')
            print('    So its start test is NOT "cost >= 253". The only')
            print('    remaining threshold is cost == 254 (LETHAL), i.e.')
            print('    the start CELL ITSELF must hold an obstacle.')
        else:
            print('==> No accepted start at cost >= 253; consistent with')
            print('    a "cost >= 253" (inscribed) start test.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
