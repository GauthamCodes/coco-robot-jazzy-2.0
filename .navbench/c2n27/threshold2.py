#!/usr/bin/env python3
"""C2-NAV.27 -- the accepted-start cost, DISTINCT plans only.

OFFLINE ONLY.

threshold.py's first pass was wrong and this replaces it. Three runs
(c2n26_slow_r3, c2n21_base_r2, c2n21_bbase_r1) stalled with ONE plan
republished 46-48 times; counting each republish as an accepted plan put
46 copies of a single stalled cell into the sample. Deduplicating on the
plan's own first pose removes that.

The published costmap encodes raw 253 (INSCRIBED) as 99 and raw 254
(LETHAL) as 100, so "did the planner accept a start at 253" is answerable
directly: is there a DISTINCT returned plan whose first pose sits on a 99?
"""
import bisect
import collections
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
RES = os.path.join(ROOT, 'results')


def main():
    tot = collections.Counter()
    per = []
    for pp in sorted(glob.glob(os.path.join(
            RES, '*_planwindow_enclosure_entry_rep0.json'))):
        tag = os.path.basename(pp).split('_planwindow')[0]
        mp = os.path.join(
            RES, '%s_costmapwindow_enclosure_entry_rep0_meta.json' % tag)
        zp = os.path.join(
            RES, '%s_costmapwindow_enclosure_entry_rep0.npz' % tag)
        if not (os.path.exists(mp) and os.path.exists(zp)):
            continue
        meta = json.load(open(mp))
        z = np.load(zp)
        grids = z[list(z.keys())[0]]
        s0 = meta['snapshots'][0]
        h, w, res = s0['height'], s0['width'], s0['resolution']
        ox, oy = s0['origin_x'], s0['origin_y']
        if grids.ndim == 2:
            grids = grids.reshape(-1, h, w)
        cts = [s['ts_sim_s'] for s in meta['snapshots']]

        plans = json.load(open(pp))
        seen, costs = set(), []
        for pl in plans['snapshots']:
            if not pl.get('poses_world'):
                continue
            key = (tuple(pl['poses_world'][0]), pl['n_poses'])
            if key in seen:
                continue
            seen.add(key)
            mx, my = pl['poses_world'][0][0] + 2.0, pl['poses_world'][0][1]
            i = min(max(bisect.bisect_left(cts, pl['ts_sim_s']), 0),
                    len(cts) - 1)
            cx, cy = int((mx - ox) / res), int((my - oy) / res)
            if not (0 <= cx < w and 0 <= cy < h):
                continue
            costs.append(int(grids[i, cy, cx]))
        if costs:
            a = np.array(costs)
            per.append((tag, len(a), a.max(), int((a == 99).sum()),
                        int((a == 100).sum())))
            tot.update(a.tolist())

    print('=== DISTINCT returned plans, cost of the plan\'s first pose in '
          'the LIVE published costmap ===')
    print('(published 99 == raw 253 INSCRIBED ; 100 == raw 254 LETHAL)')
    print()
    print('%-18s%9s%9s%9s%9s'
          % ('run', 'distinct', 'max', 'n at 99', 'n at 100'))
    print('-' * 54)
    for r in per:
        print('%-18s%9d%9d%9d%9d' % r)
    print()
    a = np.array(sorted(tot.elements()))
    print('ALL RUNS: %d distinct accepted plan starts' % len(a))
    print('   max published cost = %d' % a.max())
    print('   n at 99 (raw 253, INSCRIBED) = %d' % int((a == 99).sum()))
    print('   n at 100 (raw 254, LETHAL)   = %d' % int((a == 100).sum()))
    print('   p99 = %d, p95 = %d, median = %d'
          % (np.percentile(a, 99), np.percentile(a, 95), np.median(a)))
    print()
    if (a >= 99).sum() == 0:
        print('==> NO distinct plan was ever accepted from a cell at raw')
        print('    253 or above. Every refusal measured sits at exactly')
        print('    raw 253. The start test behaves as cost >= 253')
        print('    (INSCRIBED_INFLATED_OBSTACLE), i.e. the estimated pose')
        print('    within inscribed_radius = 0.196 m of a LETHAL cell.')
    else:
        print('==> Some distinct plan WAS accepted at raw >= 253; the')
        print('    threshold is not a plain cost>=253 test.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
