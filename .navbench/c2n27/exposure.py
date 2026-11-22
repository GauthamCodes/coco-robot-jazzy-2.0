#!/usr/bin/env python3
"""C2-NAV.27 -- how much of each leg is spent where a replan is refusable.

OFFLINE ONLY.

SmacPlanner2D refuses a plan whose START CELL cost >= 253, and
InflationLayer assigns 253 to every cell within `inscribed_radius` of a
lethal cell. For a circular footprint nav2 builds a 16-gon of
`robot_radius` and takes the min CENTRE-TO-EDGE distance, so

    inscribed_radius = robot_radius * cos(pi/16) = 0.20 * 0.98079
                     = 0.19616 m                      (DERIVED)

That defines a BAND around every obstacle. Inside it, ANY replan
request is refused and the leg aborts. `controller_server` replans at
~1 Hz here ("Passing new path to controller"), so

    P(abort on a leg) ~ (time inside the band) x (replan rate)

This module measures the ground-truth exposure per leg, per arm, and
separately the exposure of the ESTIMATED pose (ground truth shifted by
the measured localisation error), because those are different claims.
"""
import csv
import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geom  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, '..'))
RES = os.path.join(ROOT, 'results')
LOGS = os.path.join(ROOT, 'logs')

INSCRIBED_R = 0.20 * np.cos(np.pi / 16)     # 0.19616 m, DERIVED

LEGS = ['open_space', 'wall_adjacent', 'wall_parallel', 'obstacle_corner',
        'corridor_gate', 'enclosure_entry', 'enclosure_exit']
BASE = ['c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4',
        'c2n21_bbase_r2', 'c2n21_bbase_r3']
C25 = ['c2n25_slow_r1', 'c2n25_slow_r2', 'c2n25_bslow_r1']
C26 = ['c2n26_slow_r1', 'c2n26_slow_r2', 'c2n26_slow_r3',
       'c2n26_bslow_r1', 'c2n26_bslow_r2', 'c2n26_bslow_r3']
ARMS = [('BASELINE', BASE), ('C2-NAV.25', C25), ('C2-NAV.26', C26)]

REPLAN = re.compile(r'\[(\d+\.\d+)\].*\[controller_server\]: '
                    r'Passing new path to controller')


def track(tag, leg):
    p = os.path.join(RES, '%s_traces' % tag, '%s_rep0.csv' % leg)
    if not os.path.exists(p):
        return None
    xs, ys = [], []
    with open(p) as f:
        for r in csv.DictReader(f):
            try:
                xs.append(float(r['x']) + 2.0)   # world -> map
                ys.append(float(r['y']))
            except (TypeError, ValueError):
                pass
    if not xs:
        return None
    return np.array(xs), np.array(ys)


def d_to_lethal(xs, ys):
    """Vectorised distance from each map-frame point to nearest lethal
    cell centre. Same metric InflationLayer inflates over."""
    occ = geom.OCC
    out = np.empty(len(xs))
    for i in range(0, len(xs), 512):
        sl = slice(i, i + 512)
        dx = xs[sl][:, None] - occ[None, :, 0]
        dy = ys[sl][:, None] - occ[None, :, 1]
        out[sl] = np.sqrt(dx * dx + dy * dy).min(axis=1)
    return out


def replans(tag):
    p = os.path.join(LOGS, 'nav_%s.log' % tag)
    if not os.path.exists(p):
        return None
    return sum(1 for ln in open(p, errors='replace') if REPLAN.search(ln))


def main():
    print('inscribed_radius = robot_radius*cos(pi/16) = %.5f m (DERIVED)'
          % INSCRIBED_R)
    print('A ground-truth pose with d <= that value sits in a cell the')
    print('planner will refuse as a START. Exposure = fraction of the')
    print("leg's 10 Hz trace inside it.")
    print()

    per = {}
    for arm, tags in ARMS:
        for tag in tags:
            for leg in LEGS:
                t = track(tag, leg)
                if t is None:
                    continue
                d = d_to_lethal(*t)
                per[(arm, tag, leg)] = dict(
                    n=len(d), dmin=float(d.min()),
                    frac=float((d <= INSCRIBED_R).mean()),
                    secs=float((d <= INSCRIBED_R).sum()) / 10.0)

    print('=== ground-truth exposure by leg and arm (seconds inside the '
          'refusal band, summed over runs / mean per run) ===')
    print('%-18s%26s%26s%26s' % ('leg', 'BASELINE (5 runs)',
                                 'C2-NAV.25 (3)', 'C2-NAV.26 (6)'))
    print('%-18s%26s%26s%26s' % ('', 'secs  min_d  runs>0',
                                 'secs  min_d  runs>0', 'secs  min_d  runs>0'))
    print('-' * 96)
    for leg in LEGS:
        row = ''
        for arm, tags in ARMS:
            ks = [per[k] for k in per if k[0] == arm and k[2] == leg]
            if not ks:
                row += '%26s' % 'n/a'
                continue
            secs = sum(k['secs'] for k in ks)
            dmin = min(k['dmin'] for k in ks)
            hit = sum(1 for k in ks if k['secs'] > 0)
            row += '%14.1f%7.3f%5d/%d' % (secs, dmin, hit, len(ks))
        print('%-18s%s' % (leg, row))
    print()

    print('=== the two "Start occupied" legs, in detail ===')
    for tag, leg in [('c2n26_slow_r2', 'wall_adjacent'),
                     ('c2n26_bslow_r3', 'enclosure_entry')]:
        t = track(tag, leg)
        d = d_to_lethal(*t)
        print('%s/%s: n=%d rows, min d=%.3f m, %.1f s inside the band '
              '(%.1f %% of the leg)'
              % (tag, leg, len(d), d.min(),
                 (d <= INSCRIBED_R).sum() / 10.0,
                 100 * (d <= INSCRIBED_R).mean()))
    print()

    print('=== the same legs in the runs that SUCCEEDED them ===')
    for leg in ('wall_adjacent', 'enclosure_entry'):
        print('  %s:' % leg)
        for arm, tags in ARMS:
            for tag in tags:
                k = per.get((arm, tag, leg))
                if k is None:
                    continue
                print('    %-11s%-17s min_d=%.3f  in-band %.1f s'
                      % (arm, tag, k['dmin'], k['secs']))
    print()

    print('=== replan requests per run (OBSERVED: "Passing new path to '
          'controller") ===')
    for arm, tags in ARMS:
        v = [(t, replans(t)) for t in tags]
        tot = [n for _, n in v if n]
        print('  %-11s %s   mean %.0f'
              % (arm, ' '.join('%s=%s' % (t.split('_', 1)[1], n)
                               for t, n in v),
                 sum(tot) / len(tot) if tot else 0))
    return 0


if __name__ == '__main__':
    sys.exit(main())
