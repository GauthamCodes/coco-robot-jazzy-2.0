#!/usr/bin/env python3
"""C2-NAV.27 -- decompose the wall_adjacent refusal into its two terms.

OFFLINE ONLY.

The planner refuses a start whose ESTIMATED pose is close enough to a
lethal cell. At the wall_adjacent goal the wall's lethal cells sit at
map y = -3.335 and the goal at y = -3.000, so the estimated pose has
0.335 m of room. How far the estimate ends up past the goal is the sum
of two independent things:

    est_past = gt_past + amcl_err_toward_wall

  gt_past   how far past the goal the robot PHYSICALLY stopped
            (positive = past the goal, toward the wall)
  amcl_err  how much further toward the wall AMCL believed it was

The refusal boundary, read off geom.py's sweep, is est_past ~ 0.16 m.
Printing both terms separately answers the brief's question about
whether slowdown_ratio = 1.0 systematically changes final position.
"""
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geom  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, '..'))
RES = os.path.join(ROOT, 'results')
LOGS = os.path.join(ROOT, 'logs')

BEGIN = re.compile(r'\[bt_navigator\]: Begin navigating from current '
                   r'location \(([-\d.]+), ([-\d.]+)\) to '
                   r'\(([-\d.]+), ([-\d.]+)\)')
REFUSE = re.compile(r'GridBased plugin failed to plan from '
                    r'\(([-\d.]+), ([-\d.]+)\)')

ARMS = [('BASELINE', ['c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4',
                      'c2n21_bbase_r2', 'c2n21_bbase_r3']),
        ('BASE(excl)', ['c2n21_bbase_r1']),
        ('C2-NAV.25', ['c2n25_slow_r1', 'c2n25_slow_r2', 'c2n25_bslow_r1']),
        ('C2-NAV.26', ['c2n26_slow_r1', 'c2n26_slow_r2', 'c2n26_slow_r3',
                       'c2n26_bslow_r1', 'c2n26_bslow_r2',
                       'c2n26_bslow_r3'])]


def amcl_after_wall_adjacent(tag):
    """AMCL y the planner was given for the FIRST request issued after
    the wall_adjacent leg -- either the next leg's "Begin navigating" or,
    if wall_adjacent itself was refused, the refusal's own start."""
    p = os.path.join(LOGS, 'nav_%s.log' % tag)
    if not os.path.exists(p):
        return None, None
    seen_wa = False
    for ln in open(p, errors='replace'):
        m = BEGIN.search(ln)
        if m:
            if seen_wa:
                return float(m.group(2)), 'next leg begin'
            if abs(float(m.group(3))) < 1e-6 and \
                    abs(float(m.group(4)) + 3.0) < 1e-6:
                seen_wa = True          # this is the wall_adjacent goal
            continue
        m = REFUSE.search(ln)
        if m and seen_wa:
            return float(m.group(2)), 'refusal start'
    return None, None


def main():
    # where the refusal boundary is, measured
    lo = None
    for i in range(0, 400):
        y = -3.0 - i * 0.001
        if geom.verdict(0.0, y)['occupied']:
            lo = -3.0 - y
            break
    print('refusal boundary at the wall_adjacent goal: est_past >= '
          '%.3f m (measured by sweep on the static costmap)' % lo)
    print('wall lethal cell row at map y = -3.335; goal at y = -3.000, '
          'so 0.335 m of room, of which %.3f m may be spent' % lo)
    print()
    print('%-11s%-17s%10s%10s%10s%10s  %s'
          % ('arm', 'run', 'gt_past', 'amcl_err', 'est_past', 'margin',
             'outcome'))
    print('-' * 82)
    for arm, tags in ARMS:
        for tag in tags:
            p = os.path.join(RES, tag + '.json')
            if not os.path.exists(p):
                continue
            legs = {r['scenario']: r for r in json.load(open(p))['legs']}
            wa = legs.get('wall_adjacent')
            if not wa or not wa.get('end_world'):
                continue
            gt_past = -3.0 - wa['end_world'][1]
            ay, src = amcl_after_wall_adjacent(tag)
            if ay is None:
                continue
            est_past = -3.0 - ay
            print('%-11s%-17s%10.3f%10.3f%10.3f%10.3f  %s (%s)'
                  % (arm, tag, gt_past, est_past - gt_past, est_past,
                     lo - est_past,
                     'REFUSED' if est_past >= lo else wa['status'], src))
    print()
    print('gt_past   = ground truth, +ve is PAST the goal toward the wall')
    print('amcl_err  = est_past - gt_past, the localisation term')
    print('margin    = how much further the ESTIMATE could have gone')
    print()
    print('=== the two terms, summarised by arm ===')
    return 0


if __name__ == '__main__':
    sys.exit(main())
