#!/usr/bin/env python3
"""C2-NAV.27 -- inventory every planner refusal ever logged in .navbench.

OFFLINE ONLY. Reads committed/frozen artifacts. Changes nothing.
"""
import collections
import glob
import os
import re

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
LOGS = os.path.join(ROOT, 'logs')

PAT = re.compile(
    r'\[(\d+\.\d+)\]\s+\[(\w+)\]: GridBased plugin failed to plan from '
    r'\(([-\d.]+), ([-\d.]+)\) to \(([-\d.]+), ([-\d.]+)\): "([^"]+)"')

# Arm membership, transcribed BY VALUE from docs/data/c2nav25_slow.py
# (BASE_A/BASE_B/CAND) and docs/data/c2nav26_robust.py (C26_A/C26_B).
BASE_A = ['c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4']
BASE_B = ['c2n21_bbase_r2', 'c2n21_bbase_r3']
C25 = ['c2n25_slow_r1', 'c2n25_slow_r2', 'c2n25_bslow_r1']
C26 = ['c2n26_slow_r1', 'c2n26_slow_r2', 'c2n26_slow_r3',
       'c2n26_bslow_r1', 'c2n26_bslow_r2', 'c2n26_bslow_r3']


def arm(tag):
    if tag in BASE_A + BASE_B:
        return 'BASELINE frozen'
    if tag in C25:
        return 'C2-NAV.25 cand'
    if tag in C26:
        return 'C2-NAV.26 cand'
    return 'other (unarmed)'


def main():
    rows = []
    for p in sorted(glob.glob(os.path.join(LOGS, 'nav_*.log'))):
        tag = os.path.basename(p)[4:-4]
        txt = open(p, errors='replace').read()
        for m in PAT.finditer(txt):
            rows.append(dict(tag=tag, arm=arm(tag), t=float(m.group(1)),
                             sx=float(m.group(3)), sy=float(m.group(4)),
                             gx=float(m.group(5)), gy=float(m.group(6)),
                             why=m.group(7)))

    print('=== every GridBased planner refusal in .navbench/logs '
          '(OBSERVED) ===')
    print('%-20s%-18s%-18s%-20s%-20s' %
          ('run', 'arm', 'reason', 'start(map)', 'goal(map)'))
    print('-' * 96)
    for r in rows:
        print('%-20s%-18s%-18s(%6.2f,%6.2f)     (%6.2f,%6.2f)' %
              (r['tag'], r['arm'], r['why'], r['sx'], r['sy'],
               r['gx'], r['gy']))

    print()
    print('=== refusals by arm x reason ===')
    c = collections.Counter((r['arm'], r['why']) for r in rows)
    for k, v in sorted(c.items()):
        print('  %-18s%-18s%d' % (k[0], k[1], v))

    print()
    print('=== runs with >=1 "Start occupied" ===')
    runs = collections.Counter(r['tag'] for r in rows
                               if r['why'] == 'Start occupied')
    for k, v in sorted(runs.items()):
        print('  %-20s%-18s%d refusal(s)' % (k, arm(k), v))

    print()
    print('=== log files searched ===')
    print('  %d nav_*.log files' %
          len(glob.glob(os.path.join(LOGS, 'nav_*.log'))))


if __name__ == '__main__':
    main()
