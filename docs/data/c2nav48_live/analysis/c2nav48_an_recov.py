#!/usr/bin/env python3
"""C2-NAV.48 regression check: did raising the local footprint make Nav2 work
harder? Counts recovery / progress-failure activity per run from mission.log."""
import os
import re
import sys

PATTERNS = {
    'controller_failed_progress': r'controller_failed_progress|Failed to make progress',
    'costmap clears': r'clearing_costmap|Clearing .*costmap|ClearEntireCostmap',
    'spin': r'\bSpin\b|Running Spin',
    'backup': r'\bBackUp\b|Running BackUp',
    'wait': r'Running Wait\b',
    'goal aborted': r'Goal was aborted|aborted the goal|bt_navigator.*ABORT',
    'no valid traj': r'no valid trajector|dwb_no_valid',
    'RECOVERY state': r'-> RECOVERY',
    'invalid path/plan fail': r'Failed to create a plan|No valid path|planner_failed',
}


def show(label, path):
    p = os.path.join(path, 'mission.log')
    if not os.path.exists(p):
        print(f'{label:<24} (no mission.log)')
        return
    txt = open(p, errors='replace').read()
    counts = {k: len(re.findall(v, txt, re.I)) for k, v in PATTERNS.items()}
    print(f'{label:<24}' + ''.join(f'{counts[k]:>8}' for k in PATTERNS))


H = os.path.expanduser('~/coco_nav_runs/c2nav46_m6')
hdr = ' ' * 24 + ''.join(f'{k[:7]:>8}' for k in PATTERNS)
print(hdr)
print('-- C2-NAV.46 (robot_radius 0.20)')
for r in ('r1_blue', 'r2_blue', 'r3_blue'):
    show(f'  {r}', os.path.join(H, r))
for base in ('c2nav48_repro', 'c2nav48_valid'):
    N = os.path.expanduser(f'~/coco_nav_runs/{base}')
    if not os.path.isdir(N):
        continue
    print(f'-- {base} (robot_radius 0.25)')
    for d in sorted(os.listdir(N)):
        show(f'  {d}', os.path.join(N, d))
print()
print('key:', ', '.join(f'{k[:7]}={k}' for k in PATTERNS))
