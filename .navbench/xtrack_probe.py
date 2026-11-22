#!/usr/bin/env python3
"""Is the cross-track difference real, or an artefact of the terminal phase?

xtrack_med_m is the median over SAMPLES of the distance from the driven
pose to the last global plan, and the last plan is a stub near the goal
(plan_len_m_last: 0.05 m baseline, 0.383 m post-change). Samples are
uniform in time, so a leg that parks at the goal for a third of its
duration contributes a third of its samples at distance ~0 and drags the
median down. This measures that directly.
"""
import csv
import glob
import math
import os
import statistics

GOALS = {'open_space': (-2.00, -2.20), 'wall_adjacent': (-2.00, -3.00),
         'wall_parallel': (0.50, -2.95), 'obstacle_corner': (0.30, -0.30),
         'corridor_gate': (-2.60, -0.10), 'enclosure_entry': (-3.45, 2.95),
         'enclosure_exit': (-2.00, 0.00)}

BASE = ('/home/gautham/ros2_ws/src/coco-robot-ros2/.claude/worktrees/'
        'c2nav0-diagnosis/.navbench/results')

for d, tag in ((f'{BASE}/baselineA_traces', 'baseline'),
               (f'{BASE}/navA_goalyaw_traces', 'c2nav1')):
    fracs, meds = [], []
    for p in sorted(glob.glob(os.path.join(d, '*.csv'))):
        b = os.path.basename(p)
        g = next((v for k, v in GOALS.items() if b.startswith(k)), None)
        if g is None:
            continue
        ds = []
        for r in csv.DictReader(open(p)):
            try:
                ds.append(math.dist((float(r['x']), float(r['y'])), g))
            except (ValueError, TypeError):
                pass
        if not ds:
            continue
        fracs.append(sum(1 for v in ds if v <= 0.25) / len(ds))
        meds.append(statistics.median(ds))
    print(f'{tag:>9}: samples parked within 0.25 m of the goal = '
          f'{100 * statistics.median(fracs):5.1f}% of a leg   |   '
          f'median |track-goal| = {statistics.median(meds):.3f} m')
