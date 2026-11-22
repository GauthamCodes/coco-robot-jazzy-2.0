"""Prove the frozen bundle carries what the scratch tree carried.

The bundle is the archival artifact: `.navbench/` is untracked, so a
clean clone has only this file. Check it holds every run, every leg, the
same statuses and the same final errors the live analysis read.
"""
import json
import os

WT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
B = os.path.join(WT, 'docs', 'data', 'c2nav26_robust.json')
R = os.path.join(WT, '.navbench', 'results')

d = json.load(open(B))
print('records:', len(d['records']), sorted(d['records']))
print('traces per run:', {k: len(v) for k, v in sorted(d['traces'].items())})

bad = 0
for tag, rec in sorted(d['records'].items()):
    live = json.load(open(os.path.join(R, tag + '.json')))
    for a, b in zip(live['legs'], rec['legs']):
        if (a['status'] != b['status']
                or a['final_goal_err_m'] != b['final_goal_err_m']):
            print(f'  MISMATCH {tag}/{a["scenario"]}')
            bad += 1
rows = sum(len(v['rows']) for t in d['traces'].values() for v in t.values())
print(f'trace rows in bundle: {rows}')
print('bundle matches the scratch tree' if not bad
      else f'{bad} MISMATCHES')
