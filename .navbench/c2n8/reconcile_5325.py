#!/usr/bin/env python3
"""What did C2-NAV.7's committed '5325 frames' actually count?"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, '..', '..', 'docs', 'data'))

tot_rows = tot_two = 0
for t in ('c2nav7_cand_r1', 'c2nav7_cand_r2', 'c2nav7_cand_r3'):
    d = json.load(open(os.path.join(DATA, f'{t}_stop.json')))
    per = {k: v['n_rows'] for k, v in d['legs'].items()}
    two = sum(v for k, v in per.items()
              if k in ('enclosure_entry', 'enclosure_exit'))
    tot_rows += d['n_rows']
    tot_two += two
    print(f'{t}: n_rows={d["n_rows"]}  per-leg={per}  two-leg={two}')
print(f'TOTAL CSV rows      {tot_rows}')
print(f'TOTAL two-leg rows  {tot_two}')
