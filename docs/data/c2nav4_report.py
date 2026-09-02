#!/usr/bin/env python3
# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""C2-NAV.4 report: the static prediction and the live behaviour, side by
side, in the shape the record needs.

  c2nav4_report.py static  <pred.json> [<pred.json> ...]
  c2nav4_report.py live    <navbench.json> [...]
  c2nav4_report.py timeline <timeline.csv> [...]

Nothing here computes anything new. It reads what
`c2nav4_costfield.py` and `nav_bench.py` wrote and prints it.
"""
import csv
import json
import math
import os
import sys


def static(paths):
    print('{:<14} {:>5} {:>5} {:>5} {:>7} {:>5} {:>5} {:>5} | '
          '{:>8} {:>8} {:>8} {:>7} {:>8}'.format(
              'capture', 'CSF', 'min', 'p25', 'median', 'p75', 'max', '<=3',
              'win_vx', 'win_wz', 'win_tot', 'BaseOb', 'zero_tot'))
    for p in paths:
        d = json.load(open(p))
        tag = os.path.basename(p).replace('c2n4_pred', '').replace(
            '.json', '')
        head = (f"{tag}  d={d['dist_to_goal']:.3f}m "
                f"hdg={d['heading_error_deg']:+.2f}deg "
                f"DWB_real_vx={d['dwb_chosen']:.4f} "
                f"total={d['dwb_chosen_total']:.2f} "
                f"gen_err={d['generator_error_m']*1e6:.0f}um")
        print(f'-- {head}')
        for csf, r in sorted(d['results'].items(), key=lambda kv: float(kv[0])):
            pl = r['plan']
            w = r['winner']
            print('{:<14} {:>5g} {:>5} {:>5} {:>7} {:>5} {:>5} {:>5} | '
                  '{:>8} {:>8} {:>8} {:>7} {:>8}'.format(
                      '', float(csf), pl['min'], pl['p25'], pl['median'],
                      pl['p75'], pl['max'], pl['n_le_3'],
                      f"{w['vx']:.4f}" if w else 'none',
                      f"{w['wz']:.4f}" if w else 'none',
                      f"{w['total']:.2f}" if w else 'n/a',
                      f"{w['raw'].get('BaseObstacle', float('nan')):.0f}"
                      if w else 'n/a',
                      f"{r['zero_total']:.2f}"
                      if r['zero_total'] is not None else 'n/a'))


def live(paths):
    keys = [('status', ''), ('duration_sim_s', ''), ('rtf', ''),
            ('path_len_m', ''), ('final_goal_err_m', ''),
            ('min_clearance_m', ''), ('med_clearance_m', ''),
            ('min_scan_range_m', ''), ('n_stops', ''),
            ('longest_stop_s', ''), ('v_cmd_med', ''),
            ('dwb_illegal_frac', ''), ('end_world', '')]
    for p in paths:
        d = json.load(open(p))
        for leg in d['legs']:
            print(f"-- {d['tag']}  {leg.get('scenario')}  rep{leg.get('rep')}")
            for k, _ in keys:
                if k in leg:
                    print(f'     {k:<22} {leg[k]}')
            for k in sorted(leg):
                if k.startswith(('transit_', 'terminal_', 'frac_', 'dwb_',
                                 'n_stop', 'stop_', 'best_vx', 'v_cmd')):
                    if k not in dict(keys):
                        print(f'     {k:<22} {leg[k]}')


GOAL_WORLD = (-3.45, 2.95)


def timeline(paths):
    print('{:<22} {:>5} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}'.format(
        'timeline', 'rows', 't_end', 'd_min', 'd_end', 'vx_max',
        'f(vx=0)', 'f(vx>.05)', 'stall_s'))
    for p in paths:
        rows = list(csv.DictReader(open(p)))
        if not rows:
            print(f'{os.path.basename(p):<22} EMPTY')
            continue
        def f(r, k, dv=0.0):
            try:
                return float(r[k])
            except (KeyError, TypeError, ValueError):
                return dv
        d = [f(r, 'dist_goal', float('nan')) for r in rows]
        d = [v for v in d if not math.isnan(v)]
        vx = [f(r, 'best_vx') for r in rows]
        z = [f(r, 'zero_for') for r in rows]
        n = len(rows)
        print('{:<22} {:>5} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}'.format(
            os.path.basename(p)[:22], n,
            f"{f(rows[-1], 't'):.1f}",
            f'{min(d):.3f}' if d else 'n/a',
            f'{d[-1]:.3f}' if d else 'n/a',
            f'{max(vx):.4f}',
            f'{sum(1 for v in vx if abs(v) < 1e-9)/n:.3f}',
            f'{sum(1 for v in vx if v > 0.05)/n:.3f}',
            f'{max(z):.1f}'))


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    mode, paths = sys.argv[1], sys.argv[2:]
    {'static': static, 'live': live, 'timeline': timeline}[mode](paths)
    return 0


if __name__ == '__main__':
    sys.exit(main())
