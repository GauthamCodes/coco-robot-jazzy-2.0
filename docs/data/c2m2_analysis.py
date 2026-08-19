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

"""C2-M2.1 analysis: failure clustering and the estimator/control tables.

Read-only over ``docs/data/c2m2_benchmark.json``. Adds nothing to the
experiment; it only reads what the frozen runner already recorded::

    python3 docs/data/c2m2_analysis.py
"""

import json
import math
import os
import sys

import numpy as np


HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, 'c2m2_benchmark.json')


def rows(d, k, r):
    return d['results'].get(f'{k}/{r}', {}).get('rows', [])


def summ(d, k, r):
    return d['results'].get(f'{k}/{r}', {}).get('summary', {})


def _f(v, d=float('nan')):
    if v is None:
        return d
    try:
        f = float(v)
    except (TypeError, ValueError):
        return d
    return d if math.isnan(f) else f


def episode_accounting(d):
    routes, kinds = d['config']['routes'], d['config']['controllers']
    print('=' * 74)
    print('EPISODE ACCOUNTING')
    print('=' * 74)
    total = 0
    errs = 0
    for k in kinds:
        for r in routes:
            n = len(rows(d, k, r))
            total += n
            e = sum(1 for x in rows(d, k, r)
                    if str(x.get('outcome', '')).startswith('error:'))
            errs += e
            if n != len(d['config']['seeds']):
                print(f'  !! {k}/{r}: {n} episodes, expected '
                      f'{len(d["config"]["seeds"])}')
    intended = len(kinds) * len(routes) * len(d['config']['seeds'])
    print(f'  intended  {intended}')
    print(f'  completed {total}')
    print(f'  runner errors (outcome=error:*): {errs}')
    print(f'  ACCOUNTED: {"YES" if total == intended else "NO"}')


def control_table(d):
    routes, kinds = d['config']['routes'], d['config']['controllers']
    print()
    print('=' * 74)
    print('CONTROL  --  120 seeds per cell')
    print('=' * 74)
    print(f'{"route":>6} {"ctrl":>4} {"ascent%":>8} {"succ%":>7} '
          f'{"asc|clm%":>9} {"xtrk m":>8} {"xtrkMax":>8} {"time s":>7} '
          f'{"fallbk":>7}')
    for r in routes:
        for k in kinds:
            s = summ(d, k, r)
            if not s:
                continue
            fb = s.get('fallback_rate')
            fbs = f'{fb:7.3f}' if fb is not None and not math.isnan(
                _f(fb)) else '      --'
            t = _f(s.get('time_s'))
            ts = f'{t:7.1f}' if not math.isnan(t) else '     --'
            print(f'{r:>6} {k:>4} {s["ascent"]*100:8.1f} '
                  f'{s["success"]*100:7.1f} '
                  f'{_f(s.get("ascent_climbable"))*100:9.1f} '
                  f'{s["xtrack_mean"]:8.4f} {s["xtrack_max"]:8.4f} {ts} {fbs}')
        print()


def estimator_table(d):
    routes = d['config']['routes']
    print('=' * 74)
    print('ESTIMATOR (B3 only)')
    print('  No column here is a friction estimate. C2-M2.0 measured that')
    print('  true mu is NOT identifiable from this robot\'s IMU and')
    print('  encoders anywhere in the Yard\'s envelope.')
    print('=' * 74)
    print(f'{"route":>6} {"gradeMAE":>9} {"gradeMax":>9} {"bias":>8} '
          f'{"conv s":>7} {"tau":>7} {"tau-tan(g)":>11} {"bound%":>7} '
          f'{"invalid%":>9} {"sat%":>6} {"schedGap":>9}')
    for r in routes:
        s = summ(d, 'B3', r)
        if not s or 'grade_mae' not in s:
            continue
        print(f'{r:>6} {math.degrees(_f(s["grade_mae"])):9.3f} '
              f'{math.degrees(_f(s["grade_max"])):9.3f} '
              f'{math.degrees(_f(s["grade_bias"])):8.3f} '
              f'{_f(s.get("grade_conv_s")):7.2f} '
              f'{_f(s.get("tau_mean")):7.4f} '
              f'{_f(s.get("tau_minus_tangrade_bias")):11.4f} '
              f'{_f(s.get("mu_bound_held"))*100:7.1f} '
              f'{_f(s.get("invalid_rate"))*100:9.1f} '
              f'{_f(s.get("saturated_rate"))*100:6.1f} '
              f'{_f(s.get("sched_mu_gap_mae")):9.3f}')
    print()
    print('  gradeMAE/Max/bias: degrees, ramp face only (both axles, one plane)')
    print('  conv s:   seconds to settle inside 2 deg and stay there')
    print('  tau:      traction-demand ratio, dimensionless')
    print('  tau-tan(g): tau minus its equilibrium value. ~0 means tau is')
    print('            pinned by GEOMETRY and carries no information on mu')
    print('  bound%:   samples where mu_lower <= true mu held')
    print('  schedGap: |B3 schedule input - B2 privileged input|, NOT an')
    print('            estimator error -- the privileged-information gap')


def failure_clusters(d):
    routes, kinds = d['config']['routes'], d['config']['controllers']
    print()
    print('=' * 74)
    print('FAILURE CLUSTERS  --  outcome counts per cell')
    print('=' * 74)
    all_modes = []
    for k in kinds:
        for r in routes:
            for x in rows(d, k, r):
                m = x.get('outcome', '?')
                if m not in all_modes:
                    all_modes.append(m)
    order = [m for m in ('completed', 'timed out', 'fell off', 'tipped',
                         'slid back', 'high-centred') if m in all_modes]
    order += [m for m in all_modes if m not in order]
    hdr = ''.join(f'{m[:11]:>12}' for m in order)
    print(f'{"cell":>8}{hdr}')
    for r in routes:
        for k in kinds:
            rr = rows(d, k, r)
            if not rr:
                continue
            c = {m: 0 for m in order}
            for x in rr:
                c[x.get('outcome', '?')] = c.get(x.get('outcome', '?'), 0) + 1
            print(f'{k + "/" + r:>8}' + ''.join(f'{c[m]:12d}' for m in order))
        print()


def b3_vs_b1_seeds(d):
    """Where B3 and B1 actually diverge, seed by seed.

    B3 falls back to B1, so on any episode where it never engages the two
    are the SAME controller and must produce the same result. Divergence
    is therefore a direct count of where the observer changed behaviour.
    """
    routes = d['config']['routes']
    print('=' * 74)
    print('B3 vs B1  --  where the observer changed the outcome')
    print('=' * 74)
    seeds = d['config']['seeds']
    for r in routes:
        # Rows carry no seed field, but `run_many` builds its jobs in seed
        # order and `pool.map` preserves order, so row i IS seed i.
        b1r, b3r = rows(d, 'B1', r), rows(d, 'B3', r)
        n = min(len(b1r), len(b3r), len(seeds))
        b1 = {seeds[i]: b1r[i] for i in range(n)}
        b3 = {seeds[i]: b3r[i] for i in range(n)}
        common = sorted(b1)
        if not common:
            print(f'  route {r}: no paired rows')
            continue
        same = [s for s in common
                if b1[s]['outcome'] == b3[s]['outcome']]
        b3_better = [s for s in common
                     if b3[s]['reached_deck'] and not b1[s]['reached_deck']]
        b3_worse = [s for s in common
                    if b1[s]['reached_deck'] and not b3[s]['reached_deck']]
        fb = [_f(b3[s].get('fallback_rate')) for s in common]
        fb = [v for v in fb if not math.isnan(v)]
        never = sum(1 for v in fb if v >= 0.999)
        print(f'  route {r}: {len(common)} paired seeds; identical outcome on '
              f'{len(same)}')
        print(f'      ascent gained by B3: {len(b3_better)} '
              f'{b3_better[:12]}')
        print(f'      ascent lost   by B3: {len(b3_worse)} {b3_worse[:12]}')
        if fb:
            print(f'      fallback rate mean {np.mean(fb):.3f}; episodes '
                  f'never engaging: {never}/{len(fb)}')
    print()


def decision(d):
    routes = d['config']['routes']
    task = d['config']['decision_task']
    margin = d['config']['decision_margin_pp']
    print('=' * 74)
    print(f'DECISION RULE  --  task "{task}", margin {margin:.0f} percentage')
    print('  points. Fixed in C2-M2.0 BEFORE any result existed. Unchanged.')
    print('=' * 74)
    verdicts = []
    for r in routes:
        b2, b3 = summ(d, 'B2', r), summ(d, 'B3', r)
        if not b2 or not b3:
            continue
        gap = (b2[task] - b3[task]) * 100.0
        rl = gap > margin
        verdicts.append((r, gap, rl))
        print(f'  route {r}: B2 {b2[task]*100:5.1f} %   B3 {b3[task]*100:5.1f} %'
              f'   gap {gap:+6.1f} pp   -> '
              f'{"RL JUSTIFIED" if rl else "observer closes the gap"}')
    n = sum(1 for _, _, v in verdicts if v)
    print()
    print(f'  RL justified on {n} of {len(verdicts)} routes.')
    return verdicts


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DATA
    if not os.path.exists(path):
        print(f'no benchmark data at {path}', file=sys.stderr)
        return 1
    with open(path) as fh:
        d = json.load(fh)
    episode_accounting(d)
    control_table(d)
    estimator_table(d)
    failure_clusters(d)
    b3_vs_b1_seeds(d)
    decision(d)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
