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

"""
The C2-M2.1 terrain benchmark. **Configuration frozen in C2-M2.0.**

Written before any of it was run, and that is the point: the seeds, the
routes, the repetitions, the metrics and the task the decision rule is
evaluated on are all fixed here, in C2-M2.0, so that C2-M2.1 spends its
time measuring rather than deciding what to measure. Changing any of it
after seeing a result is the failure this file exists to prevent.

Run it::

    python3 -m coco_rl.terrain_benchmark --out docs/data/c2m2_benchmark.json

and then, for a quick look without re-running::

    python3 -m coco_rl.terrain_benchmark --report docs/data/c2m2_benchmark.json


Controllers
===========
====  ===========================  ===========================
B0    open-loop constant throttle  no feedback at all
B1    the shipped PD               **the fixed-gain baseline**
B2    gain-scheduled PD            **PRIVILEGED**: true grade, true mu
B3    gain-scheduled PD            **the observer**: estimated both
====  ===========================  ===========================

B0 is carried because M7 Phase 3 measured it and dropping a column
changes what the table means. The comparison the milestone is about is
B1 against B2 against B3.

Terrain
=======
Routes A, B and C of the Yard, exactly as ``yard_params.yaml`` builds
them. No geometry is added, altered or resurrected for this benchmark.

Route B is included **with a caveat that is recorded rather than
hidden**: M7 Phase 3 measured that **39.3 % of its episodes have
mu < tan(grade) and are physically unclimbable**, and that is one of the
three decisions still gating M7 Phase 4. Those episodes are not dropped —
dropping them would be selecting the data — they are FLAGGED per episode
via ``climbable``, and the summary reports success both raw and over the
climbable subset. Route B also happens to be the only route on which
friction is observable at all (see ``ascent`` below), so removing it
would remove the experiment.

Seeds and repetitions
=====================
**Seeds 0-119 per route, 120 episodes, chosen because M7 Phase 3 used
exactly these** — so every number here lands beside the existing
1,080-episode table instead of next to it. They are disjoint from the
tuning seeds 10000-10011 that produced ``TUNED_SCHEDULE``, which is what
keeps B2 honest, and B3 inherits the same schedule so it is not tuned
either.

4 controllers x 3 routes x 120 seeds = **1,440 episodes**. Fixed here.

The task the decision rule is evaluated on
==========================================
**ASCENT — reaching the deck.** Stated before any result exists, because
the ROADMAP's rule ("expand RL only if the observer-driven controller
stays more than 10 percentage points below the privileged controller on a
measured task") is only meaningful if the task is named first.

Ascent and not completion, for a measured reason. M7 Phase 3 established
that **B1 reaches the deck 99 % of the time and then falls off the bridge
in 105 of 120 episodes**: completion is dominated by the deck convergence
geometry — 1.95 m of lateral shift in 1.80 m of travel against a 0.40 m
turn radius — which is an unresolved M7 Phase 4 decision and explicitly
NOT a terrain-control problem. A rule evaluated on completion would be
scoring the bridge.

Completion is reported anyway, in the same table, so nothing is hidden by
the choice.

Metrics, frozen
===============
Estimator (B3 only; B0-B2 carry no observer):

    grade_mae, grade_max, grade_bias   rad, on the ramp face only
    grade_conv_s                       s to settle inside 2 deg
    mu_mae, mu_bias                    against the episode's true mu
    mu_bound_held                      bound intact, single-plane samples
    mu_bound_held_all                  bound intact, every sample
    invalid_rate                       fraction of samples withdrawn
    saturated_rate                     fraction at the contact limit

Control (all four):

    ascent          reached the deck                 <- the decision task
    success         completed to the bay
    xtrack_mean     mean |cross-track|, m
    xtrack_max      worst |cross-track|, m
    time_s          seconds, completed episodes only
    fallback_rate   B3 only: fraction of steps on B1's gains
    modes           the failure taxonomy, counted

Grade error is scored **on the ramp face only**. The analytic surface is
discontinuous at the bridge void and a central difference across a
0.650 m drop reports tens of degrees of "grade" that is an artefact of
the edge; ``baseline_eval._on_ramp_face`` bounds it by position rather
than by discarding an outlier afterwards.
"""

import argparse
import json
import math
import sys

from coco_rl.baseline_eval import run_many, summarise
from coco_rl.baselines import TUNED_SCHEDULE

from coco_sim.yard import load_params

import numpy as np


# ── the frozen configuration ─────────────────────────────────────────────
ROUTES = ('a', 'b', 'c')
SEEDS = tuple(range(120))            # M7 Phase 3's evaluation seeds
MAX_STEPS = 900                      # baseline_eval's default, unchanged

CONTROLLERS = (
    ('B0', dict(throttle=0.5)),
    ('B1', dict(throttle=0.5)),
    ('B2', dict(schedule=TUNED_SCHEDULE)),
    ('B3', dict(schedule=TUNED_SCHEDULE)),
)

# The rule, restated where the runner can print it. NOT to be changed.
DECISION_TASK = 'ascent'
DECISION_MARGIN_PP = 10.0


def climbable(row):
    """Was this episode physically climbable at all?

    ``mu >= tan(grade)``. M7 Phase 3 measured that 39.3 % of Route B's
    episodes fail this. Recorded per episode rather than filtered, so the
    summary can report both and neither is a selection.
    """
    g = row.get('grade')
    mu = row.get('friction')
    if g is None or mu is None or math.isnan(g) or math.isnan(mu):
        return False
    return mu >= math.tan(math.radians(g))


def _mean(rows, key):
    vals = [r[key] for r in rows
            if key in r and r[key] is not None
            and not (isinstance(r[key], float) and math.isnan(r[key]))]
    return float(np.mean(vals)) if vals else float('nan')


def summarise_cell(rows):
    """Phase 3's summary, plus the C2-M2 columns."""
    out = dict(summarise(rows))
    ok = [r for r in rows if climbable(r)]
    out['climbable'] = len(ok) / len(rows) if rows else float('nan')
    out['ascent_climbable'] = (
        sum(r['reached_deck'] for r in ok) / len(ok) if ok else float('nan'))
    out['success_climbable'] = (
        sum(r['completed'] for r in ok) / len(ok) if ok else float('nan'))
    for key in ('grade_mae', 'grade_max', 'grade_bias', 'grade_conv_s',
                'mu_mae', 'mu_bias', 'mu_bound_held',
                'mu_bound_held_all', 'invalid_rate',
                'saturated_rate', 'fallback_rate'):
        if any(key in r for r in rows):
            out[key] = _mean(rows, key)
    return out


def run(routes=ROUTES, seeds=SEEDS, workers=8, controllers=CONTROLLERS,
        max_steps=MAX_STEPS):
    params = load_params()
    results = {}
    for kind, cfg in controllers:
        for route in routes:
            rows = run_many(kind, cfg, route, list(seeds), params,
                            workers=workers)
            results[f'{kind}/{route}'] = dict(
                kind=kind, route=route, n=len(rows),
                summary=summarise_cell(rows), rows=rows)
            s = results[f'{kind}/{route}']['summary']
            print(f'{kind} route {route}: ascent {s["ascent"]:.3f} '
                  f'success {s["success"]:.3f} '
                  f'xtrack {s["xtrack_mean"]:.4f}', flush=True)
    return dict(config=dict(routes=list(routes), seeds=list(seeds),
                            max_steps=max_steps,
                            controllers=[c[0] for c in controllers],
                            decision_task=DECISION_TASK,
                            decision_margin_pp=DECISION_MARGIN_PP),
                results=results)


def report(data):
    """The table, and the decision rule applied to it."""
    res = data['results']
    routes = data['config']['routes']
    kinds = data['config']['controllers']

    print('\nCONTROL')
    print(f'{"route":>6} {"ctrl":>4} {"ascent":>7} {"success":>8} '
          f'{"asc|clm":>8} {"xtrack":>8} {"xt max":>8} {"fallback":>9}')
    for route in routes:
        for kind in kinds:
            s = res.get(f'{kind}/{route}', {}).get('summary')
            if not s:
                continue
            fb = s.get('fallback_rate', float('nan'))
            print(f'{route:>6} {kind:>4} {s["ascent"]:7.3f} '
                  f'{s["success"]:8.3f} {s["ascent_climbable"]:8.3f} '
                  f'{s["xtrack_mean"]:8.4f} {s["xtrack_max"]:8.4f} '
                  f'{fb:9.3f}')

    print('\nESTIMATOR (B3)')
    print(f'{"route":>6} {"gradeMAE":>9} {"gradeMax":>9} {"bias":>8} '
          f'{"conv s":>7} {"muMAE":>7} {"muBias":>8} {"bound":>7} '
          f'{"invalid":>8} {"sat":>7}')
    for route in routes:
        s = res.get(f'B3/{route}', {}).get('summary')
        if not s or 'grade_mae' not in s:
            continue
        print(f'{route:>6} {math.degrees(s["grade_mae"]):9.3f} '
              f'{math.degrees(s["grade_max"]):9.3f} '
              f'{math.degrees(s["grade_bias"]):8.3f} '
              f'{s["grade_conv_s"]:7.2f} {s["mu_mae"]:7.3f} '
              f'{s["mu_bias"]:8.3f} {s["mu_bound_held"]:7.3f} '
              f'{s["invalid_rate"]:8.3f} {s["saturated_rate"]:7.3f}')

    task = data['config']['decision_task']
    margin = data['config']['decision_margin_pp']
    print(f'\nDECISION RULE  --  task "{task}", margin {margin:.0f} '
          f'percentage points. Fixed in C2-M2.0, not to be changed.')
    verdicts = []
    for route in routes:
        b2 = res.get(f'B2/{route}', {}).get('summary')
        b3 = res.get(f'B3/{route}', {}).get('summary')
        if not b2 or not b3:
            continue
        gap = (b2[task] - b3[task]) * 100.0
        rl = gap > margin
        verdicts.append(rl)
        print(f'  route {route}: B2 {b2[task] * 100:5.1f} %  '
              f'B3 {b3[task] * 100:5.1f} %  gap {gap:+6.1f} pp  '
              f'-> {"RL justified" if rl else "observer closes the gap"}')
    if verdicts:
        print(f'\n  Overall: RL is justified on {sum(verdicts)} of '
              f'{len(verdicts)} routes.')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--out', default='docs/data/c2m2_benchmark.json')
    ap.add_argument('--report', default=None,
                    help='report an existing JSON instead of running')
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--routes', default=','.join(ROUTES))
    ap.add_argument('--seeds', type=int, default=len(SEEDS),
                    help='HOW MANY of seeds 0..N-1. The frozen value is '
                         '120; a smaller one is a smoke test and must be '
                         'reported as such.')
    args = ap.parse_args(argv)

    if args.report:
        with open(args.report) as fh:
            report(json.load(fh))
        return 0

    if args.seeds != len(SEEDS):
        print(f'WARNING: running {args.seeds} seeds, not the frozen '
              f'{len(SEEDS)}. This is a smoke test, not the benchmark.',
              file=sys.stderr)
    data = run(routes=tuple(args.routes.split(',')),
               seeds=tuple(range(args.seeds)), workers=args.workers)
    with open(args.out, 'w') as fh:
        json.dump(data, fh, indent=1, default=float)
    print(f'\nwrote {args.out}')
    report(data)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
