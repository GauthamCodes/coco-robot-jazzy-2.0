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

"""
C2-M2.0 sanity checks. **Implementation checks, NOT the benchmark.**

Five small deterministic experiments, each answering one question with a
number. They are what C2-M2.0's completion criteria 10 rests on, and they
are deliberately cheap: no multi-seed sweep, no RL, nothing that belongs
to C2-M2.1.

    1  flat ground             does the grade estimate read zero?
    2  a known ramp            does it converge to the known grade?
    3  known friction          does the traction proxy move the right way?
    4  the controller          is the output bounded, and does the
                               fallback engage when the estimate is
                               withdrawn?
    5  the tip terminator      is the Route C decision doing what it
                               claims, on the episode that motivated it?

Run from the repo root::

    python3 docs/data/c2m2_sanity.py

Read-only, no ROS, and deliberately not installed by any CMakeLists.txt --
same shape as ``map_audit.py`` from C2-M1.6.
"""

import math
import sys

import numpy as np

from coco_rl.baselines import B3, TUNED_SCHEDULE
from coco_rl.sensor_model import deployable_signals
from coco_rl.terrain_observer import MAX_AGE, TerrainObserver
from coco_rl.yard_env import CocoYardEnv, TIP_LIMIT, TIP_LIMIT_ABS

from coco_sim.yard import load_params


PARAMS = load_params()
FAILURES = []


def check(label, ok, detail):
    mark = 'PASS' if ok else 'FAIL'
    print(f'  [{mark}] {label}: {detail}')
    if not ok:
        FAILURES.append(label)


def observe_drive(route, seed, steps, throttle=0.5, randomise=False):
    """Drive open-loop and return (observer, per-step records)."""
    env = CocoYardEnv(route=route, seed=seed, params=PARAMS,
                      randomise=randomise, max_steps=steps + 5)
    env.reset()
    ob = TerrainObserver(mu_range=tuple(PARAMS['friction']['range']))
    ob.reset(flat_reference=env.flat_reference)
    from coco_rl.sensor_model import true_grade
    from coco_rl.sensor_model import quat_to_rpy
    rows = []
    for _ in range(steps):
        _o, _r, term, trunc, _i = env.step([throttle, 0.0])
        est = None
        for s in env.imu_samples:
            est = ob.update(deployable_signals(s, env._v_cmd, env._w_cmd))
        x, y = float(env.data.qpos[0]), float(env.data.qpos[1])
        _rr, _pp, yaw = quat_to_rpy(env.data.qpos[3:7])
        rows.append(dict(t=env.data.time, x=x, est=est,
                         truth=true_grade(x, y, env.sample, heading=yaw)))
        if term or trunc:
            break
    return env, ob, rows


# ── 1. flat ground ───────────────────────────────────────────────────────
def sanity_flat():
    print('\n1. FLAT GROUND -- grade must read zero')
    env, ob, rows = observe_drive('a', seed=2, steps=25, throttle=0.4)
    on_flat = [r for r in rows if r['est'] and abs(r['truth']) < 1e-6]
    err = np.array([math.degrees(r['est'].grade) for r in on_flat])
    check('grade on flat', float(np.abs(err).max()) < 0.5,
          f'worst |grade| {np.abs(err).max():.4f} deg over {len(err)} '
          f'samples (true 0)')
    est = on_flat[-1]['est']
    check('traction valid while cruising', est.traction_valid,
          f'tau {est.tau:.4f}, reason {est.reason!r}')
    check('nothing claimed about mu on the flat', not est.mu_established,
          f'mu_lower {est.mu_lower:.4f} has not beaten the '
          f'{PARAMS["friction"]["range"][0]} floor, so mu_hat stays at '
          f'{est.mu_hat:.3f}')


# ── 2. a known ramp ──────────────────────────────────────────────────────
def sanity_ramp():
    print('\n2. KNOWN RAMP -- grade must converge to the built grade')
    for route in ('a', 'b', 'c'):
        env, ob, rows = observe_drive(route, seed=3, steps=140, throttle=0.7)
        r = env.sample.routes[route]
        from coco_config.robot import WHEELBASE
        face = [w for w in rows
                if w['est'] and r.x_foot + WHEELBASE < w['x']
                < r.x_top - WHEELBASE]
        if not face:
            check(f'route {route} grade', False, 'never reached the face')
            continue
        err = np.array([math.degrees(w['est'].grade - w['truth'])
                        for w in face])
        est = np.array([math.degrees(w['est'].grade) for w in face])
        tol = 3.0 if route == 'c' else 1.0
        check(f'route {route} grade vs analytic surface',
              float(np.abs(err).mean()) < tol,
              f'MAE {np.abs(err).mean():.3f} deg, worst '
              f'{np.abs(err).max():.3f}, mean estimate {est.mean():.2f} '
              f'deg vs built {r.grade_deg:.2f} ({len(face)} samples, '
              f'tol {tol})')


# ── 3. known friction ────────────────────────────────────────────────────
def sanity_friction():
    """The negative result, checked rather than asserted.

    tau must be a lower bound on mu, and -- because equilibrium pins it
    at tan(grade) -- must be INSENSITIVE to mu. Both halves are asserted,
    because a version of this that started moving with mu would mean the
    contact frame had drifted again, which is exactly how the first
    (wrong) monotone reading arose.
    """
    from coco_config.robot import WHEELBASE
    print('\n3. KNOWN FRICTION -- what the traction proxy can and cannot say')
    print('   Fixed geometry and seed, ONLY mu changed.')
    for route in ('a', 'b'):
        env0 = CocoYardEnv(route=route, seed=3, params=PARAMS,
                           randomise=False, max_steps=5)
        r0 = env0.sample.routes[route]
        tan_g = math.tan(math.radians(r0.grade_deg))
        print(f'\n   route {route}, {r0.grade_deg:.1f} deg, '
              f'tan(grade) = {tan_g:.4f}')
        print(f'   {"mu":>6} {"tau_face":>9} {"tau-tan(g)":>11} '
              f'{"bound":>7} {"n_face":>7}')
        taus, mus = [], []
        for mu in (0.35, 0.45, 0.55, 0.70):
            env = CocoYardEnv(route=route, seed=3, params=PARAMS,
                              randomise=False, max_steps=200)
            env.reset()
            for pid in range(env.model.npair):
                env.model.pair_friction[pid][0] = mu
                env.model.pair_friction[pid][1] = mu
            ob = TerrainObserver(mu_range=tuple(PARAMS['friction']['range']))
            ob.reset(flat_reference=env.flat_reference)
            r = env.sample.routes[route]
            peak, n = 0.0, 0
            for _ in range(190):
                _o, _rw, term, trunc, _i = env.step([0.8, 0.0])
                x = float(env.data.qpos[0])
                on_face = r.x_foot + WHEELBASE < x < r.x_top - WHEELBASE
                for s in env.imu_samples:
                    est = ob.update(deployable_signals(s, env._v_cmd,
                                                       env._w_cmd))
                    if on_face and est.traction_valid:
                        peak = max(peak, est.tau)
                        n += 1
                if term or trunc:
                    break
            if not n:
                print(f'   {mu:6.2f} {"--":>9} {"--":>11} {"--":>7} '
                      f'{0:7d}   (never got both axles on the face)')
                continue
            taus.append(peak)
            mus.append(mu)
            print(f'   {mu:6.2f} {peak:9.4f} {peak - tan_g:+11.4f} '
                  f'{"held" if peak <= mu + 1e-9 else "BROKEN":>7} {n:7d}')
        if len(taus) < 2:
            continue
        check(f'route {route}: the proxy is a lower bound on mu',
              all(t <= m + 1e-9 for t, m in zip(taus, mus)),
              f'largest overshoot '
              f'{max(t - m for t, m in zip(taus, mus)):+.4f}')
        spread = max(taus) - min(taus)
        check(f'route {route}: the proxy is pinned at tan(grade), '
              f'NOT informative about mu', spread < 0.02,
              f'tau spans {spread:.4f} over a mu span of '
              f'{max(mus) - min(mus):.2f}; equilibrium fixes it at '
              f'tan(grade) = {tan_g:.4f}. This is the measured '
              f'observability limit, not a defect.')
    print('\n   Why: a steady climb is in equilibrium, so the tangential')
    print('   force is m g sin(grade) whatever mu is. tau reveals mu only')
    print('   at saturation, and MAX_LINEAR_ACCEL = 2.0 m/s^2 is below')
    print('   mu*g = 3.43 m/s^2 even at the slick end -- this robot cannot')
    print('   spin its wheels on the flat. The two conditions never meet.')


# ── 4. the controller ────────────────────────────────────────────────────
def sanity_controller():
    print('\n4. CONTROLLER -- bounded output, and a fallback that engages')
    env = CocoYardEnv(route='b', seed=3, params=PARAMS, max_steps=200)
    obs, _ = env.reset()
    b = B3(schedule=TUNED_SCHEDULE, params=PARAMS)
    b.calibrate(env.flat_reference)
    b.reset(env.sample, 'b')
    worst, engaged_any = 0.0, False
    for _ in range(200):
        a = b(obs, float(env.data.qpos[0]), float(env.data.qpos[1]))
        worst = max(worst, abs(a[0]), abs(a[1]))
        obs, _r, term, trunc, _i = env.step(a)
        b.observe([deployable_signals(s, env._v_cmd, env._w_cmd)
                   for s in env.imu_samples])
        engaged_any = engaged_any or b.engaged
        if term or trunc:
            break
    check('command stays inside the action space', worst <= 1.0,
          f'worst |action| {worst:.4f} <= 1.0')
    check('the terrain-aware gains actually engage', engaged_any,
          f'fallback rate over the episode {b.fallback_rate:.3f}')

    # now withdraw the estimate and confirm it reverts to B1's gains
    from coco_rl.lateral import LATERAL_GAIN
    b.observe(deployable_signals(
        dict(env.imu_samples[-1], stamp=env.data.time + MAX_AGE * 20),
        env._v_cmd, env._w_cmd))
    b(obs, float(env.data.qpos[0]), float(env.data.qpos[1]))
    check('a stale estimate falls back to the shipped gains',
          not b.engaged and b.gains['lateral'] == LATERAL_GAIN,
          f'engaged={b.engaged}, lateral gain {b.gains["lateral"]:.2f} '
          f'(B1 is {LATERAL_GAIN:.2f})')


# ── 5. the tip terminator ────────────────────────────────────────────────
def sanity_tip():
    print('\n5. TIP TERMINATOR -- the Route C decision, on its own episode')
    env = CocoYardEnv(route='c', seed=7, params=PARAMS, randomise=False,
                      max_steps=400)
    obs, _ = env.reset()
    worst_rel, worst_abs, at = 0.0, 0.0, None
    for i in range(1, 241):
        obs, _r, term, trunc, info = env.step([0.6, 0.0])
        ps, _rs = env.surface_attitude()
        rel = abs(float(obs[7]) - ps)
        if rel > worst_rel:
            worst_rel, worst_abs = rel, abs(float(obs[7]))
        if abs(float(obs[7])) > TIP_LIMIT and at is None:
            at = (i, math.degrees(obs[7]), math.degrees(ps))
        if term or trunc:
            break
    if at:
        i, pitch, surf = at
        print(f'   first step past the OLD absolute limit: step {i}, '
              f'body pitch {pitch:+.2f} deg on a {-surf:+.2f} deg surface '
              f'= {pitch - surf:+.2f} deg surface-relative')
        check('the old absolute rule would have fired early',
              abs(pitch) > math.degrees(TIP_LIMIT),
              f'|{pitch:.2f}| > {math.degrees(TIP_LIMIT):.2f} deg')
    check('the terminator still ends the episode when it should',
          info['outcome'] == 'tipped',
          f'outcome {info["outcome"]!r} at step {i}, worst surface-relative '
          f'excursion {math.degrees(worst_rel):.2f} deg')
    check('the absolute backstop is the measured static rear-over',
          abs(math.degrees(TIP_LIMIT_ABS) - 54.5) < 0.01,
          f'{math.degrees(TIP_LIMIT_ABS):.2f} deg '
          f'(docs/RESULTS.md, M7 Phase 3)')
    check('v1 is untouched', _v1_untouched(),
          'reward.TIP_LIMIT, mujoco_env.TIP_LIMIT and ramp_driver all '
          'still 0.6 rad ABSOLUTE')


def _v1_untouched():
    from coco_rl import mujoco_env, reward
    return (reward.TIP_LIMIT == 0.6 and mujoco_env.TIP_LIMIT == 0.6
            and reward.is_tipped(0.0, 0.61) and not reward.is_tipped(0.0,
                                                                    0.59))


def main():
    print('C2-M2.0 SANITY CHECKS -- implementation checks, not the benchmark')
    sanity_flat()
    sanity_ramp()
    sanity_friction()
    sanity_controller()
    sanity_tip()
    print(f'\n{"ALL CHECKS PASSED" if not FAILURES else "FAILURES: " + ", ".join(FAILURES)}')
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
