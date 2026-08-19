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
Run a baseline over the Yard and classify how it failed.

The failure taxonomy is M7_DESIGN §3.1's, and each label is decided by a
measurement rather than by which terminator happened to fire first:

    tipped        roll or pitch past the 0.6 rad terminator
    fell off      left the deck sideways into the bridge void
    slid back     ended DOWNHILL of its own high-water mark by >0.15 m
    high-centred  stopped moving with the belly within 2 mm of terrain
    timed out     still upright, still on course, out of steps

`slid back` and `high-centred` both look like "timed out" if you only
read the terminator, and they are the two that distinguish a friction
failure from a geometry failure — which is the whole question Route B and
Route C are asking.
"""

import math

from coco_config.robot import CHASSIS_GROUND_CLEARANCE, WHEEL_RADIUS

from coco_rl.baselines import reference_y
from coco_rl.sensor_model import deployable_signals, ground_truth
from coco_rl.yard_env import CocoYardEnv

import numpy as np

BAY_X = 3.0            # completing the Yard means reaching the target bay
DECK_MARGIN = 0.60     # surface height that counts as "on the deck"


def run_episode(baseline, route, seed, params=None, max_steps=900):
    """One episode. Returns a dict of outcome and metrics."""
    env = CocoYardEnv(route=route, seed=seed, params=params,
                      max_steps=max_steps)
    obs, info = env.reset()
    # An observer-driven baseline gets the robot's level-ground attitude,
    # which is a robot constant and not episode state. Nothing else about
    # the episode reaches it. See B3's docstring for the boundary.
    if hasattr(baseline, 'calibrate'):
        baseline.calibrate(env.flat_reference)
    baseline.reset(env.sample, route)
    route_y = env.sample.routes[route].y_centre
    observing = hasattr(baseline, 'observe')

    xtrack = []
    high_water = float(env.data.qpos[0])
    stalled = 0
    reached_deck = False
    outcome = 'timed out'
    steps = 0
    est_rows = []

    for steps in range(1, max_steps + 1):
        x = float(env.data.qpos[0])
        y = float(env.data.qpos[1])
        action = baseline(obs, x, y)
        obs, _r, terminated, truncated, inf = env.step(action)

        if observing:
            # Causal: the controller acted on the estimate standing BEFORE
            # this step, and only now sees the step's five IMU samples.
            baseline.observe([
                deployable_signals(s, cmd_linear=env._v_cmd,
                                   cmd_angular=env._w_cmd)
                for s in env.imu_samples])
            est_rows.append(_score_estimate(env, route, baseline))

        x = float(env.data.qpos[0])
        y = float(env.data.qpos[1])
        z = float(env.data.qpos[2])
        xtrack.append(y - reference_y(x, route_y, params or env.params))
        if z - WHEEL_RADIUS > DECK_MARGIN:
            reached_deck = True
        if x > high_water:
            high_water = x
        # stalled: barely moving forward
        stalled = stalled + 1 if abs(float(env.data.qvel[0])) < 0.02 else 0

        if x > BAY_X and reached_deck:
            outcome = 'completed'
            break
        if terminated:
            outcome = 'fell off' if inf['outcome'] == 'fell' else 'tipped'
            break
        if truncated:
            break

    if outcome == 'timed out':
        x = float(env.data.qpos[0])
        if high_water - x > 0.15:
            outcome = 'slid back'
        elif stalled > 20:
            # belly close to the terrain under the chassis centre?
            gap = _belly_gap(env)
            outcome = 'high-centred' if gap < 0.002 else 'slid back'

    arr = np.array(xtrack) if xtrack else np.array([0.0])
    row = dict(outcome=outcome, completed=(outcome == 'completed'),
               reached_deck=reached_deck, steps=steps,
               time_s=steps * 0.1,
               xtrack_mean=float(np.abs(arr).mean()),
               xtrack_max=float(np.abs(arr).max()),
               friction=env.sample.routes[route].friction,
               grade=env.sample.routes[route].grade_deg,
               camber=env.sample.routes[route].camber_deg,
               payload=env.sample.payload_mass)
    if observing:
        row.update(_estimator_metrics(est_rows))
        row['fallback_rate'] = float(getattr(baseline, 'fallback_rate', 0.0))
    return row


def _score_estimate(env, route, baseline):
    """One (estimate, truth) pair, for the estimator half of the table.

    Ground truth is read HERE, in the scorer, and never handed to the
    controller — that separation is the whole reason ``sensor_model``
    keeps two types instead of one dict with a naming convention.
    """
    truth = ground_truth(env, route)
    est = baseline.last_estimate
    if est is None:
        return None
    on_ramp = _on_ramp_face(env, route)
    return dict(
        t=env.data.time,
        on_ramp=on_ramp,
        grade_err=est.grade - truth.grade,
        grade_true=truth.grade,
        grade_est=est.grade,
        mu_err=est.mu_hat - truth.friction,
        mu_true=truth.friction,
        mu_est=est.mu_hat,
        mu_lower=est.mu_lower,
        bound_held=bool(est.mu_lower <= truth.friction + 1e-9),
        valid=bool(est.valid),
        grade_conf=est.grade_confidence,
        saturated=bool(est.saturated),
        slip_true=truth.slip)


def _on_ramp_face(env, route):
    """Is the robot wholly on the uniform ramp, on ONE plane?

    Two reasons this is bounded, and the second was measured rather than
    anticipated.

    The analytic surface is discontinuous at the bridge void — a central
    difference across a 0.650 m drop returns tens of degrees of "grade"
    that is an artefact and not a slope — so the deck and bridge are
    excluded by position rather than by discarding an outlier afterwards.

    And the margin is **one WHEELBASE**, derived, not chosen. Both axles
    have to be on the same plane before either the grade or the traction
    bound means anything, and the robot straddles the slope break for
    exactly one wheelbase of travel. Measured on Route B seeds 1, 2 and
    4: with a 0.15 m margin the robot was scored as "on the ramp" 0.07 m
    past the foot while its rear axle was still on the flat apron, whose
    friction capacity is not the ramp's. The bound tau <= mu then broke on
    47 % of samples — not because the estimator was wrong (the grade
    estimate read 25.3-26.7 deg against a true 25.1-26.3, at confidence
    0.60-0.97) but because a two-plane contact is outside what a
    single-plane law can say anything about.
    """
    from coco_config.robot import WHEELBASE
    r = env.sample.routes[route]
    x = float(env.data.qpos[0])
    return bool(r.x_foot + WHEELBASE < x < r.x_top - WHEELBASE)


def _estimator_metrics(rows):
    """Aggregate the estimator's own accuracy over one episode."""
    rows = [r for r in rows if r]
    if not rows:
        return {}
    scored = [r for r in rows if r['on_ramp']]
    if not scored:
        # The robot never got both axles onto one plane -- it stalled at
        # the foot, or tipped before reaching it. The grade error and the
        # traction bound are single-plane claims, so the honest answer
        # here is "not measured on this episode", not the same quantities
        # computed where they mean nothing. `_mean` in terrain_benchmark
        # skips NaN, so these episodes drop out of the estimator columns
        # while still counting in every control column.
        #
        # Substituting the off-plane samples instead is not a neutral
        # fallback: it was measured to pull Route B's bound-held rate to
        # 0.53, entirely from the three seeds of six whose robots never
        # left the ramp foot.
        return dict(
            grade_mae=float('nan'), grade_max=float('nan'),
            grade_bias=float('nan'), grade_conv_s=float('nan'),
            mu_mae=float('nan'), mu_bias=float('nan'),
            mu_bound_held=float('nan'),
            mu_bound_held_all=float(
                np.mean([r['bound_held'] for r in rows])),
            invalid_rate=float(1.0 - np.mean([r['valid'] for r in rows])),
            saturated_rate=float(np.mean([r['saturated'] for r in rows])),
            slip_true_mean=float(
                np.mean([abs(r['slip_true']) for r in rows])),
            n_scored=0)
    g_err = np.array([r['grade_err'] for r in scored])
    # mu error and the bound are scored on the SAME single-plane samples
    # as the grade. Scoring the bound everywhere charged it for the
    # straddle at the ramp foot, where a single-plane law makes no claim
    # -- see _on_ramp_face. The invalid rate stays over ALL samples,
    # because "how often did the observer withdraw?" is a question about
    # the whole episode.
    m_err = np.array([r['mu_err'] for r in scored])
    # Convergence: first time the grade error stays inside 2 deg for the
    # rest of the ramp. 2 deg is the roughness at which body pitch stops
    # representing the surface (measured, Route A 0.03 vs Route C 1.3-2.7).
    conv = float('nan')
    tol = math.radians(2.0)
    for i, r in enumerate(scored):
        if all(abs(s['grade_err']) <= tol for s in scored[i:]):
            conv = scored[i]['t'] - scored[0]['t']
            break
    return dict(
        grade_mae=float(np.abs(g_err).mean()),
        grade_max=float(np.abs(g_err).max()),
        grade_bias=float(g_err.mean()),
        grade_conv_s=conv,
        mu_mae=float(np.abs(m_err).mean()),
        mu_bias=float(m_err.mean()),
        mu_bound_held=float(np.mean([r['bound_held'] for r in scored])),
        mu_bound_held_all=float(np.mean([r['bound_held'] for r in rows])),
        invalid_rate=float(1.0 - np.mean([r['valid'] for r in rows])),
        saturated_rate=float(np.mean([r['saturated'] for r in rows])),
        slip_true_mean=float(np.mean([abs(r['slip_true']) for r in rows])),
        n_scored=len(scored))


def _belly_gap(env):
    """Clearance between the chassis underside and the terrain beneath it.

    Uses the analytic surface rather than a contact query because a belly
    resting ON the terrain generates no contact in the wheels' pairs and
    would otherwise be invisible.
    """
    from coco_sim.yard import height
    x = float(env.data.qpos[0])
    y = float(env.data.qpos[1])
    z = float(env.data.qpos[2])
    surface = height(x, y, env.sample)
    return (z - WHEEL_RADIUS + CHASSIS_GROUND_CLEARANCE) - surface


def summarise(rows):
    """Aggregate a list of run_episode results."""
    n = len(rows)
    if not n:
        return {}
    done = [r for r in rows if r['completed']]
    modes = {}
    for r in rows:
        modes[r['outcome']] = modes.get(r['outcome'], 0) + 1
    return dict(
        n=n,
        success=len(done) / n,
        ascent=sum(r['reached_deck'] for r in rows) / n,
        time_s=(sum(r['time_s'] for r in done) / len(done)) if done else
        float('nan'),
        xtrack_mean=sum(r['xtrack_mean'] for r in rows) / n,
        xtrack_max=max(r['xtrack_max'] for r in rows),
        modes=modes,
    )


def _worker(args):
    from coco_rl.baselines import B0, B1, B2, B3
    kind, cfg, route, seed, params = args
    cls = {'B0': B0, 'B1': B1, 'B2': B2, 'B3': B3}[kind]
    baseline = cls(params=params, **cfg)
    try:
        return run_episode(baseline, route, seed, params=params)
    except Exception as exc:                      # noqa: BLE001
        return dict(outcome=f'error:{type(exc).__name__}', completed=False,
                    reached_deck=False, steps=0, time_s=float('nan'),
                    xtrack_mean=float('nan'), xtrack_max=float('nan'),
                    friction=float('nan'), grade=float('nan'),
                    camber=float('nan'), payload=float('nan'))


def run_many(kind, cfg, route, seeds, params, workers=8):
    """Parallel evaluation. Falls back to serial if workers <= 1."""
    jobs = [(kind, cfg, route, s, params) for s in seeds]
    if workers <= 1:
        return [_worker(j) for j in jobs]
    import multiprocessing as mp
    with mp.Pool(workers) as pool:
        return pool.map(_worker, jobs)
