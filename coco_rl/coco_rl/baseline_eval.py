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
from coco_rl.yard_env import CocoYardEnv

import numpy as np

BAY_X = 3.0            # completing the Yard means reaching the target bay
DECK_MARGIN = 0.60     # surface height that counts as "on the deck"


def run_episode(baseline, route, seed, params=None, max_steps=900):
    """One episode. Returns a dict of outcome and metrics."""
    env = CocoYardEnv(route=route, seed=seed, params=params,
                      max_steps=max_steps)
    obs, info = env.reset()
    baseline.reset(env.sample, route)
    route_y = env.sample.routes[route].y_centre

    xtrack = []
    high_water = float(env.data.qpos[0])
    stalled = 0
    reached_deck = False
    outcome = 'timed out'
    steps = 0

    for steps in range(1, max_steps + 1):
        x = float(env.data.qpos[0])
        y = float(env.data.qpos[1])
        action = baseline(obs, x, y)
        obs, _r, terminated, truncated, inf = env.step(action)

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
    return dict(outcome=outcome, completed=(outcome == 'completed'),
                reached_deck=reached_deck, steps=steps,
                time_s=steps * 0.1,
                xtrack_mean=float(np.abs(arr).mean()),
                xtrack_max=float(np.abs(arr).max()),
                friction=env.sample.routes[route].friction,
                grade=env.sample.routes[route].grade_deg,
                camber=env.sample.routes[route].camber_deg,
                payload=env.sample.payload_mass)


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
    from coco_rl.baselines import B0, B1, B2
    kind, cfg, route, seed, params = args
    cls = {'B0': B0, 'B1': B1, 'B2': B2}[kind]
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
