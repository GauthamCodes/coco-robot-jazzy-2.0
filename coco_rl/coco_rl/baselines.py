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
The three classical baselines the policy has to beat — M7_DESIGN §3.1.

Built **before** any policy exists, and the ordering is the point: build
them afterwards and they get tuned to lose without anyone deciding to do
that.

    B0  open-loop constant throttle
    B1  the shipped ``lateral_hold`` PD, global gains, unchanged
    B2  gain-scheduled PD, retuned per route, with PRIVILEGED access to
        the episode's true grade and true friction

**B2 is the honest strong baseline and it is meant to win where it can.**
It is handed two numbers the policy will never observe. A weak B2 makes
the whole M8 comparison worthless, so it gets a real grid search over a
tuning seed set and is then evaluated on a disjoint one — tuning and
reporting on the same seeds would measure memorisation, which is the
mistake this phase exists to avoid making about the *policy*.

The reference path, and why the baselines get one
-------------------------------------------------
A lane-hold controller with no lane is not a baseline, it is a straight
line. All three baselines are therefore given the same reference: the
route centreline up the ramp, a converging segment across the staging
deck, and the bridge centreline after that. B0 ignores it — that is what
makes it B0 — but B1 and B2 regulate against it, and the cross-track
number reported for every baseline is measured against it, so the three
columns are comparable.

This is a deliberate choice and it favours the baselines: the policy will
have to discover the same convergence from reward alone.
"""

import math

from coco_rl.lateral import (HEADING_GAIN, LATERAL_CLAMP, LATERAL_GAIN,
                             lateral_hold)


def reference_y(x, route_y, params):
    """Lane centreline at longitudinal position `x`, in world y.

    Route centreline on the ramp; a straight convergence across the
    staging and washboard sections; the bridge centreline from the
    transition onward. The convergence has to finish before the bridge
    because the bridge is 0.65 m wide and there is no room to correct on
    it.
    """
    deck_x0 = params['deck']['x'][0]
    converge_end = params['deck']['sections']['transition']['x'][0]
    bridge_y = params['bridge']['y_centre']
    if x <= deck_x0:
        return route_y
    if x >= converge_end:
        return bridge_y
    frac = (x - deck_x0) / (converge_end - deck_x0)
    return route_y + (bridge_y - route_y) * frac


def _heading(obs):
    return math.atan2(float(obs[2]), float(obs[3]))


class B0:
    """Open-loop constant throttle. No feedback of any kind."""

    name = 'B0 open-loop'
    privileged = False

    def __init__(self, throttle=0.5, **_):
        self.throttle = throttle

    def reset(self, sample, route):
        pass

    def __call__(self, obs, x_world, y_world):
        return [self.throttle, 0.0]


class B1:
    """The shipped lane hold, with the shipped global gains, unchanged.

    Imports ``lateral_hold`` from ``coco_rl.lateral`` rather than
    reimplementing it, so this baseline cannot drift away from the
    controller it is supposed to represent. The gains are the module
    constants: LATERAL_GAIN 3.0, HEADING_GAIN 2.5, LATERAL_CLAMP 0.8.
    """

    name = 'B1 shipped PD'
    privileged = False

    def __init__(self, throttle=0.5, params=None, **_):
        self.throttle = throttle
        self.params = params
        self.route_y = 0.0

    def reset(self, sample, route):
        self.route_y = sample.routes[route].y_centre

    def __call__(self, obs, x_world, y_world):
        y_ref = reference_y(x_world, self.route_y, self.params)
        return lateral_hold([self.throttle, 0.0],
                            y_err=y_world - y_ref, yaw=_heading(obs),
                            gain=LATERAL_GAIN, heading_gain=HEADING_GAIN,
                            clamp=LATERAL_CLAMP)


class B2:
    """Gain-scheduled PD with privileged access to grade and friction.

    The schedule is deliberately simple — a per-route base gain set, with
    throttle and lateral gain interpolated on the episode's true friction
    — because a schedule with more knobs than the tuning set has episodes
    fits the tuning set. What it is *not* allowed to be is a lookup keyed
    on the episode seed, which would be memorisation wearing a schedule's
    clothes.

    ``schedule`` is ``{route: dict}`` from the tuner.
    """

    name = 'B2 scheduled PD'
    privileged = True

    def __init__(self, schedule, params=None, **_):
        self.schedule = schedule
        self.params = params
        self.route_y = 0.0
        self.gains = None

    def reset(self, sample, route):
        r = sample.routes[route]
        self.route_y = r.y_centre
        cfg = self.schedule[route]
        # PRIVILEGED: the true friction and grade of this episode.
        lo, hi = self.params['friction']['range']
        t = (r.friction - lo) / (hi - lo)        # 0 at slick, 1 at grippy
        self.gains = dict(
            throttle=cfg['throttle_lo'] + t * (cfg['throttle_hi']
                                               - cfg['throttle_lo']),
            deck_throttle=cfg['deck_throttle'],
            lateral=cfg['lateral_lo'] + t * (cfg['lateral_hi']
                                             - cfg['lateral_lo']),
            heading=cfg['heading'],
            clamp=cfg['clamp'],
        )
        # grade is privileged too: steeper needs more throttle to hold
        # speed, and the schedule is linear in the jitter about nominal
        nominal = self.params['routes'][route]['grade_deg']
        self.gains['throttle'] = min(
            1.0, self.gains['throttle']
            + cfg['grade_k'] * (r.grade_deg - nominal))

    def __call__(self, obs, x_world, y_world):
        y_ref = reference_y(x_world, self.route_y, self.params)
        g = self.gains
        # Slow down on the deck. The deck asks for up to 1.95 m of lateral
        # convergence in 1.80 m of travel before the bridge, and the
        # minimum turn radius is v / MAX_ANG -- 0.40 m at 0.2 m/s but only
        # 0.20 m at 0.1 m/s. Halving speed halves the radius, which is the
        # difference between converging and arriving at the void still
        # offset. A classical controller is entitled to this; it is gain
        # scheduling on a known geometry, not privileged episode state.
        throttle = g['throttle']
        if x_world > self.params['deck']['x'][0]:
            throttle = g['deck_throttle']
        return lateral_hold([throttle, 0.0],
                            y_err=y_world - y_ref, yaw=_heading(obs),
                            gain=g['lateral'], heading_gain=g['heading'],
                            clamp=g['clamp'])


# A starting schedule. Replaced by the tuner's output; kept so the module
# is usable and testable without a tuning run.
DEFAULT_SCHEDULE = {
    k: dict(throttle_lo=0.50, throttle_hi=0.50, deck_throttle=0.30,
            lateral_lo=3.0, lateral_hi=3.0, heading=2.5, clamp=0.8,
            grade_k=0.0)
    for k in ('a', 'b', 'c')
}
