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
from coco_rl.terrain_observer import TerrainObserver


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


def schedule_gains(cfg, friction, grade_deg, nominal_grade_deg, mu_range):
    """The gain schedule, as one function, shared by B2 and B3.

    Extracted so the observer-driven controller reuses the privileged
    controller's RELATIONSHIP rather than a copy of it — a copy stops
    being the same schedule the first time either is touched, and then
    C2-M2 is measuring two controllers instead of two information sets.
    B2 calls this with the episode's true values, B3 with the observer's
    estimates, and that difference is the entire experiment.

    The body is B2's own, unchanged. ``test_baselines.py`` pins the
    output against the values B2 produced before the extraction.
    """
    lo, hi = mu_range
    t = (friction - lo) / (hi - lo)          # 0 at slick, 1 at grippy
    gains = dict(
        throttle=cfg['throttle_lo'] + t * (cfg['throttle_hi']
                                           - cfg['throttle_lo']),
        deck_throttle=cfg['deck_throttle'],
        lateral=cfg['lateral_lo'] + t * (cfg['lateral_hi']
                                         - cfg['lateral_lo']),
        heading=cfg['heading'],
        clamp=cfg['clamp'],
    )
    # steeper needs more throttle to hold speed, and the schedule is
    # linear in the jitter about nominal
    gains['throttle'] = min(
        1.0, gains['throttle'] + cfg['grade_k'] * (grade_deg
                                                   - nominal_grade_deg))
    return gains


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
        # PRIVILEGED: the true friction and grade of this episode.
        self.gains = schedule_gains(
            self.schedule[route], friction=r.friction, grade_deg=r.grade_deg,
            nominal_grade_deg=self.params['routes'][route]['grade_deg'],
            mu_range=self.params['friction']['range'])

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


class B3:
    """B2's schedule, driven by the OBSERVER instead of by the truth.

    This is C2-M2's deliverable. It is B2 with two numbers replaced:

        B2   schedule_gains(cfg, r.friction,   r.grade_deg,   ...)
        B3   schedule_gains(cfg, obs.mu_hat,   deg(obs.grade), ...)

    Everything else — the reference path, the lane hold, the deck
    slow-down, the clamp — is B2's, unchanged, because the experiment is
    about the information and not about the controller.

    Three differences from B2 are forced by the fact that an estimate
    arrives over time rather than at reset, and each is deliberate:

    1. **The gains are recomputed every control step.** B2 resolves its
       gains once, from constants it is given up front. B3 cannot: the
       traction bound only tightens as the episode asks more of the
       contact, and the grade changes when the robot reaches the ramp.

    2. **There is a fallback, and it is B1.** When the observer withdraws
       its estimate — stale input, a turn hard enough that skid-steer
       scrub contaminates the longitudinal balance, or body pitch
       scattering too much to represent the surface — B3 reverts to the
       shipped global gains. B1 is the natural conservative default
       because it is exactly the deployable controller that does not
       claim to know the terrain, and it is already a column in the
       benchmark. ``fallback_rate`` is reported per episode.

    3. **It starts in fallback.** At reset the observer has proved
       nothing about the contact, so ``mu_lower`` is 0 and the confidence
       is 0. A real robot is in the same position, and pretending
       otherwise is how an estimator gets credit for information it does
       not have.

    What B3 is NOT deployable with respect to
    -----------------------------------------
    **Pose.** ``__call__`` receives ``x_world``/``y_world``, exactly as
    B0, B1 and B2 do. That is ground truth, and it stays because the
    experiment isolates the TERRAIN information channel: all four
    controllers get identical pose, so the only thing that differs
    between B2 and B3 is grade and friction. Degrading the pose channel
    as well would confound the measurement and break comparability with
    M7 Phase 3's 1,080 episodes. Localisation is C2-M5's milestone, and
    it says so in ``docs/ROADMAP.md``.

    **Route identity.** B3 knows which route it is on, like B1 and B2,
    and reads ``y_centre`` from it. That is fixed design geometry in
    ``yard_params.yaml`` (a 1.95, b 0.0, c -1.70) and is not among the
    randomised quantities; it is the reference path the module docstring
    already hands to every baseline on purpose.
    """

    name = 'B3 observer-scheduled PD'
    privileged = False

    # Engage the terrain-aware gains only above this grade confidence.
    # Derived from the two measured populations rather than chosen: the
    # confidence ramp zeroes at 2.0 deg of pitch scatter and saturates at
    # 0.5 deg, so 0.25 corresponds to ~1.6 deg of scatter — above Route
    # A's measured 0.03 deg and below Route C's 1.3-2.7 deg.
    CONF_MIN = 0.25

    def __init__(self, schedule, params=None, observer=None, **_):
        self.schedule = schedule
        self.params = params
        self.route = None
        self.route_y = 0.0
        self.observer = observer or TerrainObserver(
            mu_range=tuple((params or {}).get('friction', {})
                           .get('range', (0.35, 0.70))))
        self.gains = None
        self.engaged = False
        self.n_steps = 0
        self.n_fallback = 0
        self.last_estimate = None
        self.flat_reference = None

    def reset(self, sample, route):
        self.route = route
        self.route_y = sample.routes[route].y_centre
        self.observer.reset(flat_reference=self.flat_reference)
        self.gains = None
        self.engaged = False
        self.n_steps = 0
        self.n_fallback = 0
        self.last_estimate = None

    def calibrate(self, flat_reference):
        """Install the robot's level-ground attitude, ``(pitch, roll)``.

        A robot constant measured once by ``CocoYardEnv._measure_rest_z``
        on the flat apron, not episode state — the same number for every
        seed, every route and every friction. Left uncalibrated the
        observer still runs, reports ``grade_calibrated = False`` and
        halves its own confidence.
        """
        self.flat_reference = tuple(flat_reference)
        self.observer.grade_est.calibrate_flat(*self.flat_reference)

    # ── the observer's feed ──────────────────────────────────────────────
    def observe(self, signals):
        """Fold in deployable signals. Called once per IMU sample, 50 Hz.

        Separate from ``__call__`` because the IMU runs five times faster
        than the controller, and an accelerometer decimated to the control
        rate is a different sensor — measured: the traction channel's
        acceleration deficit is a transient that a 10 Hz sample misses.
        """
        for sig in (signals if isinstance(signals, (list, tuple))
                    else [signals]):
            self.last_estimate = self.observer.update(sig)
        return self.last_estimate

    def _resolve_gains(self):
        """Terrain-aware gains if the estimate stands, else B1's.

        Engaging needs the grade to be valid and confident AND the
        traction bound to be ESTABLISHED -- tighter than the a-priori
        floor the friction range already guarantees. It deliberately does
        NOT need the current traction sample to be valid: ``mu_lower`` is
        a bound accumulated over the episode, not an instantaneous
        reading, so a momentary rejection (a turn, a wheel off the ground)
        withdraws the sample and not the knowledge.

        The ``established`` test is what stops B3 scheduling on an
        assumption. Without it the controller reads ``mu_hat`` = the floor
        before it has measured anything, and on Route A the floor maps to
        the HIGH-throttle end of the schedule -- measured: B3 charged the
        cambered route at 0.65 where B2 ran 0.528, and finished with 0.744
        m of cross-track. A conservative fallback has to be conservative
        in the controller's terms, not in the estimate's.
        """
        est = self.last_estimate
        good = (est is not None and est.grade_valid and est.mu_established
                and est.grade_confidence >= self.CONF_MIN)
        # Hysteresis, so a marginal estimate does not chatter the lateral
        # gain between 3.0 and 6.0 in the middle of a climb. One constant,
        # one rule: engage at CONF_MIN, hold until half of it.
        if self.engaged and est is not None and est.grade_valid:
            good = (est.mu_established
                    and est.grade_confidence >= self.CONF_MIN / 2.0)
        self.engaged = bool(good)
        if not good:
            self.n_fallback += 1
            return dict(throttle=0.5, deck_throttle=0.5,
                        lateral=LATERAL_GAIN, heading=HEADING_GAIN,
                        clamp=LATERAL_CLAMP)
        return schedule_gains(
            self.schedule[self.route],
            friction=est.mu_hat,
            grade_deg=math.degrees(est.grade),
            nominal_grade_deg=self.params['routes'][self.route]['grade_deg'],
            mu_range=self.params['friction']['range'])

    def __call__(self, obs, x_world, y_world):
        self.n_steps += 1
        g = self.gains = self._resolve_gains()
        y_ref = reference_y(x_world, self.route_y, self.params)
        throttle = g['throttle']
        if x_world > self.params['deck']['x'][0]:
            throttle = g['deck_throttle']
        return lateral_hold([throttle, 0.0],
                            y_err=y_world - y_ref, yaw=_heading(obs),
                            gain=g['lateral'], heading_gain=g['heading'],
                            clamp=g['clamp'])

    @property
    def fallback_rate(self):
        return self.n_fallback / self.n_steps if self.n_steps else 0.0


# The TUNED schedule, from a grid search on seeds 10000-10011 -- disjoint
# from the evaluation seeds 0-119. Numbers in docs/RESULTS.md.
#
# The first grid searched throttle only over {0.45, 0.65} and B2 came out
# WORSE than B0 at full throttle on Route B (0 % against 8 %). That is the
# failure M7_DESIGN 3.1 warns about in as many words -- "a weak B2 makes
# the entire M8 result worthless" -- and it was not a tuning subtlety, it
# was a grid that never tried the throttle a 26 deg chute needs.
# Re-searched over {0.45, 0.65, 0.85, 1.0}: Route A 88 -> 98 %,
# Route B 0 -> 3 %, Route C 7 -> 15 %.
TUNED_SCHEDULE = {
    'a': {
        'throttle_lo': 0.65,
        'throttle_hi': 0.45,
        'deck_throttle': 0.6,
        'heading': 2.5,
        'clamp': 1.6,
        'grade_k': 0.0,
        'lateral_lo': 6.0,
        'lateral_hi': 6.0
    },
    'b': {
        'throttle_lo': 0.45,
        'throttle_hi': 0.65,
        'deck_throttle': 0.6,
        'heading': 2.5,
        'clamp': 0.8,
        'grade_k': 0.0,
        'lateral_lo': 6.0,
        'lateral_hi': 6.0
    },
    'c': {
        'throttle_lo': 0.45,
        'throttle_hi': 0.65,
        'deck_throttle': 0.6,
        'heading': 2.5,
        'clamp': 1.6,
        'grade_k': 0.0,
        'lateral_lo': 6.0,
        'lateral_hi': 6.0
    }
}

# Kept so the module is usable and testable without a tuning run.
DEFAULT_SCHEDULE = {
    k: dict(throttle_lo=0.50, throttle_hi=0.50, deck_throttle=0.30,
            lateral_lo=3.0, lateral_hi=3.0, heading=2.5, clamp=0.8,
            grade_k=0.0)
    for k in ('a', 'b', 'c')
}
