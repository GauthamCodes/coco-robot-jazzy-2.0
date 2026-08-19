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
The terrain observer — C2-M2's deployable estimate of grade and traction.

**Never imports rclpy**, and never will: it is reached from ``baselines``
and therefore from ``yard_env``, and CLAUDE.md §2 makes that structural.
The ROS node that wraps it lives in ``terrain_observer_node``, which
imports *this* and not the other way round.

The whole point of this module
------------------------------
``B2`` is handed the episode's true grade and true friction. A real robot
is handed neither. This module estimates both from signals a real robot
actually has, and ``B3`` in ``baselines`` schedules on the estimates using
``B2``'s own relationship. C2-M2.1 then measures the gap.

The information boundary is a TYPE, not a convention
----------------------------------------------------
:class:`DeployableSignals` is the only thing :class:`TerrainObserver`
accepts. It carries IMU attitude, IMU specific force, wheel speeds and the
commanded twist — and nothing else. World pose, true grade, true friction
and true body velocity live in :class:`GroundTruth` in ``sensor_model``,
which the observer cannot see. A ground-truth leak is therefore a type
error rather than a review miss, which is the lesson C2-M1.5 paid for: a
signal that is *almost always* right (``/ramp/status`` pitch) passes every
test taken on the ramp and is wrong everywhere else.


Grade
=====
Measured, in this session, on Route A's uniform 12.000 deg face:

    body pitch  -12.00 deg      true surface grade  +12.00 deg

so **nose-up is NEGATIVE pitch** under ``quat_to_rp``'s convention, and

    gamma = -(theta - theta_0)                                       (1)

with ``theta`` the low-pass filtered body pitch and ``theta_0`` the
flat-ground reference. Renaming ``body_pitch`` to ``grade`` would have
been wrong in SIGN as well as in reference, which is worth stating
plainly because it is the exact mistake §6 of the C2-M2.0 brief warns
against.

``theta_0`` is not assumed to be zero. It absorbs IMU mounting
misalignment and the standing pitch the compliant contact takes under the
robot's own weight, and it is estimated by averaging ``theta`` while the
robot is quasi-stationary on ground declared flat. Until that has
happened the estimate is published with ``grade_calibrated = False``
rather than silently using 0.0.

**Body pitch is not terrain grade**, and the filter is where that is
handled. On a smooth face they agree to 0.03 deg (measured, Route A,
three samples across the climb). On Route C's rubble they disagree by
1.3-2.7 deg (measured), because the chassis sits on the two contact
patches and not on the surface under its centre. The low-pass time
constant is therefore set from the rubble's own correlation length:
0.12 m at ~0.25 m/s is a 0.48 s feature, so ``tau_grade = 0.5`` s
(derived) suppresses it while still tracking a ramp entry within 0.12 m
of travel. The residual scatter about the filter is reported as
``grade_roughness`` and is what drives the confidence down on rubble --
it measures, live, how badly body pitch is representing the surface.


Traction
========
The honest part of this module, and the part with a negative result in it.

**What was measured.** On Route B (26 deg) at fixed geometry and seed,
sweeping only mu:

    mu     wheel speed   servo lag   body speed   true slip
    0.55     0.3189       0.0185       0.2146      0.327
    0.70     0.3189       0.0185       0.2758      0.135

The wheel speed is IDENTICAL to four decimals, and so is the velocity
servo's lag behind its command. The actuators are velocity servos with
authority to spare, so **the encoders cannot see friction at all** --
wheel-odometry slip is identically zero by construction. The only
observable that separates the two surfaces is body velocity.

**Why body velocity is not used.** An inertial estimate was built and
measured: integrating specific force with gravity removed by the measured
attitude, after a zero-velocity update, at both 10 Hz and the IMU's real
50 Hz. It lost 0.10-0.15 m/s inside the first two seconds against a true
speed of 0.28 m/s, and the estimated slip came out in the WRONG ORDER
between the two surfaces. A world-frame mechanisation would have been
exact -- and exactly circular, because the Yard's IMU is NOISELESS:
``yard_params.yaml`` records ``imu_noise_sigma: not_yet_measured``
because ``coco_robo2.xacro`` declares no ``<noise>`` element. Any
integrator would therefore have scored its own arithmetic rather than the
robot's observability, and would have transferred nothing. **So nothing
here integrates.**

**What is estimated instead.** Coulomb bounds the friction force by
``mu`` times the normal force, both taken at the contact patch, and an
accelerometer measures both components of that force directly. So

    tau = f_t / f_n                                                  (2)

with ``f_t`` and ``f_n`` the tangential and normal specific force in the
CONTACT frame, is the fraction of available normal load currently being
turned into traction. It is a **lower bound on mu**, it equals mu at
saturation, and it involves no integration and therefore no drift. The
running maximum ``mu_lower`` is the tightest bound the episode has
proved. See the code for the two corrections this form went through --
both were wrong in a way that looked right, and both were caught by
measurement rather than by reading.

**The negative result, and it is the substantive finding of C2-M2.0.**

Sweeping only mu on fixed geometry and a fixed seed, and taking the peak
tau over samples where both axles are on one plane:

    route A, 12 deg     mu    0.35     0.45     0.55     0.70
                        tau   0.2131   0.2128   0.2128   0.2127
                        tan(grade) = 0.2126

    route B, 26 deg     mu             0.55     0.70
                        tau            0.4950   0.4874
                        tan(grade) = 0.4877

**tau does not move with mu. It equals tan(grade), to four decimal
places, on every surface the Yard builds.** The earlier version of this
module reported a monotone tau and a 5:1 compression; that was an
artefact of taking the ratio in the BODY frame while the chassis was
reared out of the contact plane, and it disappeared when the frame was
corrected. The apparent signal was the error.

The physics behind it is simple and it closes the question:

* A robot climbing steadily is in equilibrium, so the tangential force is
  ``m g sin(gamma)`` whatever mu is. Equilibrium pins tau at
  ``tan(gamma)`` -- a property of the GEOMETRY.
* tau reveals mu only when the contact saturates, and saturation needs a
  demand above ``mu g cos(gamma)``. On level ground the drivetrain cannot
  produce one: ``MAX_LINEAR_ACCEL`` is **2.0 m/s^2** against
  ``mu g = 3.43 m/s^2`` at the range's slick end. **The robot cannot spin
  its wheels on the flat.**
* On a grade the margin shrinks and saturation becomes reachable -- but
  that is precisely where equilibrium has already pinned tau.

The two conditions never overlap, so:

    **Coulomb friction is NOT identifiable on this robot, with an IMU and
    wheel encoders, anywhere in the Yard's operating envelope.**

That is a statement about the robot and its instrumentation, not about
this estimator, and it is the reason nothing here reports a friction
coefficient. What is reported is a **traction-demand ratio**: precisely
defined by (2), a true lower bound on mu, and carrying no further
information about mu. ``mu_hat`` is ``mu_lower`` clamped into the range
``yard_params.yaml`` declares -- public design knowledge, not episode
state -- and there is **no fitted constant anywhere in this channel**. A
regression of tau onto mu would have fitted the tuning set and meant
nothing off it.

The consequence for the experiment is worth stating plainly rather than
leaving for C2-M2.1 to discover: **B3 is a grade-aware controller with a
traction bound, not a friction-aware one.** The privileged advantage B2
holds over it is, as tuned, exactly one number -- throttle interpolated
on true mu -- and the observer cannot recover that number. How much of
B2's performance survives anyway is what the benchmark measures.

**Two known exceptions to the bound**, both stated rather than smoothed:

1. **A slope break.** The bound assumes one plane under the whole robot.
   The robot straddles the foot of a ramp for one wheelbase of travel,
   with its rear axle still on the apron, and there the two contact
   planes have different capacities. Measured on Route B seeds 1, 2 and
   4, all of which stalled in exactly that position.
2. **A vertical face.** Route C's 24 mm curb pushes back with a NORMAL
   reaction, which ``mu * f_n`` does not bound. Measured: Route C is the
   one route whose bound breaks with the robot correctly on the face.

Neither is detectable from an IMU and encoders alone -- both would need
to know where the robot is relative to the terrain -- so the benchmark
reports ``mu_bound_held`` as a measured rate rather than asserting it.
"""

import math

from dataclasses import dataclass, field


G = 9.81                     # m/s^2

# ── grade channel ────────────────────────────────────────────────────────
# Derived, not tuned: Route C's rubble has correlation length 0.12 m
# (yard_params.yaml routes.c.rubble.correlation_length) and the robot
# crosses it at roughly 0.25 m/s, so its features arrive as a ~0.48 s
# disturbance. A 0.5 s low-pass suppresses that while still settling to a
# step in grade within ~0.12 m of travel.
TAU_GRADE = 0.5              # s

# Samples of quasi-stationary flat ground needed before the reference is
# trusted. At the observer's 50 Hz that is 0.5 s.
REF_SAMPLES = 25

# "Quasi-stationary" for the purpose of taking the reference.
REF_MAX_SPEED = 0.02         # m/s of commanded linear speed
REF_MAX_RATE = 0.05          # rad/s of body pitch rate

# ── the two confidence thresholds, set from measured DISTRIBUTIONS ───────
#
# Both were first guessed and both were wrong, in the same direction: the
# guesses came from a comparison of FILTERED pitch against grade on a
# non-randomised Route A episode (0.03 deg), and the quantity they
# actually gate is the scatter of the RAW signal about the filter under
# full randomisation, which is 50 times larger. Measured over randomised
# episodes on all three routes, on the ramp face:
#
#     route/seed   rough p50   rough p90   |pitch rate| p50   p90
#     a / 11         2.333       5.026          0.073        0.819
#     a / 23         1.736       5.051          0.030        0.538
#     b / 11         1.592       3.460          0.286        0.976
#     c / 11         2.196       2.891          0.130        0.412
#     c / 23         2.670      10.519          0.099        0.703
#
# The first guesses (2.0 deg and 0.5 rad/s) sat at or below the MEDIAN of
# both, so the observer disqualified itself on most of the ramp and B3 ran
# in fallback 78-94 % of the time.
#
# These are set from the input distributions and NOT from any success
# rate: the full confidence point is around the median of the smooth
# routes, and the zero point above the 90th percentile of every route but
# Route C's rubble tail, which is exactly the population that should score
# zero. Chosen before B3's outcome on these routes was looked at.
RATE_TRANSIENT = 1.5         # rad/s, above p90 on every route
ROUGH_FULL_CONF = math.radians(1.5)
ROUGH_ZERO_CONF = math.radians(6.0)

# ── traction channel ─────────────────────────────────────────────────────
# Below this the longitudinal force is dominated by rolling resistance and
# contact settling rather than by traction demand, and tau means nothing.
MIN_SPEED_FOR_TAU = 0.05     # m/s of wheel surface speed

# Skid-steer scrub puts a large lateral force through the same contact
# patch, so a hard turn contaminates the longitudinal balance tau assumes.
# 0.2 rad/s is 40 % of MAX_ANG.
MAX_YAW_FOR_TAU = 0.2        # rad/s

# An acceleration deficit this large means the wheels are delivering
# markedly less than they command: the contact is saturated and tau is a
# tight estimate of mu rather than a loose bound. 0.5 m/s^2 is a quarter
# of MAX_LINEAR_ACCEL.
SATURATION_DEFICIT = 0.5     # m/s^2

# The normal specific force must be near its static value for tau to be a
# traction measurement at all. Derived: at rest on grade g the
# accelerometer reads f_z = g*cos(grade), and the steepest surface the
# Yard builds is Route B at 26 deg + 2 deg of jitter, so the static band
# is [0.883 g, 1.000 g]. A +/-30 % dynamic allowance around that rejects
# the two cases that are not measurements -- a wheel momentarily airborne
# (f_z -> 0) and a slam onto rubble (f_z >> g) -- while leaving normal
# weight transfer inside. Without this gate the running bound is set by a
# ramp-entry impact rather than by traction: measured, the bound held on
# only 60 % of Route B's samples.
FZ_MIN = 0.7 * G
FZ_MAX = 1.3 * G

# How stale an input may be before the estimate is withdrawn. The IMU runs
# at 50 Hz (coco_robo2.xacro <update_rate>), so this is five missed
# samples.
MAX_AGE = 0.1                # s


# ── the information boundary ─────────────────────────────────────────────
@dataclass(frozen=True)
class DeployableSignals:
    """Everything the observer is allowed to see. Nothing else exists.

    Every field here is something the physical robot publishes today:
    ``/imu`` supplies the attitude, the rates and the specific force;
    ``/joint_states`` supplies the wheel velocities; the commanded twist
    is what the controller itself just sent.

    ``accel_body`` is SPECIFIC FORCE in the body frame, which is what an
    accelerometer measures -- at rest on flat ground it reads
    ``(0, 0, +9.81)``, not zero. Getting that wrong inverts the sign of
    the whole traction channel, so it is stated here rather than left to
    the reader.
    """

    stamp: float                      # s, simulation clock
    roll: float                       # rad, IMU attitude
    pitch: float                      # rad, NEGATIVE nose-up (measured)
    yaw: float                        # rad
    roll_rate: float                  # rad/s, IMU gyro
    pitch_rate: float                 # rad/s
    yaw_rate: float                   # rad/s
    accel_body: tuple                 # (fx, fy, fz) m/s^2, specific force
    wheel_speeds: tuple               # rad/s, per wheel, from encoders
    cmd_linear: float                 # m/s, what the controller asked for
    cmd_angular: float                # rad/s
    wheel_radius: float               # m, a robot constant, not state
    on_declared_flat: bool = False    # operator knowledge: "this is flat"


@dataclass(frozen=True)
class TerrainEstimate:
    """What the observer publishes. Never contains a ground-truth value."""

    stamp: float
    grade: float                      # rad, +ve uphill along heading
    grade_valid: bool
    grade_confidence: float           # [0, 1]
    grade_calibrated: bool            # has the flat reference been taken?
    grade_roughness: float            # rad, scatter of pitch about filter

    camber: float                     # rad, cross-slope, +ve left-up
    camber_valid: bool

    tau: float                        # (2), filtered traction ratio
    mu_lower: float                   # running max of tau: proven bound
    mu_hat: float                     # mu_lower clamped to declared range
    traction_valid: bool              # THIS SAMPLE was a valid measurement
    traction_confidence: float        # [0, 1]
    mu_established: bool = False      # the bound has beaten the a-priori floor
    saturated: bool = False           # contact at its limit right now
    deficit: float = 0.0              # m/s^2, wheel-implied minus actual

    reason: str = ''                  # why something is invalid, if it is

    @property
    def valid(self):
        return self.grade_valid and self.traction_valid


def _lp(prev, new, dt, tau):
    """First-order low-pass. dt <= 0 or tau <= 0 passes the input through."""
    if prev is None or dt <= 0.0 or tau <= 0.0:
        return float(new)
    a = dt / (tau + dt)
    return float(prev + a * (new - prev))


class GradeEstimator:
    """Body attitude to terrain grade, by equation (1).

    Holds the flat-ground reference, the low-pass state and the running
    estimate of how far body pitch is scattering about it.
    """

    def __init__(self, tau=TAU_GRADE, ref_samples=REF_SAMPLES):
        self.tau = float(tau)
        self.ref_samples = int(ref_samples)
        self.reset()

    def reset(self):
        self._pitch_f = None
        self._roll_f = None
        self._rough = 0.0
        self._ref_pitch = 0.0
        self._ref_roll = 0.0
        self._ref_n = 0
        self._ref_sum = 0.0
        self._ref_sum_roll = 0.0
        self.calibrated = False

    def calibrate_flat(self, pitch, roll=0.0):
        """Install the flat-ground reference measured off-line.

        The alternative -- a stationary window at the start of every
        episode -- would hand the observer-driven controller steps the
        baselines do not get. This is a robot constant, so it is measured
        once where the robot is known to be level and level only.
        """
        self._ref_pitch = float(pitch)
        self._ref_roll = float(roll)
        self.calibrated = True

    def update(self, sig, dt):
        """Returns (grade, camber, confidence, roughness)."""
        # ── flat reference ───────────────────────────────────────────────
        # Taken only where the caller declares the ground flat AND the
        # robot is quasi-stationary. Both conditions matter: a moving
        # robot on flat ground still pitches under its own acceleration,
        # and that pitch would be baked into the reference forever.
        if (sig.on_declared_flat
                and abs(sig.cmd_linear) <= REF_MAX_SPEED
                and abs(sig.pitch_rate) <= REF_MAX_RATE
                and not self.calibrated):
            self._ref_sum += sig.pitch
            self._ref_sum_roll += sig.roll
            self._ref_n += 1
            if self._ref_n >= self.ref_samples:
                self._ref_pitch = self._ref_sum / self._ref_n
                self._ref_roll = self._ref_sum_roll / self._ref_n
                self.calibrated = True

        self._pitch_f = _lp(self._pitch_f, sig.pitch, dt, self.tau)
        self._roll_f = _lp(self._roll_f, sig.roll, dt, self.tau)

        # Scatter of the raw signal about the filtered one. This is the
        # live measurement of "body pitch is not terrain grade".
        resid = abs(sig.pitch - self._pitch_f)
        self._rough = _lp(self._rough, resid, dt, self.tau)

        grade = -(self._pitch_f - self._ref_pitch)
        camber = self._roll_f - self._ref_roll

        # ── confidence ───────────────────────────────────────────────────
        # Two independent penalties, multiplied, because either alone is
        # enough to make the number meaningless.
        c_rough = _span(self._rough, ROUGH_ZERO_CONF, ROUGH_FULL_CONF)
        c_rate = _span(abs(sig.pitch_rate), RATE_TRANSIENT, 0.0)
        conf = c_rough * c_rate
        if not self.calibrated:
            # Usable, but the reference is an assumption rather than a
            # measurement. Say so in the number as well as in the flag.
            conf *= 0.5
        return grade, camber, conf, self._rough


def _span(x, at_zero, at_one):
    """Linear ramp: 1.0 at ``at_one``, 0.0 at ``at_zero``, clamped."""
    if at_zero == at_one:
        return 1.0
    t = (x - at_zero) / (at_one - at_zero)
    return max(0.0, min(1.0, t))


class TractionEstimator:
    """Traction ratio by equation (2), and the bound it proves.

    Estimates no velocity and integrates nothing. See the module
    docstring for why, and for the measured limit on what this can
    resolve.
    """

    def __init__(self, mu_range=(0.35, 0.70)):
        self.mu_lo, self.mu_hi = float(mu_range[0]), float(mu_range[1])
        self.reset()

    def reset(self):
        self.mu_lower = 0.0
        self.established = False
        self._wheel_prev = None
        self._t_prev = None
        self._tau_f = 0.0

    def update(self, sig, grade, dt):
        """Returns (tau, mu_lower, mu_hat, valid, confidence, sat, deficit)."""
        w_mean = (sum(sig.wheel_speeds) / len(sig.wheel_speeds)
                  if sig.wheel_speeds else 0.0)
        v_wheel = w_mean * sig.wheel_radius

        # Wheel-implied acceleration, straight off the encoders. This is
        # differentiated, never integrated: a derivative of a clean signal
        # is noisy, an integral of a biased one is wrong, and only one of
        # those is recoverable by filtering.
        a_wheel = 0.0
        if self._wheel_prev is not None and dt > 0.0:
            a_wheel = (v_wheel - self._wheel_prev) / dt
        self._wheel_prev = v_wheel

        f_x = float(sig.accel_body[0])
        f_z = float(sig.accel_body[2])
        # Gravity removed with the MEASURED body pitch, not the estimated
        # terrain grade. The two are different quantities and only the
        # first is a direct sensor reading.
        a_body = f_x + G * math.sin(sig.pitch)
        deficit = a_wheel - a_body

        # tau is the friction utilisation IN THE CONTACT FRAME, and both
        # its numerator and its denominator are measured.
        #
        # Two corrections, both forced by measurement, both worth keeping
        # written down because each looked right until it was checked.
        #
        # 1. The denominator is f_z, not g*cos(grade). Coulomb bounds the
        #    friction force by mu times the ACTUAL normal force, and an
        #    accelerometer measures that directly. Modelling it as the
        #    static gravity component ignores weight transfer, payload and
        #    rubble, and the bound tau <= mu then held on only 27 % of
        #    Route B's samples.
        #
        # 2. The ratio is taken in the CONTACT frame, not the body frame.
        #    Coulomb's law is a statement about forces at the contact
        #    patch, and body-x is the contact tangent only while the
        #    chassis is aligned with the surface it stands on. It is not:
        #    measured, the robot rears to body pitch -30 deg on a 26.66
        #    deg chute. With the raw body-frame ratio that reads
        #    tan(30 deg) = 0.577 where the surface truly demands
        #    tan(26.66 deg) = 0.502, so the bound broke on episodes whose
        #    mu sat between the two -- 47 % of Route B's samples across
        #    four seeds.
        #
        # The surface-relative pitch is phi = pitch + grade_hat (zero when
        # the chassis lies along the slope), and rotating the specific
        # force by it gives the tangential and normal components at the
        # patch. **grade_hat is the ESTIMATE**, so this stays deployable;
        # it is the one place the traction channel leans on the grade
        # channel, which is the better-conditioned of the two.
        phi = sig.pitch + grade
        c_p, s_p = math.cos(phi), math.sin(phi)
        f_t = f_x * c_p + f_z * s_p
        f_n = -f_x * s_p + f_z * c_p
        tau = f_t / max(f_n, 1.0)

        # ── validity ─────────────────────────────────────────────────────
        # Gated BEFORE the filter, so a rejected sample does not enter the
        # low-pass and then leak into the bound half a second later. That
        # was the first version's actual failure: the sample at the ramp
        # foot was gated, and the filtered value it had already moved
        # still set mu_lower to 0.972 against a true mu of 0.592.
        reason = ''
        if abs(v_wheel) < MIN_SPEED_FOR_TAU:
            reason = 'wheel speed below the traction-demand floor'
        elif abs(sig.yaw_rate) > MAX_YAW_FOR_TAU:
            reason = 'turning: skid-steer scrub contaminates the balance'
        elif not FZ_MIN <= f_n <= FZ_MAX:
            # The CONTACT normal, for the same reason tau uses it.
            reason = f'normal load off its static band: f_n = {f_n:.2f}'
        elif abs(tau) > self.mu_hi:
            # Coulomb bounds the friction force by mu * N, so a
            # longitudinal specific force above the declared friction
            # ceiling is not a grippy surface -- it is evidence that the
            # assumption behind (2) has been violated. Measured: the robot
            # rearing at the foot of the 26 deg chute reaches body pitch
            # -32 deg against a 26.66 deg surface, and once body-x has
            # left the contact plane the ratio stops being a friction
            # utilisation at all. Rejected rather than clamped, because a
            # clamped value would be indistinguishable from a real
            # measurement at the ceiling.
            reason = f'tau {tau:.2f} exceeds the declared ceiling: not a grip'
        valid = not reason

        sat = valid and deficit > SATURATION_DEFICIT

        if valid:
            self._tau_f = _lp(self._tau_f, tau, dt, TAU_GRADE)
            if self._tau_f > self.mu_lower:
                # A bound is only ever tightened, and only by a valid
                # sample that passed every gate above.
                self.mu_lower = float(self._tau_f)
                # ESTABLISHED means the bound has become informative: it
                # is now tighter than the a-priori floor the friction
                # range already guarantees. Below that, `mu_hat` is the
                # floor whatever the robot has measured, and scheduling on
                # it would be scheduling on an assumption while claiming
                # to schedule on an observation.
                #
                # This is where the observer's honest limit shows. A
                # steady climb only proves mu >= tan(grade), so Route A's
                # 12 deg face proves mu >= 0.213 -- WEAKER than the
                # declared floor of 0.35. **Friction is not observable on
                # Route A by this method**, and the observer reports that
                # rather than manufacturing a number. Route B's 26 deg
                # chute demands 0.49 and does establish a bound; Route C
                # sits between, at tan(16 deg) = 0.29 plus whatever the
                # curb asks for.
                if self.mu_lower > self.mu_lo:
                    self.established = True

        mu_hat = max(self.mu_lo, min(self.mu_hi, self.mu_lower))

        # Confidence is what the bound has actually proved, not how long
        # it has been running. A robot that has never asked much of the
        # contact knows nothing about it, and says so.
        conf = 0.0
        if valid and self.established:
            conf = _span(self.mu_lower, self.mu_lo, self.mu_hi)
            if sat:
                # At saturation tau IS mu rather than a bound on it.
                conf = min(1.0, conf + 0.5)
        return (self._tau_f, self.mu_lower, mu_hat, valid, conf, sat,
                deficit, reason, self.established)


class TerrainObserver:
    """Grade and traction from :class:`DeployableSignals`. Nothing else.

    Failure is explicit. A missing sample, a clock that has not advanced
    and a stale one each produce an estimate with the relevant
    ``*_valid`` cleared and a ``reason`` set, never a plausible-looking
    number. That is the C2-M1.5 lesson applied at the source: the failure
    mode that costs a session is the one that reads correct.
    """

    def __init__(self, mu_range=(0.35, 0.70), tau_grade=TAU_GRADE,
                 max_age=MAX_AGE):
        self.grade_est = GradeEstimator(tau=tau_grade)
        self.traction_est = TractionEstimator(mu_range=mu_range)
        self.max_age = float(max_age)
        self.reset()

    def reset(self, flat_reference=None):
        """Clear the episode state. Keeps the flat reference if given.

        ``flat_reference`` is ``(pitch, roll)`` from
        ``CocoYardEnv.flat_reference`` — a robot constant, so re-supplying
        it every episode is bookkeeping, not information.
        """
        self.grade_est.reset()
        self.traction_est.reset()
        self._t_prev = None
        self._last = None
        if flat_reference is not None:
            self.grade_est.calibrate_flat(*flat_reference)

    @property
    def last(self):
        return self._last

    def update(self, sig):
        """Fold one sample in and return the current :class:`TerrainEstimate`."""
        if sig is None:
            return self._invalid(
                self._t_prev or 0.0, 'no signal: input was None')

        dt = 0.0 if self._t_prev is None else sig.stamp - self._t_prev
        if self._t_prev is not None:
            if dt <= 0.0:
                return self._invalid(sig.stamp,
                                     'clock did not advance between samples')
            if dt > self.max_age:
                # The gap itself is the fault. Restart the filters rather
                # than smearing a 0.5 s time constant across a hole in the
                # data and reporting the result as current.
                self.grade_est._pitch_f = None
                self.grade_est._roll_f = None
                self._t_prev = sig.stamp
                return self._invalid(
                    sig.stamp, f'stale input: {dt:.3f} s > {self.max_age:.3f} s')
        self._t_prev = sig.stamp

        grade, camber, g_conf, rough = self.grade_est.update(sig, dt)
        (tau, mu_lower, mu_hat, t_valid, t_conf, sat, deficit,
         t_reason, established) = self.traction_est.update(sig, grade, dt)

        est = TerrainEstimate(
            stamp=sig.stamp,
            grade=grade, grade_valid=True, grade_confidence=g_conf,
            grade_calibrated=self.grade_est.calibrated,
            grade_roughness=rough,
            camber=camber, camber_valid=True,
            tau=tau, mu_lower=mu_lower, mu_hat=mu_hat,
            traction_valid=t_valid, traction_confidence=t_conf,
            mu_established=established,
            saturated=sat, deficit=deficit, reason=t_reason)
        self._last = est
        return est

    def _invalid(self, stamp, reason):
        est = TerrainEstimate(
            stamp=stamp, grade=0.0, grade_valid=False, grade_confidence=0.0,
            grade_calibrated=self.grade_est.calibrated, grade_roughness=0.0,
            camber=0.0, camber_valid=False,
            # The BOUND survives an invalid sample: it was proved by
            # earlier valid ones and nothing here disproves it. Only the
            # instantaneous channels are withdrawn.
            tau=0.0, mu_lower=self.traction_est.mu_lower,
            mu_hat=max(self.traction_est.mu_lo,
                       min(self.traction_est.mu_hi,
                           self.traction_est.mu_lower)),
            traction_valid=False, traction_confidence=0.0,
            mu_established=self.traction_est.established,
            saturated=False, deficit=0.0, reason=reason)
        self._last = est
        return est
