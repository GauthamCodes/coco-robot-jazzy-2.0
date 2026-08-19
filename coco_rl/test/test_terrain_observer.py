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
C2-M2.0: the terrain observer, its information boundary, and B3.

The synthetic signals below are not a convenience. They let the sign
convention, the staleness contract and the fallback be asserted without a
simulator, which is the same reason ``reward.py`` is pure — and the
values they are built from are the MEASURED ones: at rest on flat ground
an accelerometer reads (0, 0, +9.81), and on a grade of gamma the robot
holds body pitch -gamma and reads f_x = +g sin(gamma), f_z = g cos(gamma).
Route B at 26 deg measured f_x 4.2-5.0 and f_z 8.4-8.8 against the
predicted 4.30 and 8.82.
"""

import math

from coco_rl.baselines import (B1, B2, B3, DEFAULT_SCHEDULE, TUNED_SCHEDULE,
                               schedule_gains)
from coco_rl.lateral import HEADING_GAIN, LATERAL_CLAMP, LATERAL_GAIN
from coco_rl.terrain_observer import (DeployableSignals, G, MAX_AGE,
                                      TerrainEstimate, TerrainObserver)

import pytest


IMU_DT = 1.0 / 50.0          # coco_robo2.xacro <update_rate>50</update_rate>


def signal(t, grade_deg=0.0, accel=0.0, wheel=3.5, yaw_rate=0.0,
           pitch_noise=0.0, camber_deg=0.0, fz_scale=1.0):
    """One deployable sample for a robot on ``grade_deg``, accelerating at
    ``accel`` m/s^2 along the slope."""
    g = math.radians(grade_deg)
    c = math.radians(camber_deg)
    return DeployableSignals(
        stamp=t,
        roll=c, pitch=-g + pitch_noise, yaw=0.0,
        roll_rate=0.0, pitch_rate=0.0, yaw_rate=yaw_rate,
        accel_body=(accel + G * math.sin(g), 0.0,
                    G * math.cos(g) * fz_scale),
        wheel_speeds=(wheel,) * 4,
        cmd_linear=0.3, cmd_angular=0.0, wheel_radius=0.0585)


def drive(observer, n=200, **kw):
    """Feed ``n`` samples at 50 Hz and return the final estimate."""
    est = None
    for i in range(n):
        est = observer.update(signal((i + 1) * IMU_DT, **kw))
    return est


# ── grade ────────────────────────────────────────────────────────────────
def test_flat_ground_reads_zero_grade():
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    est = drive(ob, grade_deg=0.0)
    assert est.grade_valid
    assert abs(math.degrees(est.grade)) < 0.01


def test_positive_grade_and_the_sign_convention():
    """Nose-up is NEGATIVE pitch and POSITIVE grade.

    Measured this session on Route A's uniform 12.000 deg face: obs[7]
    read -12.00 deg. A rename of body pitch to grade would be wrong in
    sign, which is the whole reason equation (1) carries a minus.
    """
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    est = drive(ob, grade_deg=12.0)
    assert est.grade > 0.0, 'climbing must report a POSITIVE grade'
    assert math.degrees(est.grade) == pytest.approx(12.0, abs=0.05)
    # and the raw signal it came from really was nose-DOWN in sign
    assert signal(0.0, grade_deg=12.0).pitch < 0.0


def test_negative_grade_is_a_descent():
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    est = drive(ob, grade_deg=-20.0)
    assert math.degrees(est.grade) == pytest.approx(-20.0, abs=0.05)


def test_camber_is_reported_from_roll():
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    est = drive(ob, grade_deg=12.0, camber_deg=8.0)
    assert math.degrees(est.camber) == pytest.approx(8.0, abs=0.05)


def test_noise_is_filtered_and_reported_as_roughness():
    """A grade under noise still converges, and the observer says it is noisy."""
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    est = None
    for i in range(400):
        # deterministic zig-zag: no RNG, so the assertion cannot flake
        noise = math.radians(3.0) * (1 if i % 2 else -1)
        est = ob.update(signal((i + 1) * IMU_DT, grade_deg=16.0,
                               pitch_noise=noise))
    assert math.degrees(est.grade) == pytest.approx(16.0, abs=1.0)
    assert est.grade_roughness > math.radians(1.0), 'noise must show up'
    clean = TerrainObserver()
    clean.reset(flat_reference=(0.0, 0.0))
    quiet = drive(clean, n=400, grade_deg=16.0)
    assert quiet.grade_confidence > est.grade_confidence


def test_flat_reference_is_subtracted_not_assumed():
    """A robot that sits nose-up on level ground must still read zero grade."""
    bias = math.radians(-2.0)          # 2 deg nose-up standing pitch
    ob = TerrainObserver()
    ob.reset(flat_reference=(bias, 0.0))
    est = drive(ob, grade_deg=0.0, pitch_noise=bias)
    assert abs(math.degrees(est.grade)) < 0.05
    assert est.grade_calibrated


def test_uncalibrated_observer_says_so_and_halves_its_confidence():
    cal, raw = TerrainObserver(), TerrainObserver()
    cal.reset(flat_reference=(0.0, 0.0))
    raw.reset()
    a = drive(cal, grade_deg=12.0)
    b = drive(raw, grade_deg=12.0)
    assert a.grade_calibrated and not b.grade_calibrated
    assert b.grade_confidence == pytest.approx(a.grade_confidence / 2.0)


def test_reference_can_be_learned_from_declared_flat_samples():
    ob = TerrainObserver()
    ob.reset()
    bias = math.radians(-1.5)
    for i in range(60):
        s = signal((i + 1) * IMU_DT, pitch_noise=bias, wheel=0.0)
        ob.update(DeployableSignals(**{**s.__dict__,
                                       'on_declared_flat': True,
                                       'cmd_linear': 0.0}))
    assert ob.grade_est.calibrated
    assert ob.grade_est._ref_pitch == pytest.approx(bias, abs=1e-6)


# ── staleness and the clock ──────────────────────────────────────────────
def test_stale_input_invalidates_rather_than_extrapolating():
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    drive(ob, n=100, grade_deg=12.0)
    late = ob.update(signal(100 * IMU_DT + MAX_AGE * 5, grade_deg=12.0))
    assert not late.grade_valid
    assert not late.valid
    assert 'stale' in late.reason


def test_a_clock_that_does_not_advance_is_a_fault():
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    ob.update(signal(1.0, grade_deg=12.0))
    same = ob.update(signal(1.0, grade_deg=12.0))
    assert not same.grade_valid
    assert 'clock' in same.reason
    back = ob.update(signal(0.5, grade_deg=12.0))
    assert not back.grade_valid


def test_missing_signal_is_explicit():
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    est = ob.update(None)
    assert not est.valid
    assert 'None' in est.reason


def test_estimate_carries_the_stamp_of_its_input():
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    est = ob.update(signal(4.25, grade_deg=12.0))
    assert est.stamp == 4.25


def test_estimate_exposes_every_documented_field():
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    est = drive(ob, grade_deg=12.0)
    for field in ('stamp', 'grade', 'grade_valid', 'grade_confidence',
                  'grade_calibrated', 'grade_roughness', 'camber',
                  'camber_valid', 'tau', 'mu_lower', 'mu_hat',
                  'traction_valid', 'traction_confidence', 'mu_established',
                  'saturated', 'deficit', 'reason'):
        assert hasattr(est, field), field
    assert isinstance(est, TerrainEstimate)


# ── traction ─────────────────────────────────────────────────────────────
def test_level_cruise_demands_no_traction():
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    est = drive(ob, grade_deg=0.0, accel=0.0)
    assert est.traction_valid
    assert abs(est.tau) < 0.01
    assert not est.mu_established, 'nothing has been proved about the contact'


def test_a_steady_climb_proves_mu_is_at_least_tan_grade():
    """The bound, and its honest weakness, in one test."""
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    est = drive(ob, n=400, grade_deg=26.0, accel=0.0)
    assert est.tau == pytest.approx(math.tan(math.radians(26.0)), abs=0.01)
    assert est.mu_lower == pytest.approx(math.tan(math.radians(26.0)),
                                         abs=0.01)
    assert est.mu_established, '0.488 beats the 0.35 floor'


def test_a_gentle_climb_proves_nothing_beyond_the_a_priori_floor():
    """Route A's own result, as a unit test.

    tan(12 deg) = 0.213, which is WEAKER than the declared floor of 0.35,
    so the bound never becomes informative and B3 stays in fallback. This
    is the physical limit of the method, not a defect in it.
    """
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    est = drive(ob, n=400, grade_deg=12.0, accel=0.0)
    assert est.mu_lower < 0.35
    assert not est.mu_established
    assert est.mu_hat == 0.35


def test_saturated_acceleration_recovers_mu_directly():
    """At the friction limit tau IS mu: a = g(mu cos g - sin g)."""
    mu, grade = 0.60, 16.0
    g = math.radians(grade)
    a_sat = G * (mu * math.cos(g) - math.sin(g))
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    est = drive(ob, n=400, grade_deg=grade, accel=a_sat)
    assert est.tau == pytest.approx(mu, abs=0.01)
    assert est.mu_hat == pytest.approx(mu, abs=0.01)


def test_the_bound_only_ever_tightens():
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    drive(ob, n=200, grade_deg=26.0)
    peak = ob.last.mu_lower
    drive(ob, n=200, grade_deg=0.0)
    assert ob.last.mu_lower == pytest.approx(peak)


def test_low_speed_is_rejected_not_guessed():
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    est = drive(ob, grade_deg=26.0, wheel=0.1)     # 0.006 m/s of wheel
    assert not est.traction_valid
    assert 'floor' in est.reason
    assert est.mu_lower == 0.0


def test_turning_is_rejected_because_skid_steer_scrubs():
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    est = drive(ob, grade_deg=26.0, yaw_rate=0.4)
    assert not est.traction_valid
    assert 'scrub' in est.reason


def test_an_off_band_normal_load_is_rejected():
    """A wheel momentarily airborne, and a slam onto rubble."""
    for scale, in ((0.4,), (1.6,)):
        ob = TerrainObserver()
        ob.reset(flat_reference=(0.0, 0.0))
        est = drive(ob, grade_deg=16.0, fz_scale=scale)
        assert not est.traction_valid
        assert 'normal load' in est.reason


def test_tau_above_the_declared_ceiling_is_rejected_not_clamped():
    """The measured failure: the robot rearing at the foot of the chute.

    Body pitch reached -32 deg against a 26.66 deg surface, body-x left
    the contact plane, and the running bound was driven to 0.972 against a
    true mu of 0.592. A value above the ceiling is a violated assumption,
    so it is dropped rather than clamped -- a clamped one would be
    indistinguishable from a real measurement at the ceiling.
    """
    ob = TerrainObserver(mu_range=(0.35, 0.70))
    ob.reset(flat_reference=(0.0, 0.0))
    est = drive(ob, n=400, grade_deg=16.0, accel=8.0)
    assert not est.traction_valid
    assert 'ceiling' in est.reason
    assert ob.traction_est.mu_lower <= 0.70


def test_the_bound_survives_a_sample_that_does_not():
    """An invalid sample withdraws the reading, never the knowledge."""
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    drive(ob, n=200, grade_deg=26.0)
    proved = ob.last.mu_lower
    assert ob.last.mu_established
    est = ob.update(signal(200 * IMU_DT + IMU_DT, grade_deg=26.0,
                           yaw_rate=0.9))
    assert not est.traction_valid
    assert est.mu_lower == pytest.approx(proved)
    assert est.mu_established, 'a turn does not unprove the bound'


def test_mu_hat_is_clamped_into_the_declared_range():
    ob = TerrainObserver(mu_range=(0.35, 0.70))
    ob.reset(flat_reference=(0.0, 0.0))
    est = drive(ob, n=400, grade_deg=0.0)
    assert 0.35 <= est.mu_hat <= 0.70


# ── the information boundary ─────────────────────────────────────────────
def test_ground_truth_is_a_different_type_with_no_shared_fields():
    from coco_rl.sensor_model import GroundTruth
    deployable = set(DeployableSignals.__dataclass_fields__)
    truth = set(GroundTruth.__slots__)
    assert not (deployable & truth), (
        'a field name shared between the two would let a copy-paste '
        'across the boundary typecheck')


def test_the_observer_cannot_consume_ground_truth():
    from coco_rl.sensor_model import GroundTruth
    truth = GroundTruth(x=1.0, y=0.0, z=0.1, grade=0.3, camber=0.0,
                        friction=0.7, v_body=0.2, v_wheel=0.3, slip=0.3)
    ob = TerrainObserver()
    ob.reset(flat_reference=(0.0, 0.0))
    with pytest.raises(AttributeError):
        ob.update(truth)


# ── B2 is unchanged by the extraction ────────────────────────────────────
def test_schedule_gains_reproduces_b2s_original_arithmetic():
    """Pins the extracted function against B2's inlined body.

    ``schedule_gains`` was lifted out of ``B2.reset`` so B3 could reuse
    the RELATIONSHIP rather than a copy. The values below are recomputed
    here the way the original wrote them, so a change to either side
    fails this rather than silently re-tuning the privileged baseline.
    """
    cfg = TUNED_SCHEDULE['b']
    mu_range, friction = (0.35, 0.70), 0.592
    grade_deg, nominal = 26.66, 26.0
    t = (friction - mu_range[0]) / (mu_range[1] - mu_range[0])
    expect_throttle = min(
        1.0,
        cfg['throttle_lo'] + t * (cfg['throttle_hi'] - cfg['throttle_lo'])
        + cfg['grade_k'] * (grade_deg - nominal))
    expect_lateral = cfg['lateral_lo'] + t * (cfg['lateral_hi']
                                              - cfg['lateral_lo'])
    got = schedule_gains(cfg, friction=friction, grade_deg=grade_deg,
                         nominal_grade_deg=nominal, mu_range=mu_range)
    assert got['throttle'] == pytest.approx(expect_throttle)
    assert got['lateral'] == pytest.approx(expect_lateral)
    assert got['heading'] == cfg['heading']
    assert got['clamp'] == cfg['clamp']
    assert got['deck_throttle'] == cfg['deck_throttle']


def test_b2_still_resolves_the_gains_it_always_did():
    params = _params()
    b2 = B2(schedule=TUNED_SCHEDULE, params=params)
    b2.reset(_sample(params, friction=0.592, grade_deg=26.66), 'b')
    direct = schedule_gains(
        TUNED_SCHEDULE['b'], friction=0.592, grade_deg=26.66,
        nominal_grade_deg=params['routes']['b']['grade_deg'],
        mu_range=params['friction']['range'])
    assert b2.gains == direct


# ── B3 ───────────────────────────────────────────────────────────────────
class _Route:
    def __init__(self, y_centre=0.0, friction=0.6, grade_deg=26.0):
        self.y_centre = y_centre
        self.friction = friction
        self.grade_deg = grade_deg


class _Sample:
    def __init__(self, routes):
        self.routes = routes


def _sample(params, friction=0.6, grade_deg=26.0):
    return _Sample({k: _Route(params['routes'][k]['y_centre'], friction,
                              grade_deg) for k in ('a', 'b', 'c')})


def _params():
    from coco_sim.yard import load_params
    return load_params()


def _b3(params, route='b'):
    b = B3(schedule=TUNED_SCHEDULE, params=params)
    b.calibrate((0.0, 0.0))
    b.reset(_sample(params), route)
    return b


def test_b3_starts_in_fallback_and_it_is_exactly_b1():
    params = _params()
    b3, b1 = _b3(params), B1(params=params)
    b1.reset(_sample(params), 'b')
    obs = [0.0, 0.0, 0.0, 1.0, 0.2, 0.0, 0.0, -0.45]
    assert b3(obs, -1.0, 0.05) == b1(obs, -1.0, 0.05)
    assert not b3.engaged
    assert b3.fallback_rate == 1.0


def test_b3_engages_once_the_bound_is_established():
    params = _params()
    b3 = _b3(params)
    obs = [0.0, 0.0, 0.0, 1.0, 0.2, 0.0, 0.0, math.radians(-26.0)]
    for i in range(400):
        b3.observe(signal((i + 1) * IMU_DT, grade_deg=26.0))
        b3(obs, -1.0, 0.0)
    assert b3.last_estimate.mu_established
    assert b3.engaged
    assert b3.gains['lateral'] == pytest.approx(6.0), 'B2 route gains, not B1'


def test_b3_falls_back_when_the_estimate_goes_stale():
    params = _params()
    b3 = _b3(params)
    obs = [0.0, 0.0, 0.0, 1.0, 0.2, 0.0, 0.0, math.radians(-26.0)]
    for i in range(400):
        b3.observe(signal((i + 1) * IMU_DT, grade_deg=26.0))
        b3(obs, -1.0, 0.0)
    assert b3.engaged
    b3.observe(signal(400 * IMU_DT + MAX_AGE * 10, grade_deg=26.0))
    b3(obs, -1.0, 0.0)
    assert not b3.engaged
    assert b3.gains['lateral'] == pytest.approx(LATERAL_GAIN)
    assert b3.gains['heading'] == pytest.approx(HEADING_GAIN)
    assert b3.gains['clamp'] == pytest.approx(LATERAL_CLAMP)


def test_b3_output_stays_inside_the_action_space():
    """Saturation, under a lateral error far past anything the Yard offers."""
    params = _params()
    b3 = _b3(params)
    for i in range(400):
        b3.observe(signal((i + 1) * IMU_DT, grade_deg=26.0))
    for y in (-50.0, -1.0, 0.0, 1.0, 50.0):
        for yaw in (-3.0, 0.0, 3.0):
            obs = [0.0, 0.0, math.sin(yaw), math.cos(yaw), 0.2, 0.0, 0.0,
                   math.radians(-26.0)]
            lin, ang = b3(obs, -1.0, y)
            assert -1.0 <= lin <= 1.0
            assert -1.0 <= ang <= 1.0


def test_b3_gains_stay_inside_the_schedules_own_endpoints():
    params = _params()
    lo, hi = params['friction']['range']
    cfg = TUNED_SCHEDULE['b']
    b3 = _b3(params)
    obs = [0.0, 0.0, 0.0, 1.0, 0.2, 0.0, 0.0, math.radians(-26.0)]
    bounds = sorted((cfg['throttle_lo'], cfg['throttle_hi']))
    for i in range(600):
        b3.observe(signal((i + 1) * IMU_DT, grade_deg=26.0))
        b3(obs, -1.0, 0.0)
        if b3.engaged:
            assert bounds[0] - 1e-9 <= b3.gains['throttle'] <= bounds[1] + 1e-9
            assert lo <= b3.last_estimate.mu_hat <= hi


def test_b3_never_reads_a_privileged_field():
    """B3 must not touch the sample's friction or grade.

    The sample it is handed carries both. This replaces them with objects
    that raise on access, so a future edit that reaches for either fails
    here instead of quietly turning B3 into B2.
    """
    class Tripwire:
        def __get__(self, obj, cls=None):
            raise AssertionError('B3 read a privileged field')

    class TrapRoute:
        friction = Tripwire()
        grade_deg = Tripwire()

        def __init__(self, y_centre):
            self.y_centre = y_centre

    params = _params()
    b3 = B3(schedule=TUNED_SCHEDULE, params=params)
    b3.calibrate((0.0, 0.0))
    b3.reset(_Sample({k: TrapRoute(params['routes'][k]['y_centre'])
                      for k in ('a', 'b', 'c')}), 'b')
    obs = [0.0, 0.0, 0.0, 1.0, 0.2, 0.0, 0.0, math.radians(-26.0)]
    for i in range(400):
        b3.observe(signal((i + 1) * IMU_DT, grade_deg=26.0))
        b3(obs, -1.0, 0.0)
    assert b3.engaged


def test_default_schedule_also_drives_b3():
    params = _params()
    b3 = B3(schedule=DEFAULT_SCHEDULE, params=params)
    b3.calibrate((0.0, 0.0))
    b3.reset(_sample(params), 'c')
    obs = [0.0, 0.0, 0.0, 1.0, 0.2, 0.0, 0.0, math.radians(-26.0)]
    for i in range(400):
        b3.observe(signal((i + 1) * IMU_DT, grade_deg=26.0))
        lin, ang = b3(obs, -1.0, 0.0)
        assert -1.0 <= lin <= 1.0 and -1.0 <= ang <= 1.0
