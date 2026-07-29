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
The platform approach, phase by phase, without a simulator.

Each phase is a pure function for the same reason descend_cmd is: the
cases that decide whether the grasp is reachable — never pivot on the
slope, always pivot on the platform, stop inside the window, keep the
blind leg straight — are cheap to assert here and expensive to observe in
Gazebo, where a wrong sign shows up as a robot that drove off a 0.65 m
ledge.
"""

import math

from coco_config.robot import (approach_stop_x, approach_window,
                               CHASSIS_FRONT_X, GRASP_MAX_LATERAL,
                               GRASP_SELF_COLLISION_X, TARGETS)

from custom_teleop.approach_server import (
    align_cmd, ALIGN_EPS, ALIGN_YAW_CLAMP, ALIGN_YAW_MIN, APPROACH_HZ,
    bearing_to, creep_cmd, CREEP_LEAD, CREEP_SPEED, CREEP_YAW_CLAMP,
    crest_cmd, CREST_PITCH_EPS, CREST_SPEED, format_status, servo_cmd,
    SERVO_HANDOFF_X, SERVO_LIN_MAX, SERVO_LIN_MIN, SERVO_YAW_CLAMP, wrap)

import pytest

RAMP_PITCH = 0.314        # 18 degrees, the grade the climb ends on
BLUE_STOP = approach_stop_x('blue')


def test_import():
    from custom_teleop import approach_server
    assert hasattr(approach_server, 'ApproachServer')
    assert hasattr(approach_server, 'main')


# ── crest ───────────────────────────────────────────────────────────────────
def test_the_slope_is_not_mistaken_for_the_platform():
    """
    0.314 rad of grade against a 0.06 rad threshold is not a close call.

    Both signs, because the up-slope and the down-slope pitch opposite
    ways and only one of them means "keep going".
    """
    for pitch in (RAMP_PITCH, -RAMP_PITCH):
        _, _, level = crest_cmd(pitch, 0.0)
        assert level is False


def test_the_platform_reads_as_level():
    _, _, level = crest_cmd(0.0, 0.0)
    assert level is True
    _, _, level = crest_cmd(CREST_PITCH_EPS * 0.9, 0.0)
    assert level is True


def test_the_crest_never_pivots_in_place():
    """
    The last 0.300 m of the approach is still on an 18 degree slope.

    A stationary skid-steer pivot on a grade is how this base loses its
    footing — the same rule ramp_driver.descend_cmd is built around — so
    forward speed must not depend on heading error at all.
    """
    for yaw_err in (-1.2, -0.4, 0.0, 0.4, 1.2):
        linear, _, _ = crest_cmd(-RAMP_PITCH, yaw_err)
        assert linear == pytest.approx(CREST_SPEED)


def test_the_crest_steers_back_toward_its_start_heading():
    _, ang_left, _ = crest_cmd(-RAMP_PITCH, 0.2)
    _, ang_right, _ = crest_cmd(-RAMP_PITCH, -0.2)
    assert ang_left < 0.0
    assert ang_right > 0.0


# ── servo ───────────────────────────────────────────────────────────────────
def test_the_servo_steers_toward_the_target():
    """
    A target to the left is a left turn.

    Getting this backwards drives the robot off the platform edge rather
    than into the lane it was sent to.
    """
    _, ang_left, _ = servo_cmd(1.0, +0.3, BLUE_STOP)
    _, ang_right, _ = servo_cmd(1.0, -0.3, BLUE_STOP)
    assert ang_left > 0.0
    assert ang_right < 0.0


def test_the_servo_absorbs_the_climbs_residual_lateral_error():
    """
    The lane hold leaves ~0.03 m; the servo is what removes it.

    At the range the target is first seen, 0.03 m is a small bearing and
    must still produce a correction rather than fall into a deadband.
    """
    _, angular, _ = servo_cmd(1.3, 0.03, BLUE_STOP)
    assert angular > 0.0


def test_the_servo_slows_as_it_closes():
    far, _, _ = servo_cmd(1.3, 0.0, BLUE_STOP)
    near, _, _ = servo_cmd(0.5, 0.0, BLUE_STOP)
    assert far > near
    assert SERVO_LIN_MIN <= near <= SERVO_LIN_MAX
    assert SERVO_LIN_MIN <= far <= SERVO_LIN_MAX


def test_the_servo_never_creeps_to_a_halt_short_of_the_handoff():
    """
    A proportional term alone approaches zero speed asymptotically.

    Without the floor the robot would spend the last few centimetres
    barely moving and time out one handoff short of the grasp.
    """
    _, _, done = servo_cmd(SERVO_HANDOFF_X + 0.01, 0.0, BLUE_STOP)
    assert done is False
    linear, _, _ = servo_cmd(SERVO_HANDOFF_X + 0.01, 0.0, BLUE_STOP)
    assert linear >= SERVO_LIN_MIN


def test_the_servo_hands_over_before_perception_goes_blind():
    """
    The handoff must leave the align something to see.

    target_finder's floor is 0.15 m of surface depth from a camera at
    base-x 0.125, so the last fix lands at a target axis near 0.29 m. Hand
    over above that or the align has no bearing to null.
    """
    assert SERVO_HANDOFF_X > 0.30
    _, _, done = servo_cmd(SERVO_HANDOFF_X, 0.0, BLUE_STOP)
    assert done is True


def test_the_servo_correction_is_clamped():
    for target_y in (-3.0, -1.0, 1.0, 3.0):
        _, angular, _ = servo_cmd(0.5, target_y, BLUE_STOP)
        assert abs(angular) <= SERVO_YAW_CLAMP + 1e-9


# ── align ───────────────────────────────────────────────────────────────────
def test_the_align_only_ever_turns_in_place():
    """Translating here would undo the range the servo just closed."""
    for bearing in (-0.5, -0.1, 0.1, 0.5):
        linear, _, _ = align_cmd(bearing)
        assert linear == 0.0


def test_the_align_turns_toward_the_target():
    _, ang_left, _ = align_cmd(0.2)
    _, ang_right, _ = align_cmd(-0.2)
    assert ang_left > 0.0
    assert ang_right < 0.0


def test_the_align_finishes_inside_the_lateral_budget():
    """
    ALIGN_EPS is what most of the blind leg's LATERAL error is made of.

    Against GRASP_MAX_LATERAL, not against the approach window: a residual
    heading carried through the creep moves the target sideways. It costs
    the *range* almost nothing — 0.166 m x (1 - cos 0.02) is 0.03 mm — so
    checking it against the 5.5 mm along-axis window, as this test once
    did, was comparing two different axes and passing by luck.

    0.02 rad over the 0.166 m creep is 3.3 mm. The budget is 10 mm and
    perception (~2 mm) and odometry (~1.7 mm) also draw on it, so heading
    is held to half.
    """
    _, _, done = align_cmd(ALIGN_EPS * 0.9)
    assert done is True
    creep_length = SERVO_HANDOFF_X - approach_stop_x('yellow')
    lateral = creep_length * math.sin(ALIGN_EPS)
    assert lateral < GRASP_MAX_LATERAL / 2.0, (
        f'{lateral * 1000:.2f} mm of heading-induced lateral spends more '
        f'than half the {GRASP_MAX_LATERAL * 1000:.0f} mm budget')


def test_the_align_never_commands_a_rate_the_base_cannot_execute():
    """
    Below ~0.1 rad/s the wheels buzz and the robot does not rotate.

    A pure proportional term would converge on "not quite aligned" and sit
    there until the timeout, with the status line reporting a bearing that
    never changes.
    """
    _, angular, done = align_cmd(ALIGN_EPS * 1.5)
    assert done is False
    assert abs(angular) >= ALIGN_YAW_MIN


def test_the_align_rate_is_clamped():
    for bearing in (-2.0, 2.0, math.pi):
        _, angular, _ = align_cmd(bearing)
        assert abs(angular) <= ALIGN_YAW_CLAMP + 1e-9


# ── creep ───────────────────────────────────────────────────────────────────
def test_the_creep_stops_at_the_distance_it_was_given():
    _, _, done = creep_cmd(0.10, 0.17, 0.0)
    assert done is False
    _, _, done = creep_cmd(0.17, 0.17, 0.0)
    assert done is True
    _, _, done = creep_cmd(0.25, 0.17, 0.0)
    assert done is True


def test_the_creep_holds_its_heading():
    """
    Without this the blind leg undoes the align it depends on.

    Residual yaw over a stretch perception cannot watch is exactly the
    error the align-then-creep design exists to avoid.
    """
    _, ang_left, _ = creep_cmd(0.0, 0.17, 0.05)
    _, ang_right, _ = creep_cmd(0.0, 0.17, -0.05)
    assert ang_left < 0.0
    assert ang_right > 0.0
    assert abs(ang_left) <= CREEP_YAW_CLAMP + 1e-9


def test_the_lead_centres_the_creeps_quantisation_error():
    """
    CREEP_LEAD turns a one-sided overshoot into a symmetric +/- half tick.

    The loop can only stop on a tick, so the robot passes its stop
    somewhere in [0, per_tick) past it, and then coasts the braking
    distance on top. CREEP_LEAD is exactly half a tick plus that braking
    distance, so stopping at (distance - lead) puts the error at
    +/- per_tick/2 about the intended pose instead of +per_tick +brake.

    That is what makes a 5.5 mm window hittable at all: the raw
    quantisation at 0.03 m/s and 20 Hz is 1.5 mm against a 2.75 mm
    half-window, which would leave nothing for anything else.
    """
    per_tick = CREEP_SPEED / APPROACH_HZ
    braking = CREEP_SPEED ** 2 / (2 * 2.0)
    assert CREEP_LEAD == pytest.approx(per_tick / 2.0 + braking, abs=1e-9)

    near, far = approach_window('yellow')
    half_window = (far - near) / 2.0
    assert per_tick / 2.0 < half_window / 2.0, (
        f'+/-{per_tick / 2.0 * 1000:.2f} mm of quantisation against a '
        f'{half_window * 1000:.2f} mm half-window leaves no room for '
        f'perception or odometry')


# ── the stop pose ───────────────────────────────────────────────────────────
@pytest.mark.parametrize('target', TARGETS, ids=lambda t: t.colour)
def test_the_stop_pose_clears_the_chassis_by_the_targets_radius(target):
    """
    Stopping too close is not a missed grasp, it is a collision.

    Clearing the chassis nose by the cylinder's radius is necessary but
    no longer the binding bound — see the next test — so this checks the
    weaker condition that must hold regardless of which bound dominates.
    """
    stop = approach_stop_x(target.colour)
    assert stop > CHASSIS_FRONT_X + target.diameter / 2.0


def test_the_stop_is_the_same_for_every_thickness():
    """
    This used to assert the opposite, and that is the interesting part.

    While the near bound was CHASSIS_FRONT_X + radius, thicker targets
    genuinely did stop further out. Then the arm's self-collision limit
    was measured at base-x 0.150 — 9 mm outside even the 32 mm target's
    chassis bound — and it dominates all four. Every colour now stops in
    the same place.

    Asserting equality rather than ordering matters: `sorted(stops)` is
    still true of four identical numbers, so the old test would have gone
    on passing while the fact it described had stopped being true.
    """
    stops = [approach_stop_x(t.colour) for t in TARGETS]
    assert len(set(stops)) == 1, (
        f'stops have diverged by colour: {stops} — if that is deliberate, '
        f'the self-collision bound is no longer dominant and '
        f'GRASP_SELF_COLLISION_X needs re-measuring')
    for target in TARGETS:
        near, _ = approach_window(target.colour)
        assert near >= GRASP_SELF_COLLISION_X


def test_the_creep_leg_is_the_same_for_every_target():
    """
    Handoff is fixed and the stop no longer moves, so the leg is one number.

    Worth pinning: if it ever grows past ~0.25 m the odometry error stops
    being negligible against the lateral budget and the handoff needs
    revisiting.
    """
    legs = [SERVO_HANDOFF_X - approach_stop_x(t.colour) for t in TARGETS]
    assert len(set(legs)) == 1
    assert max(legs) < 0.25
    assert min(legs) > 0.0


# ── plumbing ────────────────────────────────────────────────────────────────
def test_bearing_sign_matches_ros_convention():
    """+y is left, and a left bearing is a positive (counter-clockwise) yaw."""
    assert bearing_to(1.0, 1.0) == pytest.approx(math.pi / 4)
    assert bearing_to(1.0, -1.0) == pytest.approx(-math.pi / 4)
    assert bearing_to(1.0, 0.0) == 0.0


def test_heading_error_wraps_the_short_way():
    """A robot at +179 deg corrects 1 deg, not 359."""
    assert wrap(math.pi + 0.01) == pytest.approx(-math.pi + 0.01, abs=1e-9)
    assert wrap(-math.pi - 0.01) == pytest.approx(math.pi - 0.01, abs=1e-9)


def test_status_line_is_key_value_with_no_blank_fields():
    """The panel splits this on spaces; a blank value shifts every field."""
    line = format_status(phase='servo', tx=0.85, ty=-0.013, colour='blue')
    fields = dict(part.split('=', 1) for part in line.split(' '))
    assert fields['phase'] == 'servo'
    assert fields['ty'] == '-0.013'        # signed: which way it is off
    assert fields['outcome'] == '--'       # never blank
    assert '' not in fields.values()
