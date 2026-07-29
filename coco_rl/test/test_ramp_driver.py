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
The scripted descent controller and the climb's lane hold, without ROS.

Both are deliberately pure so the cases that matter on a slope — never
pivot in place, wrap the heading the short way, stop at the goal, and
never hand the policy an action outside its own action space — are
asserted here rather than inferred from watching a simulator.
"""

import math

import pytest

driver = pytest.importorskip('coco_rl.ramp_driver')

descend_cmd = driver.descend_cmd
DESCEND_SPEED = driver.DESCEND_SPEED
DESCEND_YAW_CLAMP = driver.DESCEND_YAW_CLAMP
DESCEND_ARRIVE = driver.DESCEND_ARRIVE

lateral_hold = driver.lateral_hold
LATERAL_CLAMP = driver.LATERAL_CLAMP

GOAL = 6.8


def test_straight_and_far_drives_at_speed_with_no_steering():
    lin, ang, done = descend_cmd(0.0, 5.0, GOAL)
    assert lin == pytest.approx(DESCEND_SPEED)
    assert ang == pytest.approx(0.0)
    assert done is False


def test_arriving_stops_and_reports_done():
    lin, ang, done = descend_cmd(0.0, GOAL - DESCEND_ARRIVE / 2, GOAL)
    assert (lin, ang) == (0.0, 0.0)
    assert done is True


def test_overshooting_the_goal_still_reports_done():
    """Past the goal is arrived, not 'drive backwards to reach it'."""
    _, _, done = descend_cmd(0.0, GOAL + 1.0, GOAL)
    assert done is True


def test_never_pivots_in_place_on_the_slope():
    """
    The whole reason this is not map_drive.steer().

    steer() zeroes its linear term and turns in place above a heading
    gate. On a downslope the robot is being accelerated by gravity and a
    stationary skid-steer pivot on a grade is how it loses its footing.
    Forward speed here must not depend on heading error at all.
    """
    for yaw in (-1.5, -0.6, -0.2, 0.0, 0.2, 0.6, 1.5):
        lin, _, _ = descend_cmd(yaw, 5.0, GOAL)
        assert lin == pytest.approx(DESCEND_SPEED)


def test_steers_back_toward_zero_heading():
    """Yaw drift left is corrected right, and vice versa."""
    _, ang_left, _ = descend_cmd(0.2, 5.0, GOAL)
    _, ang_right, _ = descend_cmd(-0.2, 5.0, GOAL)
    assert ang_left < 0.0
    assert ang_right > 0.0
    assert ang_left == pytest.approx(-ang_right)


def test_correction_is_clamped():
    """A large heading error must not command an unbounded spin."""
    for yaw in (1.0, 2.0, 3.0, -1.0, -3.0):
        _, ang, _ = descend_cmd(yaw, 5.0, GOAL)
        assert abs(ang) <= DESCEND_YAW_CLAMP + 1e-9


def test_heading_wraps_the_short_way():
    """
    A robot at +179 deg must correct 1 deg, not 359.

    Without the atan2 wrap this is the bug that makes a robot spin most of
    a full turn to fix a hair of error.
    """
    _, ang_just_under, _ = descend_cmd(math.pi - 0.01, 5.0, GOAL)
    _, ang_just_over, _ = descend_cmd(-math.pi + 0.01, 5.0, GOAL)
    # Both are ~180 deg off; the corrections must be opposite and clamped,
    # never a value implying a 359 deg journey.
    assert abs(ang_just_under) <= DESCEND_YAW_CLAMP + 1e-9
    assert abs(ang_just_over) <= DESCEND_YAW_CLAMP + 1e-9


# ── lane hold ──────────────────────────────────────────────────────────────

def test_on_the_centreline_the_policy_is_untouched():
    """
    No error, no correction.

    This is what makes the 10/10 climb still the 10/10 climb: a robot
    that never leaves its lane runs exactly the actions the policy asked
    for, and the lane hold only exists in the failure case.
    """
    assert lateral_hold([0.7, -0.3], y_err=0.0, yaw=0.0) == [0.7, -0.3]


def test_the_linear_channel_is_never_touched():
    """
    Speed stays the policy's business.

    Slowing down mid-climb is how a skid-steer base loses traction on a
    grade, and nothing here knows enough about the slope to make that
    call.
    """
    for y_err, yaw in ((0.6, 0.0), (-0.6, 0.0), (0.0, 0.4), (0.5, -0.4)):
        assert lateral_hold([0.42, 0.0], y_err, yaw)[0] == pytest.approx(0.42)


def test_drift_left_steers_right_and_vice_versa():
    """The measured failure is +0.6 m, so the sign of this is the fix."""
    assert lateral_hold([0.0, 0.0], y_err=0.3, yaw=0.0)[1] < 0.0
    assert lateral_hold([0.0, 0.0], y_err=-0.3, yaw=0.0)[1] > 0.0


def test_heading_error_alone_is_corrected():
    """
    The damping term, and the one that actually stops the drift.

    A constant yaw bias is what turns 0 m of error into 0.6 m over a 2.5 m
    climb; correcting position alone would only chase it.
    """
    assert lateral_hold([0.0, 0.0], y_err=0.0, yaw=0.2)[1] < 0.0
    assert lateral_hold([0.0, 0.0], y_err=0.0, yaw=-0.2)[1] > 0.0


def test_correction_is_clamped_away_from_the_trained_distribution():
    """
    A big error must not hand the policy's action to a bang-bang loop.

    The clamp is the whole argument that this does not need retraining:
    the action stays within LATERAL_CLAMP of what the policy asked for, so
    the perturbation is bounded and measurable rather than open-ended.
    """
    for y_err, yaw in ((5.0, 3.0), (-5.0, -3.0), (2.0, -3.0)):
        held = lateral_hold([0.5, 0.1], y_err, yaw)
        assert abs(held[1] - 0.1) <= LATERAL_CLAMP + 1e-9


def test_a_saturated_policy_action_stays_inside_the_action_space():
    """
    Saturation must not push the action out of bounds.

    ramp_env clips to [-1, 1] anyway, so returning outside it would only
    make the logged correction a lie about what the robot was commanded.
    """
    for action in ([0.0, 1.0], [0.0, -1.0]):
        for y_err in (2.0, -2.0):
            held = lateral_hold(action, y_err, yaw=0.0)
            assert -1.0 <= held[1] <= 1.0


def test_status_line_is_key_value_for_the_panel():
    line = driver.format_status('climb', 42, 2.5, 0.08, -0.31, None)
    fields = dict(p.split('=', 1) for p in line.split(' '))
    assert fields['segment'] == 'climb'
    assert fields['step'] == '42'
    assert fields['lateral'] == '+0.08'    # signed: which way it drifted
    assert fields['outcome'] == 'none'     # never blank; blank reads as a bug
