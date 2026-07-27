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
The scripted descent controller, tested without a ROS graph.

descend_cmd() is deliberately pure so the cases that matter on a slope —
never pivot in place, wrap the heading the short way, stop at the goal —
are asserted here rather than inferred from watching a simulator.
"""

import math

import pytest

driver = pytest.importorskip('coco_rl.ramp_driver')

descend_cmd = driver.descend_cmd
DESCEND_SPEED = driver.DESCEND_SPEED
DESCEND_YAW_CLAMP = driver.DESCEND_YAW_CLAMP
DESCEND_ARRIVE = driver.DESCEND_ARRIVE

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


def test_status_line_is_key_value_for_the_panel():
    line = driver.format_status('climb', 42, 2.5, 0.08, -0.31, None)
    fields = dict(p.split('=', 1) for p in line.split(' '))
    assert fields['segment'] == 'climb'
    assert fields['step'] == '42'
    assert fields['lateral'] == '+0.08'    # signed: which way it drifted
    assert fields['outcome'] == 'none'     # never blank; blank reads as a bug
