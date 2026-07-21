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

"""Waypoint steering law used by the SLAM mapping drive.

This package previously had no tests at all, so map_drive.py and
verify_sim.py were neither tested nor linted. The steering law is worth
covering because its failure mode is subtle: a missing angle wrap sends
the robot the long way round, which does not crash, it just produces a
bad map after a ten-minute drive.
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from map_drive import (ARRIVE, FWD, HEADING_GATE,  # noqa: E402
                       TURN_MAX, steer)


def test_straight_ahead_drives_without_steering():
    lin, ang, dist = steer(0.0, 0.0, 0.0, 1.0, 0.0)
    assert lin == FWD
    assert ang == pytest.approx(0.0, abs=1e-12)
    assert dist == pytest.approx(1.0)


def test_distance_is_euclidean_and_yaw_independent():
    for yaw in (0.0, 1.0, -2.5, math.pi):
        _, _, dist = steer(0.0, 0.0, yaw, 3.0, 4.0)
        assert dist == pytest.approx(5.0)


def test_target_to_the_left_turns_positive():
    """REP-103: +z angular velocity is counter-clockwise."""
    lin, ang, _ = steer(0.0, 0.0, 0.0, 0.0, 1.0)
    assert ang > 0
    assert lin == 0.0, 'a 90 deg error should turn in place, not drive'


def test_target_to_the_right_turns_negative():
    _, ang, _ = steer(0.0, 0.0, 0.0, 0.0, -1.0)
    assert ang < 0


def test_heading_error_wraps_the_short_way():
    """The bug this function exists to prevent.

    Facing just under +pi with the target just over -pi is a ~2 deg error,
    not a ~358 deg one. Without the atan2(sin, cos) wrap the robot would
    spin almost all the way round.
    """
    yaw = math.pi - 0.02
    # target lies at bearing -(pi - 0.02): 0.04 rad away, across the +/-pi cut
    tx, ty = math.cos(-(math.pi - 0.02)), math.sin(-(math.pi - 0.02))
    _, ang, _ = steer(0.0, 0.0, yaw, tx, ty)
    # Crossing the cut counter-clockwise is the short way here, so the
    # correction is small and positive. Unwrapped, the error would read
    # -6.24 rad and drive a hard clamped turn the wrong way round.
    assert ang > 0
    assert abs(ang) == pytest.approx(0.8 * 0.04, abs=1e-9)
    assert abs(ang) < TURN_MAX, 'a 2 deg error must not saturate the clamp'


def test_directly_behind_turns_at_the_clamp_and_does_not_drive():
    lin, ang, _ = steer(0.0, 0.0, 0.0, -1.0, 0.0)
    assert lin == 0.0
    assert abs(ang) == pytest.approx(TURN_MAX)


def test_angular_velocity_is_clamped():
    for yaw in [i * math.pi / 8 for i in range(-8, 9)]:
        _, ang, _ = steer(0.0, 0.0, yaw, 1.0, 0.0)
        assert abs(ang) <= TURN_MAX + 1e-12


def test_forward_motion_gated_on_heading_error():
    """Fast turns break scan matching, so drive only once roughly aimed."""
    just_inside = HEADING_GATE - 0.05
    just_outside = HEADING_GATE + 0.05
    # place the target at a known bearing by rotating the robot instead
    lin_in, _, _ = steer(0.0, 0.0, -just_inside, 1.0, 0.0)
    lin_out, _, _ = steer(0.0, 0.0, -just_outside, 1.0, 0.0)
    assert lin_in == FWD
    assert lin_out == 0.0


def test_arrive_radius_is_smaller_than_one_cruise_step():
    """A capture radius below the per-tick travel would be un-hittable."""
    assert 0 < ARRIVE
    assert ARRIVE > FWD * 0.05, 'ARRIVE must exceed the distance covered per tick'
