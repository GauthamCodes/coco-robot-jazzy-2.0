#!/usr/bin/env python3
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

"""Basic tests for teleop nodes."""


def test_import_teleop_wheels():
    """Test that teleop_wheels_node can be imported."""
    from custom_teleop import teleop_wheels_node
    assert hasattr(teleop_wheels_node, 'TeleopWheels')
    assert hasattr(teleop_wheels_node, 'main')


def test_import_teleop_arm():
    """Test that teleop_arm_node can be imported."""
    from custom_teleop import teleop_arm_node
    assert hasattr(teleop_arm_node, 'TeleopArm')
    assert hasattr(teleop_arm_node, 'main')


def test_key_bindings_derive_from_the_step_sizes():
    """
    w/s/a/d increments must follow the configured steps.

    These used to be a hardcoded dict that silently duplicated
    LINEAR_STEP/ANGULAR_STEP, so changing a step left the keys behind.
    """
    from custom_teleop.teleop_wheels_node import key_bindings
    b = key_bindings(0.25, 0.75)
    assert b['w'] == (0.25, 0.0)
    assert b['s'] == (-0.25, 0.0)
    assert b['a'] == (0.0, 0.75)
    assert b['d'] == (0.0, -0.75)


def test_default_key_bindings_match_the_default_steps():
    from custom_teleop import teleop_wheels_node as m
    assert m.KEY_BINDINGS['w'] == (m.LINEAR_STEP, 0.0)
    assert m.KEY_BINDINGS['a'] == (0.0, m.ANGULAR_STEP)


def test_opposing_keys_cancel():
    """Pressing w then s must return to a standstill, exactly."""
    from custom_teleop.teleop_wheels_node import key_bindings
    b = key_bindings(0.1, 0.2)
    lin = b['w'][0] + b['s'][0]
    ang = b['a'][1] + b['d'][1]
    assert lin == 0.0 and ang == 0.0


def test_clamp_bounds_and_passes_through():
    from custom_teleop.teleop_arm_node import TeleopArm
    clamp = TeleopArm._clamp
    assert clamp(5.0, -1.0, 1.0) == 1.0       # above range
    assert clamp(-5.0, -1.0, 1.0) == -1.0     # below range
    assert clamp(0.3, -1.0, 1.0) == 0.3       # inside range untouched
    assert clamp(1.0, -1.0, 1.0) == 1.0       # exactly at the bound


def test_arm_limits_are_ordered_and_nonempty():
    """A reversed (lo, hi) pair would make _clamp collapse to a constant."""
    from custom_teleop import teleop_arm_node as m
    for name, (lo, hi) in [('shoulder', m.SHOULDER_LIMITS),
                           ('elbow', m.ELBOW_LIMITS),
                           ('grip', m.GRIP_LIMITS)]:
        assert lo < hi, f'{name} limits are reversed: {(lo, hi)}'


def test_home_pose_is_reachable_within_limits():
    """SPACE resets to home, which must not be clamped away."""
    from custom_teleop.teleop_arm_node import (
        ELBOW_LIMITS, GRIP_LIMITS, HOME_ELBOW, HOME_GRIP, HOME_SHOULDER,
        SHOULDER_LIMITS, TeleopArm)
    clamp = TeleopArm._clamp
    assert clamp(HOME_SHOULDER, *SHOULDER_LIMITS) == HOME_SHOULDER
    assert clamp(HOME_ELBOW, *ELBOW_LIMITS) == HOME_ELBOW
    assert clamp(HOME_GRIP, *GRIP_LIMITS) == HOME_GRIP
