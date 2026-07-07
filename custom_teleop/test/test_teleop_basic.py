#!/usr/bin/env python3
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


def test_constants_wheels():
    """Test that wheel teleop constants are defined."""
    from custom_teleop import teleop_wheels_node
    assert teleop_wheels_node.LINEAR_STEP > 0
    assert teleop_wheels_node.MAX_LINEAR > 0
    assert teleop_wheels_node.TIMEOUT_SECONDS > 0


def test_constants_arm():
    """Test that arm teleop constants are defined."""
    from custom_teleop import teleop_arm_node
    assert teleop_arm_node.STEP > 0
    assert len(teleop_arm_node.SHOULDER_LIMITS) == 2
    assert len(teleop_arm_node.ELBOW_LIMITS) == 2
