"""Reward/termination math (pure functions — no ROS/Gazebo needed)."""
from coco_rl.reward import (
    GOAL_BONUS,
    GOAL_X_PROGRESS,
    TIP_LIMIT,
    TIP_PENALTY,
    is_tipped,
    reached_goal,
    step_reward,
)


def test_forward_progress_is_rewarded():
    flat_forward = step_reward(0.05, 0.0, 0.0, False, False)
    flat_still = step_reward(0.0, 0.0, 0.0, False, False)
    assert flat_forward > flat_still
    assert flat_forward > 0            # real progress beats the time penalty
    assert flat_still < 0              # standing still costs


def test_backward_progress_is_punished():
    assert step_reward(-0.05, 0.0, 0.0, False, False) < \
        step_reward(0.0, 0.0, 0.0, False, False)


def test_tilt_is_penalised_symmetrically():
    upright = step_reward(0.02, 0.0, 0.0, False, False)
    assert step_reward(0.02, 0.3, 0.0, False, False) < upright
    assert step_reward(0.02, -0.3, 0.0, False, False) < upright
    assert step_reward(0.02, 0.3, 0.0, False, False) == \
        step_reward(0.02, -0.3, 0.0, False, False)


def test_terminal_bonuses():
    base = step_reward(0.0, 0.0, 0.0, False, False)
    assert step_reward(0.0, 0.0, 0.0, True, False) == base - TIP_PENALTY
    assert step_reward(0.0, 0.0, 0.0, False, True) == base + GOAL_BONUS


def test_tip_detection():
    assert not is_tipped(0.0, 0.0)
    assert not is_tipped(TIP_LIMIT - 0.01, 0.0)
    assert is_tipped(TIP_LIMIT + 0.01, 0.0)
    assert is_tipped(0.0, -(TIP_LIMIT + 0.01))


def test_goal_detection():
    assert not reached_goal(GOAL_X_PROGRESS - 0.01)
    assert reached_goal(GOAL_X_PROGRESS)
    assert reached_goal(GOAL_X_PROGRESS + 1.0)
