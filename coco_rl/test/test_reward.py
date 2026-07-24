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

"""Reward/termination math (pure functions — no ROS/Gazebo needed)."""
import pytest

from coco_rl.reward import (
    GOAL_BONUS,
    GOAL_X_PROGRESS,
    TIP_LIMIT,
    TIP_PENALTY,
    episode_outcome,
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


def test_goal_is_the_summit_not_the_foot():
    """The bug this whole change fixes: the goal used to be 3.0 m, which only
    reaches the ramp *foot* (world x=1.0 from spawn x=-2.0). Reaching the foot
    must not count as success — only the summit (default 5.5 m) does. The env
    passes the authoritative summit distance as goal_x; here we pin the
    precedence with explicit values."""
    foot_progress = 3.0        # spawn(-2.0) -> ramp foot(1.0)
    summit_progress = 5.5      # spawn(-2.0) -> ramp summit(3.5), default 18 deg
    assert not reached_goal(foot_progress, goal_x=summit_progress)
    assert reached_goal(summit_progress, goal_x=summit_progress)


def test_default_goal_matches_the_configured_summit():
    """reward.py's fallback default and coco_config's ramp geometry must agree,
    so a run that forgets to pass goal_x still targets the real summit. This is
    the drift guard called out in reward.py's GOAL_X_PROGRESS comment."""
    robot = pytest.importorskip('coco_config.robot')
    summit_progress = robot.RAMP_SUMMIT_X - robot.SPAWN_XY[0]
    assert GOAL_X_PROGRESS == pytest.approx(summit_progress)


def test_episode_outcome_classifies_each_ending():
    assert episode_outcome(tipped=False, reached=True, truncated=False) == 'goal'
    assert episode_outcome(tipped=True, reached=False, truncated=False) == 'tipped'
    assert episode_outcome(tipped=False, reached=False, truncated=True) == 'timeout'
    assert episode_outcome(tipped=False, reached=False, truncated=False) is None


def test_episode_outcome_prefers_goal_over_truncation():
    # Hitting the goal on the very last allowed step is a success, not a
    # timeout; evaluate.py's success rate depends on this precedence.
    assert episode_outcome(tipped=False, reached=True, truncated=True) == 'goal'


def test_episode_outcome_is_independent_of_reward_sign():
    # evaluate.py used to infer the outcome from the sign of the final
    # reward. Construct a tip-over that still scores positive (a big
    # forward lunge outweighing TIP_PENALTY): the sign-based rule would
    # have called this 'goal'.
    lunge = step_reward(progress=2.0, roll=0.9, pitch=0.0,
                        tipped=True, reached=False)
    assert lunge > 0, 'need a positive-reward tip-over to make the point'
    assert episode_outcome(tipped=True, reached=False,
                           truncated=False) == 'tipped'
