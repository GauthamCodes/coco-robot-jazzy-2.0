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
reward.py
=========
Pure reward/termination math for the ramp-traversal task — no ROS, no
Gazebo, no gymnasium imports, so it is unit-testable in isolation and
the numbers live in exactly one place.
"""

TIP_LIMIT = 0.6          # rad — |roll| or |pitch| beyond this is a tip-over

# Forward progress (m, from spawn) that completes the episode. This is the
# *summit* of the ramp, not just the foot: for the default 18-deg wedge the
# crest is at world x=3.0 and the robot spawns at x=-2.0, so 5.0 m. It is only
# a fallback default and a piece of documentation — CocoRampEnv passes the
# authoritative value (RAMP_SUMMIT_X - spawn x, from coco_config.robot) into
# reached_goal(), so the two cannot drift if the ramp geometry changes. Keeping
# reward.py import-free (no coco_config dependency) is why the value is repeated
# here rather than imported.
GOAL_X_PROGRESS = 5.0    # m — reaching the ramp summit, not the foot (was 3.0)

PROGRESS_GAIN = 10.0     # reward per metre of forward progress
TILT_PENALTY = 0.05      # per rad of |roll| + |pitch|, per step
TIME_PENALTY = 0.01      # per step, pushes for faster completion
TIP_PENALTY = 10.0       # terminal penalty for falling over
GOAL_BONUS = 20.0        # terminal bonus for reaching the top


def is_tipped(roll, pitch, tip_limit=TIP_LIMIT):
    return abs(roll) > tip_limit or abs(pitch) > tip_limit


def reached_goal(x_progress, goal_x=GOAL_X_PROGRESS):
    return x_progress >= goal_x


def episode_outcome(tipped, reached, truncated):
    """Classify how an episode ended: 'goal', 'tipped', 'timeout' or None.

    Lives here so the env can report the outcome as fact in step()'s info
    dict. evaluate.py used to infer it from the sign of the final reward,
    which silently became wrong the moment GOAL_BONUS or TIP_PENALTY was
    retuned.
    """
    if reached:
        return 'goal'
    if tipped:
        return 'tipped'
    if truncated:
        return 'timeout'
    return None


def step_reward(progress, roll, pitch, tipped, reached):
    """Reward for one control step.

    progress : forward distance gained this step (m)
    roll/pitch : current attitude (rad)
    tipped/reached : terminal flags for this step
    """
    reward = (PROGRESS_GAIN * progress
              - TILT_PENALTY * (abs(roll) + abs(pitch))
              - TIME_PENALTY)
    if tipped:
        reward -= TIP_PENALTY
    if reached:
        reward += GOAL_BONUS
    return float(reward)
