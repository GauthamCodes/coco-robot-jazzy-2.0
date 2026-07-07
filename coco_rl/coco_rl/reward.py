"""
reward.py
=========
Pure reward/termination math for the ramp-traversal task — no ROS, no
Gazebo, no gymnasium imports, so it is unit-testable in isolation and
the numbers live in exactly one place.
"""

TIP_LIMIT = 0.6          # rad — |roll| or |pitch| beyond this is a tip-over
GOAL_X_PROGRESS = 3.0    # m of forward progress that completes the episode

PROGRESS_GAIN = 10.0     # reward per metre of forward progress
TILT_PENALTY = 0.05      # per rad of |roll| + |pitch|, per step
TIME_PENALTY = 0.01      # per step, pushes for faster completion
TIP_PENALTY = 10.0       # terminal penalty for falling over
GOAL_BONUS = 20.0        # terminal bonus for reaching the top


def is_tipped(roll, pitch, tip_limit=TIP_LIMIT):
    return abs(roll) > tip_limit or abs(pitch) > tip_limit


def reached_goal(x_progress, goal_x=GOAL_X_PROGRESS):
    return x_progress >= goal_x


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
