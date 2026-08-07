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

"""The MuJoCo env must be shape-compatible with ramp_env.

A policy trained against one and evaluated against the other is the whole
point of M7's sim-to-sim transfer table, and a silent shape or convention
mismatch would make that comparison meaningless rather than wrong-looking.
"""
import math

import numpy as np

import pytest

mujoco = pytest.importorskip('mujoco')

from coco_rl.mujoco_env import (  # noqa: E402
    CocoMujocoEnv, MAX_ANG, MAX_LIN, quat_to_rpy, wheel_speeds)


def test_spaces_match_ramp_env():
    env = CocoMujocoEnv()
    assert env.action_space.shape == (2,)
    assert env.observation_space.shape == (8,)
    assert float(env.action_space.low[0]) == -1.0
    assert float(env.action_space.high[0]) == 1.0


def test_reset_returns_an_observation_of_the_right_shape():
    env = CocoMujocoEnv()
    obs, info = env.reset(seed=0)
    assert obs.shape == (8,)
    assert isinstance(info, dict)


def test_x_and_y_are_relative_to_the_episode_origin():
    """ramp_env returns displacement, not world position. Matching it is
    what lets `disp` mean the same thing in both."""
    env = CocoMujocoEnv()
    obs, _ = env.reset(seed=0)
    assert obs[0] == pytest.approx(0.0, abs=1e-6)
    assert obs[1] == pytest.approx(0.0, abs=1e-6)


def test_driving_forward_moves_forward():
    """The sign convention has to be right or every reward is inverted."""
    env = CocoMujocoEnv()
    env.reset(seed=0)
    for _ in range(20):
        obs, _, _, _, _ = env.step(np.array([1.0, 0.0], dtype=np.float32))
    assert obs[0] > 0.05, f'commanded full forward, x moved {obs[0]:.4f} m'


def test_yaw_sign_is_positive_to_the_left():
    """Matches ramp_env and lateral_hold, which both take +yaw as left."""
    env = CocoMujocoEnv()
    env.reset(seed=0)
    for _ in range(20):
        obs, _, _, _, _ = env.step(np.array([0.0, 1.0], dtype=np.float32))
    yaw = math.atan2(float(obs[2]), float(obs[3]))
    assert yaw > 0.0, f'commanded +angular, yaw went {yaw:+.4f}'


def test_actions_are_clipped_not_trusted():
    env = CocoMujocoEnv()
    env.reset(seed=0)
    env.step(np.array([50.0, -50.0], dtype=np.float32))   # must not explode
    obs = env._state()
    assert np.all(np.isfinite(obs))


def test_truncates_at_max_steps():
    env = CocoMujocoEnv(max_steps=3)
    env.reset(seed=0)
    outcomes = [env.step(np.zeros(2, dtype=np.float32))[3] for _ in range(3)]
    assert outcomes[-1] is True


def test_wheel_speeds_is_differential_drive():
    """Straight means both sides equal; turning means they differ."""
    left, right = wheel_speeds(0.4, 0.0, 0.0585, 0.274)
    assert left == pytest.approx(right)
    left, right = wheel_speeds(0.0, 0.5, 0.0585, 0.274)
    assert left == pytest.approx(-right)


def test_quat_to_rpy_round_trips_level():
    roll, pitch, yaw = quat_to_rpy(1.0, 0.0, 0.0, 0.0)
    assert (roll, pitch, yaw) == pytest.approx((0.0, 0.0, 0.0))


def test_speed_limits_match_ramp_env():
    assert (MAX_LIN, MAX_ANG) == (0.4, 0.5)


# ── yaw-gain randomisation (M7_DESIGN §2.5) ──────────────────────────────
# These exist because the range was written into the design doc a phase
# before it was implemented, and a range in a doc trains nothing. Each of
# these would have failed against that version.

def test_yaw_gain_is_sampled_per_episode():
    """Different episodes must see different steering authority."""
    env = CocoMujocoEnv(seed=0)
    gains = set()
    for s in range(12):
        env.reset(seed=s)
        gains.add(round(env.yaw_gain, 6))
    assert len(gains) > 1, (
        'yaw_gain never changed across 12 resets — it is not being sampled')


def test_yaw_gain_stays_within_the_documented_range():
    from coco_rl.mujoco_env import YAW_GAIN_RANGE
    lo, hi = YAW_GAIN_RANGE
    assert (lo, hi) == (0.70, 1.45), 'range drifted from M7_DESIGN §2.5'
    env = CocoMujocoEnv(seed=1)
    for s in range(40):
        env.reset(seed=s)
        assert lo <= env.yaw_gain <= hi


def test_yaw_gain_is_constant_within_an_episode():
    """Steering authority is a property of the machine and the ground.

    Resampling per step would be noise, which teaches a policy nothing.
    """
    env = CocoMujocoEnv(seed=3)
    env.reset(seed=3)
    first = env.yaw_gain
    for _ in range(15):
        env.step(np.array([1.0, 0.5], dtype=np.float32))
        assert env.yaw_gain == first


def test_yaw_gain_actually_changes_the_achieved_yaw():
    """The load-bearing one: sampled AND applied.

    A gain stored on the object but never reaching the wheels would pass
    every test above and change nothing about the robot.
    """
    def turn_with(gain):
        env = CocoMujocoEnv(seed=0, randomize_yaw_gain=False)
        env.reset(seed=0)
        env.yaw_gain = gain
        for _ in range(30):
            obs, _, _, _, _ = env.step(
                np.array([1.0, 1.0], dtype=np.float32))
        return math.atan2(float(obs[2]), float(obs[3]))

    low, high = turn_with(0.70), turn_with(1.45)
    assert high > low * 1.2, (
        f'yaw_gain is not reaching the wheels: gain 0.70 turned {low:.4f} '
        f'rad, gain 1.45 turned {high:.4f} rad')


def test_yaw_gain_is_reproducible_from_a_seed():
    """Randomisation that cannot be replayed is not an experiment."""
    a = CocoMujocoEnv(seed=7)
    a.reset(seed=7)
    b = CocoMujocoEnv(seed=7)
    b.reset(seed=7)
    assert a.yaw_gain == b.yaw_gain


def test_randomisation_can_be_turned_off():
    """Evaluation and the sim-to-sim comparison need a fixed gain."""
    env = CocoMujocoEnv(seed=0, randomize_yaw_gain=False)
    for s in range(5):
        env.reset(seed=s)
        assert env.yaw_gain == 1.0


def test_separation_comes_from_coco_config_not_a_local_copy():
    """Rule 3: one source of truth.

    If someone pastes 1.10 into the env, this fails — the env's value must
    track coco_config, because the multiplier is the deployed
    controller's, not the env's.
    """
    from coco_config.robot import (
        WHEEL_SEPARATION, WHEEL_SEPARATION_MULTIPLIER)
    env = CocoMujocoEnv()
    assert env._separation == pytest.approx(
        WHEEL_SEPARATION * WHEEL_SEPARATION_MULTIPLIER)
