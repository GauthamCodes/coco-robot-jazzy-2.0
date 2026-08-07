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
