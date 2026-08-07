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
Gymnasium environment for the Coco base in MuJoCo, on a flat plane.

**This module must never import rclpy**, directly or transitively.
``coco_rl/test/test_mujoco_env_has_no_ros.py`` asserts it, and the point
is structural rather than stylistic. v1's worst bug was ``--fast``:
unlocking Gazebo's real-time factor made sim time outrun wall-clock ROS
delivery, ``cmd_vel`` arrived late, ``diff_drive_controller``'s 0.5 s
watchdog repeatedly halted the wheels, and the chassis reared over
backwards — 531 of 533 episodes tipped, evaluation scored 0/10, and it was
*slower* than not using the flag. Every link in that chain is a ROS
transport link. An environment with no ROS in it cannot have that class of
bug at all, rather than merely avoiding it by discipline.

Parity with ``ramp_env`` is deliberate and exact, so a policy or a
measurement can move between them:

===============  ===============================================
action           ``Box(-1, 1, (2,))`` -> (linear, angular)
observation      ``[x, y, sin yaw, cos yaw, v, w, roll, pitch]``
x, y             displacement from the episode origin, as ramp_env
control period   ``STEP_DT`` = 0.1 s of sim time
speed limits     ``MAX_LIN`` 0.4 m/s, ``MAX_ANG`` 0.5 rad/s
===============  ===============================================

The reward here is deliberately minimal — forward progress only. M7's
reward (cross-track, heading, slip, action rate; M7_DESIGN 4.3) is Phase 2
work, and inventing half of it now would make the throughput number
incomparable with the v1 figure it is supposed to be measured against.
"""

import math

from coco_sim.mjcf import CONTROL_DT, build_mjcf

import gymnasium as gym

import mujoco

import numpy as np

# Same limits as ramp_env, so an action means the same thing in both.
MAX_LIN, MAX_ANG = 0.4, 0.5
STEP_DT = CONTROL_DT
MAX_STEPS = 400

# Tip-over terminator, matching ramp_env's 0.6 rad.
TIP_LIMIT = 0.6


def quat_to_rpy(w, x, y, z):
    """Roll, pitch, yaw from a MuJoCo (w, x, y, z) quaternion."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def wheel_speeds(linear, angular, radius, separation):
    """Differential drive inverse kinematics, in rad/s per side.

    Pure and separately testable, which is the same reason ramp_env keeps
    its reward maths out of the env class.
    """
    left = (linear - angular * separation / 2.0) / radius
    right = (linear + angular * separation / 2.0) / radius
    return left, right


class CocoMujocoEnv(gym.Env):
    """The Coco base on a flat plane, stepped in MuJoCo. No ROS."""

    metadata = {'render_modes': []}

    def __init__(self, max_steps=MAX_STEPS, seed=None):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_string(build_mjcf())
        self.data = mujoco.MjData(self.model)
        self._substeps = max(1, int(round(STEP_DT / self.model.opt.timestep)))
        self.max_steps = max_steps
        self._steps = 0
        self._origin = (0.0, 0.0)

        self.action_space = gym.spaces.Box(
            -1.0, 1.0, shape=(2,), dtype=np.float32)
        high = np.array([np.inf] * 8, dtype=np.float32)
        self.observation_space = gym.spaces.Box(-high, high, dtype=np.float32)

        # Actuator order follows coco_sim.mjcf.wheel_positions():
        # front_right, rear_right, front_left, rear_left.
        self._right = [0, 1]
        self._left = [2, 3]

        from coco_config.robot import (
            WHEEL_RADIUS, WHEEL_SEPARATION, WHEEL_SEPARATION_MULTIPLIER)
        self._radius = WHEEL_RADIUS
        # The deployed diff_drive_controller computes wheel speeds from
        # separation * multiplier, not from the physical track — the
        # multiplier exists to compensate skid-steer yaw loss and the
        # xacro says so. A policy commanding (linear, angular) through
        # cmd_vel gets that conversion in Gazebo, so training must apply
        # it too or the same action means two different things. Parity
        # with the deployment target, not a tuning knob.
        self._separation = WHEEL_SEPARATION * WHEEL_SEPARATION_MULTIPLIER

        self._rng = np.random.default_rng(seed)

    # ── state ────────────────────────────────────────────────────────────
    def _state(self):
        px, py, _pz = self.data.qpos[0:3]
        qw, qx, qy, qz = self.data.qpos[3:7]
        roll, pitch, yaw = quat_to_rpy(qw, qx, qy, qz)
        vx, vy, _vz = self.data.qvel[0:3]
        wz = self.data.qvel[5]
        # Forward speed in the body frame, matching what odometry reports.
        v = vx * math.cos(yaw) + vy * math.sin(yaw)
        return np.array([
            px - self._origin[0], py - self._origin[1],
            math.sin(yaw), math.cos(yaw), v, wz, roll, pitch,
        ], dtype=np.float32)

    # ── gym API ──────────────────────────────────────────────────────────
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self._steps = 0
        self._origin = (float(self.data.qpos[0]), float(self.data.qpos[1]))
        return self._state(), {}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        left, right = wheel_speeds(
            float(action[0]) * MAX_LIN, float(action[1]) * MAX_ANG,
            self._radius, self._separation)
        for i in self._left:
            self.data.ctrl[i] = left
        for i in self._right:
            self.data.ctrl[i] = right

        before = float(self.data.qpos[0])
        for _ in range(self._substeps):
            mujoco.mj_step(self.model, self.data)
        self._steps += 1

        obs = self._state()
        progress = float(self.data.qpos[0]) - before
        tipped = abs(obs[6]) > TIP_LIMIT or abs(obs[7]) > TIP_LIMIT

        reward = progress - (10.0 if tipped else 0.0)
        terminated = bool(tipped)
        truncated = self._steps >= self.max_steps
        outcome = 'tipped' if tipped else (
            'timeout' if truncated else 'running')
        return obs, reward, terminated, truncated, {'outcome': outcome}

    def close(self):
        pass
