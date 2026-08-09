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
The Yard as a Gymnasium environment. **Never imports rclpy** — same rule
and same reason as ``mujoco_env``: v1's worst bug (``--fast``, 531/533
episodes tipped) was a chain of ROS transport links, and an environment
with no ROS in it cannot have that class of bug at all.

Everything in M7_DESIGN §2.5 is randomised here, per episode, from ONE
seeded generator. Reproducibility from a seed alone is not a nicety: a
training run that cannot be replayed cannot be debugged, and the one
thing guaranteed about a randomised course is that something will fail on
episode 40,000 and need to be looked at.

Applied IN PLACE, never by recompiling
--------------------------------------
Grade jitter and camber change ramp box geometry, which naively means a
new MJCF and a new ``mj_compile`` per episode. Compilation is ~milliseconds
against a ~27 ms episode budget at 400 steps, so it would dominate.
Instead the model is compiled once and the affected ``mjModel`` fields are
overwritten per episode:

    geom_pos / geom_quat / geom_size   ramp grade and camber
    hfield_data                        the rubble seed
    pair_friction                      per-route mu
    body_mass / body_ipos              payload and its CoG offset
    actuator_gainprm                   wheel torque scale

``test_yard_env.py`` asserts that each of these actually MOVES the
simulation — the same "is the lever connected?" check that
``coco_sim.sweep`` exists for, after a solver parameter was found in Phase
2 to have been silently disconnected for an entire calibration while
every number in the source still read as applied.

What is NOT randomised, and why
-------------------------------
IMU noise. §2.5 asks for "σ from measured Gazebo values" and there are
none: ``coco_robo2.xacro``'s ``<sensor name="imu">`` declares no
``<noise>`` element at all, so the deployed sensor is noiseless and there
is nothing to match. Left at zero and recorded as unmeasured rather than
filled with a plausible-looking default.
"""

import math

from coco_sim.mjcf import CONTROL_DT, build_mjcf  # noqa: F401
from coco_sim.yard import (apply_hfields, build_yard_mjcf, features,
                           load_params, sample_yard)

import gymnasium as gym

import mujoco

import numpy as np

# Parity with ramp_env and mujoco_env, so an action means the same thing
# in all three and a measurement can move between them.
MAX_LIN, MAX_ANG = 0.4, 0.5
STEP_DT = CONTROL_DT
MAX_STEPS = 600
TIP_LIMIT = 0.6

ROUTE_KEYS = ('a', 'b', 'c')


class CocoYardEnv(gym.Env):
    """The Coco base on one Yard route. No ROS, no /clock, no DDS."""

    metadata = {'render_modes': []}

    def __init__(self, route='b', max_steps=MAX_STEPS, seed=None,
                 randomise=True, params=None):
        super().__init__()
        if route not in ROUTE_KEYS:
            raise ValueError(f'route must be one of {ROUTE_KEYS}')
        self.route = route
        self.randomise = randomise
        self.max_steps = max_steps
        self.params = params or load_params()

        self._episode = 0
        self._seed0 = 0 if seed is None else int(seed)

        # Compile ONCE, on a nominal sample. Every later episode edits
        # this model in place.
        self.sample = sample_yard(self.params, seed=self._seed0,
                                  randomise=False)
        xml, self._fields = build_yard_mjcf(self.sample)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        apply_hfields(self.model, self._fields)

        self._substeps = max(1, int(round(STEP_DT / self.model.opt.timestep)))
        self._steps = 0
        self.yaw_gain = 1.0

        self.action_space = gym.spaces.Box(
            -1.0, 1.0, shape=(2,), dtype=np.float32)
        high = np.array([np.inf] * 8, dtype=np.float32)
        self.observation_space = gym.spaces.Box(-high, high, dtype=np.float32)

        from coco_config.robot import (
            CHASSIS_MASS, MAX_ANGULAR_ACCEL, MAX_LINEAR_ACCEL, WHEEL_RADIUS,
            WHEEL_SEPARATION, WHEEL_SEPARATION_MULTIPLIER)
        self._radius = WHEEL_RADIUS
        self._separation = WHEEL_SEPARATION * WHEEL_SEPARATION_MULTIPLIER
        self._chassis_mass = CHASSIS_MASS
        # Parity with diff_drive_controller, which RAMPS a command rather
        # than stepping to it. Without this the velocity servos deliver an
        # unbounded torque spike on the first tick and the robot wheelies
        # off grippy ground -- see coco_config.MAX_LINEAR_ACCEL.
        self._dv = MAX_LINEAR_ACCEL * STEP_DT
        self._dw = MAX_ANGULAR_ACCEL * STEP_DT
        self._v_cmd = 0.0
        self._w_cmd = 0.0

        # Measured ONCE, by settling on the flat apron: the height at
        # which the wheels rest under this contact model. Spawning here
        # rather than 2 mm clear removes the drop transient entirely,
        # which matters because the calibrated contact needs SECONDS to
        # converge -- 0.4 s of settling left the robot lower (0.0424) than
        # its true rest height (0.0577), i.e. still moving.
        self._rest_z = None
        self._right = [0, 1]
        self._left = [2, 3]
        self._base = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'base')
        self._gain0 = self.model.actuator_gainprm.copy()
        self._bias0 = self.model.actuator_biasprm.copy()
        self._mass0 = self.model.body_mass.copy()
        self._ipos0 = self.model.body_ipos.copy()
        self._rest_z = self._measure_rest_z()

    def _measure_rest_z(self):
        """Settle the bare robot on the flat apron and return its height."""
        mujoco.mj_resetData(self.model, self.data)
        from coco_config.robot import WHEEL_RADIUS
        self.data.qpos[0:3] = [-4.0, 0.0, WHEEL_RADIUS]
        mujoco.mj_forward(self.model, self.data)
        for _ in range(int(4.0 / self.model.opt.timestep)):
            self.data.ctrl[:] = 0.0
            mujoco.mj_step(self.model, self.data)
        return float(self.data.qpos[2])

    # ── applying a sample to the compiled model ──────────────────────────
    def _apply(self, s):
        """Push a YardSample into mjModel. No recompile."""
        boxes, fields = features(s)
        for b in boxes:
            gid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, b.name)
            if gid < 0:
                continue
            self.model.geom_pos[gid] = b.pos
            self.model.geom_quat[gid] = b.quat
            self.model.geom_size[gid] = b.half
        apply_hfields(self.model, fields)

        # friction: set on the PAIR, which is what actually applies. The
        # geoms' own values are combined by MuJoCo as an elementwise max
        # and would make every mu below the wheel's 0.4 a silent no-op.
        names = {b.name: b.mu for b in boxes}
        names.update({f.name: f.mu for f in fields})
        for pid in range(self.model.npair):
            g1 = self.model.pair_geom1[pid]
            nm = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, int(g1))
            if nm in names:
                mu = names[nm]
                self.model.pair_friction[pid][0] = mu
                self.model.pair_friction[pid][1] = mu

        # payload: mass added to the chassis, with a CoG offset. The
        # descent is taken CARRYING the object, so the tipping margin is
        # a function of this and never of the bare robot.
        self.model.body_mass[self._base] = (
            self._mass0[self._base] + s.payload_mass)
        ip = self._ipos0[self._base].copy()
        if s.payload_mass > 0:
            total = self._mass0[self._base] + s.payload_mass
            ip[0] += s.payload_cog * s.payload_mass / total
        self.model.body_ipos[self._base] = ip

        # Actuator authority. BOTH gainprm and biasprm are scaled.
        #
        # A MuJoCo velocity servo computes force = kv*ctrl - kv*vel, with
        # kv in gainprm[0] and -kv in biasprm[2]. Scaling only gainprm
        # leaves the damping term at the original kv, so the steady state
        # becomes vel = scale*ctrl -- a SPEED scale, not a torque scale,
        # and one that silently contradicts the §2.5 row it implements.
        # Scaling both gives scale*kv*(ctrl - vel): the servo tracks the
        # same target with more or less authority, which is what
        # "wheel torque scale 0.85-1.15" means.
        self.model.actuator_gainprm[:] = self._gain0 * s.torque_scale
        self.model.actuator_biasprm[:] = self._bias0 * s.torque_scale

    def _spawn(self, s):
        """Start pose: at the foot of this episode's route, facing uphill."""
        route = s.routes[self.route]
        x = route.x_foot - 0.45
        y = route.y_centre
        from coco_config.robot import WHEEL_RADIUS
        # EXACTLY the wheel radius: zero penetration and zero drop.
        #
        # Both alternatives were measured and both are unstable. Spawning
        # 2 mm clear makes the robot still be descending when the first
        # command lands, because this contact needs seconds to settle, and
        # the command then acts on a deeply embedded wheel. Spawning at the
        # SETTLED height (0.81 mm of penetration) is worse: mj_forward on
        # an already-penetrated state answers the first actuator torque
        # with an impulse that throws the robot 85 mm into the air and
        # loses every contact within 12 ms.
        #
        # Zero penetration is also what the flat env does -- mj_resetData
        # puts qpos at the body's declared height -- which is why that env
        # never showed this and this one did.
        self.data.qpos[0:3] = [x, y, WHEEL_RADIUS]
        yaw = s.initial_yaw
        self.data.qpos[3:7] = [math.cos(yaw / 2), 0.0, 0.0,
                               math.sin(yaw / 2)]

    def _settle(self):
        """Let the robot come to rest before the episode starts.

        Not cosmetic. The calibrated contact has a 0.25 s time constant --
        twelve times softer than MuJoCo's default, and that softness is
        the strong lever the Phase 1.5 yaw calibration was bought with. A
        robot spawned 2 mm clear is therefore still DESCENDING 0.1 s
        later: it overshoots to 11.8 mm of penetration against a static
        sink of 0.81 mm. Issue the first command into that state and the
        tangential force acts on a deeply embedded wheel, and the solver
        ejects the robot -- pitch went from -0.4 deg to +49.5 deg in one
        control step and every route failed at its foot.

        The bug was invisible until acceleration limiting was added,
        because a command held constant from t=0 never steps into the
        transient. That is the dangerous kind: a real defect that a less
        faithful model happens to hide.

        A real robot is already standing when it is told to move. So is
        this one now.
        """
        for _ in range(int(0.5 / self.model.opt.timestep)):
            self.data.ctrl[:] = 0.0
            mujoco.mj_step(self.model, self.data)
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    # ── gym API ──────────────────────────────────────────────────────────
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self._seed0 = int(seed)
            self._episode = 0
        # One seed per episode, derived deterministically from the env
        # seed and the episode index. Given (seed, episode) the entire
        # course, the payload, the actuator gain and the start pose are
        # reproducible without replaying anything in between.
        ep_seed = (self._seed0 * 1_000_003 + self._episode) % (2 ** 31)
        self.sample = sample_yard(self.params, seed=ep_seed,
                                  randomise=self.randomise)
        self._episode += 1

        mujoco.mj_resetData(self.model, self.data)
        self._apply(self.sample)
        self._spawn(self.sample)
        mujoco.mj_forward(self.model, self.data)

        self.yaw_gain = self.sample.yaw_gain if self.randomise else 1.0
        self._v_cmd = 0.0
        self._w_cmd = 0.0
        self._steps = 0
        self._origin = (float(self.data.qpos[0]), float(self.data.qpos[1]))
        return self._state(), {'seed': ep_seed}

    def _state(self):
        px, py, _pz = self.data.qpos[0:3]
        qw, qx, qy, qz = self.data.qpos[3:7]
        roll = math.atan2(2 * (qw * qx + qy * qz),
                          1 - 2 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
        yaw = math.atan2(2 * (qw * qz + qx * qy),
                         1 - 2 * (qy * qy + qz * qz))
        vx, vy, _vz = self.data.qvel[0:3]
        v = vx * math.cos(yaw) + vy * math.sin(yaw)
        return np.array([
            px - self._origin[0], py - self._origin[1],
            math.sin(yaw), math.cos(yaw), v, float(self.data.qvel[5]),
            roll, pitch,
        ], dtype=np.float32)

    def step(self, action):
        from coco_rl.mujoco_env import wheel_speeds
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        v_target = float(action[0]) * MAX_LIN
        w_target = float(action[1]) * MAX_ANG * self.yaw_gain
        self._v_cmd += np.clip(v_target - self._v_cmd, -self._dv, self._dv)
        self._w_cmd += np.clip(w_target - self._w_cmd, -self._dw, self._dw)
        left, right = wheel_speeds(
            self._v_cmd, self._w_cmd, self._radius, self._separation)
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
        # Falling off the bridge is the failure the whole deck exists to
        # test, and it is NOT a tip — the robot lands upright on the
        # apron 0.65 m below and would otherwise drive on happily.
        #
        # Bounded to the DECK's x-range. Without the upper bound it also
        # fires on the descent, where dropping below 0.30 m is the entire
        # purpose of the feature, and every successful descent would be
        # scored as a fall.
        deck_x0, deck_x1 = self.params['deck']['x']
        x_now = float(self.data.qpos[0])
        y_now = float(self.data.qpos[1])
        z_now = float(self.data.qpos[2])
        # POSITIONAL, not "has already fallen far". Over the bridge
        # section the deck is genuinely absent outside the bridge, so a
        # centre outside the half-width is over the void and the outcome
        # is settled -- there is nothing left to stand on.
        #
        # Waiting for z to drop instead (the previous test) let the robot
        # pitch and roll through the 0.6 rad terminator on the way down,
        # so every bridge fall was reported as a TIP: measured, roll -43
        # deg and pitch +44 deg at z = 0.610, two control steps after it
        # had already left the deck. That erased the distinction the deck
        # exists to measure.
        bx0, bx1 = self.params['deck']['sections']['bridge']['x']
        b = self.params['bridge']
        over_void = (bx0 <= x_now <= bx1
                     and abs(y_now - b['y_centre'])
                     > b['width']['value'] / 2.0)
        fell = over_void or (z_now < 0.30 and deck_x0 < x_now < deck_x1)

        reward = progress - (10.0 if (tipped or fell) else 0.0)
        terminated = bool(tipped or fell)
        truncated = self._steps >= self.max_steps
        outcome = ('tipped' if tipped else 'fell' if fell
                   else 'timeout' if truncated else 'running')
        return obs, reward, terminated, truncated, {'outcome': outcome}

    def close(self):
        pass
