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
The information boundary, as code.

Two functions, and the whole of C2-M2's honesty rests on which one a
caller reaches for:

    deployable_signals(...)   what the robot could know
    ground_truth(...)         what only the simulator knows

The first builds a :class:`~coco_rl.terrain_observer.DeployableSignals`,
which is the only type the observer accepts. The second builds a
:class:`GroundTruth`, which the observer cannot consume because it is a
different type with no overlapping fields. **Feeding truth to the
observer is therefore a TypeError, not a code-review miss.** That is
deliberate: C2-M1.5 was spent on a signal that was correct everywhere it
was tested and wrong everywhere else, and a convention would not have
caught it.

Nothing here is a shortcut through the physics. ``deployable_signals``
models the sensors the robot actually carries -- ``coco_robo2.xacro``
declares the IMU at ``<update_rate>50</update_rate>``, so the IMU is
sampled at 50 Hz, and specific force is formed the way an accelerometer
forms it:

    f_body = R^T (a_world - g_world)

which reads ``(0, 0, +9.81)`` at rest on flat ground rather than zero.

**One fidelity limitation, stated up front.** The simulated IMU is
noiseless. ``yard_params.yaml`` records ``imu_noise_sigma:
not_yet_measured`` because the xacro's ``<sensor name="imu">`` declares
no ``<noise>`` element, and this module does not invent one -- a
fabricated noise floor would make C2-M2.1's table read as validated when
it is not. The consequence is recorded in ``terrain_observer``'s
docstring and it is the reason nothing in the observer integrates.
"""

import math

from coco_config.robot import WHEEL_RADIUS

from coco_rl.terrain_observer import DeployableSignals

import numpy as np


G = 9.81
IMU_HZ = 50.0                    # coco_robo2.xacro <update_rate>

# The MJCF's wheel joints, in the order coco_sim.mjcf.WHEEL_SITES emits
# them. Looked up by NAME at runtime rather than by a qvel offset, because
# an offset silently follows any change to the model's joint list and a
# name does not.
WHEEL_JOINTS = ('front_right_joint', 'rear_right_joint',
                'front_left_joint', 'rear_left_joint')


def quat_to_rpy(q):
    """(roll, pitch, yaw) from a (w, x, y, z) quaternion.

    Same convention as ``coco_rl.ramp_env.quat_to_rp`` and
    ``yard_env._state``, which is what makes a measurement portable
    between the three. **Nose-up is NEGATIVE pitch** -- measured this
    session at -12.00 deg on Route A's +12.000 deg face.
    """
    qw, qx, qy, qz = (float(v) for v in q)
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def _rot(q):
    """Body-to-world rotation matrix from a (w, x, y, z) quaternion."""
    qw, qx, qy, qz = (float(v) for v in q)
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
         2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
         2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
         1 - 2 * (qx * qx + qy * qy)]])


class ImuSampler:
    """A 50 Hz IMU over a MuJoCo model. Reads state; never writes it.

    Held by the environment and ticked inside its substep loop, because
    50 Hz is five times the 10 Hz control rate and an accelerometer
    sampled at the control rate is a different sensor. It touches only
    ``data.qvel`` and ``data.qpos``, so it cannot perturb the simulation
    -- ``test_terrain_observer.py`` asserts that by running an episode
    with the sampler on and off and requiring identical final state.
    """

    def __init__(self, model, hz=IMU_HZ):
        self.hz = float(hz)
        self._v_prev = None
        self._t_prev = None
        self.latest = None
        self._wheel_dofs = self._resolve_wheels(model)

    @staticmethod
    def _resolve_wheels(model):
        import mujoco
        dofs = []
        for name in WHEEL_JOINTS:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise KeyError(f'wheel joint {name!r} is not in this model')
            dofs.append(int(model.jnt_dofadr[jid]))
        return dofs

    def reset(self, data):
        self._v_prev = np.array(data.qvel[0:3], dtype=float)
        self._t_prev = float(data.time)
        self.latest = None

    def sample(self, data):
        """One IMU reading. Returns (roll, pitch, yaw, rates, f_body, wheels)."""
        t = float(data.time)
        v = np.array(data.qvel[0:3], dtype=float)
        dt = t - (self._t_prev if self._t_prev is not None else t)
        if dt <= 0.0:
            a_world = np.zeros(3)
        else:
            a_world = (v - self._v_prev) / dt
        self._v_prev, self._t_prev = v, t

        q = data.qpos[3:7]
        # Specific force: what an accelerometer measures, gravity included.
        f_body = _rot(q).T @ (a_world - np.array([0.0, 0.0, -G]))
        roll, pitch, yaw = quat_to_rpy(q)
        # A free joint's angular velocity is already in the LOCAL frame,
        # which is exactly what a body-mounted gyro reports.
        gyro = tuple(float(w) for w in data.qvel[3:6])
        wheels = tuple(float(data.qvel[d]) for d in self._wheel_dofs)
        self.latest = dict(stamp=t, roll=roll, pitch=pitch, yaw=yaw,
                           gyro=gyro, accel=tuple(float(a) for a in f_body),
                           wheels=wheels)
        return self.latest


def deployable_signals(sample, cmd_linear, cmd_angular,
                       wheel_radius=WHEEL_RADIUS, on_declared_flat=False):
    """Wrap one IMU sample and the commanded twist. The observer's ONLY input.

    ``sample`` is an :class:`ImuSampler` reading; on the real robot it is
    one ``/imu`` message plus the matching ``/joint_states`` velocities.
    Nothing in this signature can carry world pose, true grade or true
    friction, and that is the point.
    """
    if sample is None:
        return None
    return DeployableSignals(
        stamp=float(sample['stamp']),
        roll=float(sample['roll']), pitch=float(sample['pitch']),
        yaw=float(sample['yaw']),
        roll_rate=float(sample['gyro'][0]),
        pitch_rate=float(sample['gyro'][1]),
        yaw_rate=float(sample['gyro'][2]),
        accel_body=tuple(sample['accel']),
        wheel_speeds=tuple(sample['wheels']),
        cmd_linear=float(cmd_linear), cmd_angular=float(cmd_angular),
        wheel_radius=float(wheel_radius),
        on_declared_flat=bool(on_declared_flat))


# ── the other side of the boundary ───────────────────────────────────────
class GroundTruth:
    """What only the simulator knows. **Evaluation only.**

    Deliberately not a ``DeployableSignals``, and deliberately sharing no
    field name with it, so that a copy-paste between the two does not
    typecheck. Used to score the observer in C2-M2.1 and to schedule B2,
    which is privileged by definition. Never reaches B3.
    """

    __slots__ = ('x', 'y', 'z', 'grade', 'camber', 'friction',
                 'v_body', 'v_wheel', 'slip')

    def __init__(self, x, y, z, grade, camber, friction, v_body, v_wheel,
                 slip):
        self.x, self.y, self.z = x, y, z
        self.grade, self.camber, self.friction = grade, camber, friction
        self.v_body, self.v_wheel, self.slip = v_body, v_wheel, slip

    def __repr__(self):
        return (f'GroundTruth(x={self.x:.3f} grade={math.degrees(self.grade):.2f}'
                f'deg mu={self.friction:.3f} slip={self.slip:.3f})')


# Central difference step for the analytic surface. 0.05 m is a third of
# the 0.18 m wheelbase: short enough to be local, long enough not to read
# the rubble's own 0.12 m grain as a slope.
GRADE_PROBE_DX = 0.05


def true_grade(x, y, sample, heading=0.0, dx=GRADE_PROBE_DX):
    """Surface grade along ``heading`` at (x, y), from the analytic surface.

    Positive uphill, matching the observer's convention. Uses
    ``coco_sim.yard.height``, which ``yard_params.yaml`` calls the sole
    source of Yard geometry, rather than probing contacts.

    **Undefined over the bridge void**, where the surface drops 0.650 m to
    the apron: a central difference across that edge returns a grade of
    tens of degrees that is an artefact of the discontinuity, not a
    slope. Callers scoring the estimator must bound themselves to the
    ramp face; ``benchmark`` does.
    """
    from coco_sim.yard import height
    c, s = math.cos(heading), math.sin(heading)
    h1 = height(x + dx * c, y + dx * s, sample)
    h0 = height(x - dx * c, y - dx * s, sample)
    return math.atan2(h1 - h0, 2 * dx)


def true_camber(x, y, sample, heading=0.0, dx=GRADE_PROBE_DX):
    """Cross-slope at (x, y), positive left-side-up."""
    return true_grade(x, y, sample, heading=heading + math.pi / 2.0, dx=dx)


def ground_truth(env, route):
    """Extract the evaluation-only state from a running ``CocoYardEnv``."""
    d = env.data
    x, y, z = (float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2]))
    _r, _p, yaw = quat_to_rpy(d.qpos[3:7])
    v_body = float(d.qvel[0]) * math.cos(yaw) + float(d.qvel[1]) * math.sin(yaw)
    dofs = ImuSampler._resolve_wheels(env.model)
    v_wheel = (sum(float(d.qvel[k]) for k in dofs) / len(dofs)) * WHEEL_RADIUS
    slip = ((v_wheel - v_body) / abs(v_wheel)) if abs(v_wheel) > 1e-6 else 0.0
    return GroundTruth(
        x=x, y=y, z=z,
        grade=true_grade(x, y, env.sample, heading=yaw),
        camber=true_camber(x, y, env.sample, heading=yaw),
        friction=float(env.sample.routes[route].friction),
        v_body=v_body, v_wheel=v_wheel, slip=slip)
