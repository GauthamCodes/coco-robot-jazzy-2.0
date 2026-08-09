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
The Yard env: reproducible from a seed, and every randomisation lever
actually reaching the simulation.

The second half matters more than it sounds. Phase 2 found a solver
parameter that had been silently disconnected for an entire calibration
while every number in the source still read as applied, and a re-fit
harness whose ``solref`` sweep returns bit-for-bit identical scores to
this day. A randomisation range that cannot be entered is worse than no
randomisation, because the training curve looks fine either way.
"""
import math

from coco_config.robot import (MAX_LINEAR_ACCEL, WHEEL_RADIUS)

from coco_rl.yard_env import CocoYardEnv, MAX_LIN, STEP_DT

from coco_sim.sweep import assert_lever_is_connected

import numpy as np

import pytest


def _rollout(env, action, n):
    obs = [env.reset()[0]]
    for _ in range(n):
        o, r, t, tr, _i = env.step(action)
        obs.append(o)
        if t or tr:
            break
    return np.array(obs)


BANNED = ('rclpy', 'rosidl_runtime_py', 'rmw', 'ament_index_python')


@pytest.fixture
def no_ros(monkeypatch):
    """Poison the import machinery, as test_mujoco_env_has_no_ros does.

    Checking `'rclpy' not in sys.modules` is not enough: another test
    module in the same pytest session may already have imported it, and
    the check then fails for a reason that has nothing to do with this
    env. Hostile isolation is both stricter and order-independent.
    """
    import builtins
    import sys
    for name in list(sys.modules):
        if name.split('.')[0] in BANNED or name.startswith('coco_rl'):
            monkeypatch.delitem(sys.modules, name, raising=False)
    real_import = builtins.__import__

    def guarded(name, *args, **kwargs):
        if name.split('.')[0] in BANNED:
            raise AssertionError(
                f'yard_env pulled in {name!r}. The training environment '
                f'must be pure Python + Gymnasium + MuJoCo.')
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', guarded)
    yield


def test_the_env_imports_without_ros(no_ros):
    """Structural, not stylistic: v1's --fast bug was a chain of ROS
    transport links, and this env cannot have that class of bug."""
    import coco_rl.yard_env            # noqa: F401


def test_the_no_ros_guard_itself_works(no_ros):
    """A guard that cannot fail proves nothing."""
    with pytest.raises(AssertionError):
        import rclpy                   # noqa: F401


def test_same_seed_gives_an_identical_episode():
    a = CocoYardEnv(route='c', seed=42, max_steps=60)
    b = CocoYardEnv(route='c', seed=42, max_steps=60)
    act = np.array([0.6, 0.2], dtype=np.float32)
    assert np.allclose(_rollout(a, act, 50), _rollout(b, act, 50))
    assert a.yaw_gain == b.yaw_gain


def test_different_seeds_give_different_episodes():
    a = CocoYardEnv(route='c', seed=42, max_steps=20)
    b = CocoYardEnv(route='c', seed=43, max_steps=20)
    a.reset()
    b.reset()
    assert a.yaw_gain != b.yaw_gain


def test_successive_episodes_differ_within_one_env():
    """A course that repeats is a course that can be memorised."""
    env = CocoYardEnv(route='a', seed=7, max_steps=20)
    gains = set()
    for _ in range(5):
        env.reset()
        gains.add(round(env.yaw_gain, 9))
    assert len(gains) == 5


def test_an_episode_is_reproducible_from_its_reported_seed():
    """reset() returns the episode seed; that must be enough to replay."""
    env = CocoYardEnv(route='b', seed=3, max_steps=20)
    for _ in range(3):
        env.reset()
    _obs, info = env.reset()
    replay = CocoYardEnv(route='b', seed=0, max_steps=20)
    replay._seed0, replay._episode = 0, 0
    from coco_sim.yard import sample_yard
    direct = sample_yard(replay.params, seed=info['seed'], randomise=True)
    assert direct.yaw_gain == pytest.approx(env.yaw_gain)


@pytest.mark.parametrize('lever', [
    'friction', 'grade_deg', 'camber_deg', 'yaw_gain', 'torque_scale',
    'payload_mass', 'initial_yaw',
])
def test_every_randomisation_lever_actually_varies(lever):
    """Each §2.5 row must reach the sample. Distinct seeds, distinct
    values — checked with the same canary that caught the shadowed
    <pair> parameters."""
    from coco_sim.yard import sample_yard
    env = CocoYardEnv(route='c', seed=0, max_steps=5)
    seen = []
    for seed in range(6):
        s = sample_yard(env.params, seed=seed, randomise=True)
        val = (getattr(s.routes['c'], lever)
               if hasattr(s.routes['c'], lever) else getattr(s, lever))
        seen.append((seed, val))
    assert_lever_is_connected(lever, seen)


def test_terrain_friction_reaches_the_contact_pairs():
    """MuJoCo combines geom friction as an elementwise MAX, so a terrain
    mu below the wheel's 0.4 is a silent no-op unless it is set on the
    PAIR. §2.5's range starts at 0.35, i.e. inside that dead band.

    Reads the pairs belonging to ROUTE C's own surface, not the global
    max. The global max is pinned by the deck's fixed default friction
    and stays constant no matter what the route samples — which this test
    did until the range was narrowed below that default, at which point it
    started failing for a reason unrelated to what it claims to check.
    """
    import mujoco
    env = CocoYardEnv(route='c', seed=1, max_steps=5)
    rubble = mujoco.mj_name2id(
        env.model, mujoco.mjtObj.mjOBJ_GEOM, 'rubble')
    assert rubble >= 0, 'route C surface geom not found'
    seen = []
    for seed in (1, 2, 3, 4):
        env._seed0, env._episode = seed, 0
        env.reset()
        mus = [float(env.model.pair_friction[i][0])
               for i in range(env.model.npair)
               if env.model.pair_geom1[i] == rubble]
        assert mus, 'no contact pairs for the route C surface'
        seen.append((seed, mus[0]))
    assert_lever_is_connected('pair_friction', seen)


def test_route_friction_stays_inside_what_gazebo_can_express():
    """Gazebo saturates above mu 0.7 — the wheels are pinned there by the
    xacro — so a range above it is fictional in the deployment simulator.
    """
    from coco_sim.yard import load_params, sample_yard
    params = load_params()
    lo, hi = params['friction']['range']
    assert (lo, hi) == (0.35, 0.70)
    for key in ('a', 'b', 'c'):
        r_lo, r_hi = params['routes'][key]['friction']
        assert lo <= r_lo < r_hi <= hi, f'route {key} outside {lo}-{hi}'
    for seed in range(40):
        s = sample_yard(params, seed=seed, randomise=True)
        for key in ('a', 'b', 'c'):
            assert lo <= s.routes[key].friction <= hi


def test_the_command_is_ramped_like_the_deployed_controller():
    """diff_drive_controller limits linear acceleration to 2.0 m/s^2.

    Without this the velocity servos answer a step command with whatever
    torque it takes and the robot rears off grippy ground — measured:
    pitch -0.4 deg to +49.5 deg in one control step.
    """
    env = CocoYardEnv(route='a', seed=5, randomise=False, max_steps=20)
    env.reset()
    env.step([1.0, 0.0])
    assert env._v_cmd == pytest.approx(MAX_LINEAR_ACCEL * STEP_DT)
    assert env._v_cmd < MAX_LIN
    for _ in range(10):
        env.step([1.0, 0.0])
    assert env._v_cmd == pytest.approx(MAX_LIN)


def test_torque_scale_is_authority_not_speed():
    """A MuJoCo velocity servo is force = kv*ctrl - kv*vel. Scaling only
    gainprm leaves the damping at the original kv and turns the §2.5
    "wheel torque scale" row into a speed scale."""
    env = CocoYardEnv(route='a', seed=11, max_steps=5)
    env.reset()
    scale = env.sample.torque_scale
    assert scale != pytest.approx(1.0, abs=1e-6)
    assert env.model.actuator_gainprm[0][0] == pytest.approx(
        env._gain0[0][0] * scale)
    assert env.model.actuator_biasprm[0][2] == pytest.approx(
        env._bias0[0][2] * scale)


def test_the_robot_spawns_at_zero_penetration():
    """Spawning at the settled depth makes mj_forward answer the first
    actuator torque with an impulse that throws the robot 85 mm up and
    loses every contact within 12 ms. Spawning clear makes it still be
    falling when the first command lands. Zero is the only stable one."""
    env = CocoYardEnv(route='a', seed=5, randomise=False, max_steps=20)
    env.reset()
    assert float(env.data.qpos[2]) == pytest.approx(WHEEL_RADIUS, abs=1e-9)


def test_driving_off_the_bridge_is_reported_as_a_fall_not_a_tip():
    """The robot pitches hard on the way down, so the tip test also fires;
    checked first it would relabel every bridge fall and erase the one
    distinction the deck exists to measure."""
    env = CocoYardEnv(route='a', seed=0, randomise=False, max_steps=40)
    env.reset()
    p = env.params
    x = sum(p['deck']['sections']['bridge']['x']) / 2.0
    env.data.qpos[0:3] = [x, 1.6, p['deck']['z'] + WHEEL_RADIUS]
    outcome = None
    for _ in range(40):
        _o, _r, t, _tr, info = env.step([0.2, 0.0])
        outcome = info['outcome']
        if t:
            break
    assert outcome == 'fell'


def test_a_successful_descent_is_not_scored_as_a_fall():
    """The descent drops 0.650 m by design; an unbounded fall test would
    score every completed descent as a failure."""
    env = CocoYardEnv(route='a', seed=0, randomise=False, max_steps=40)
    env.reset()
    p = env.params
    env.data.qpos[0:3] = [p['deck']['x'][1] + 1.0, 0.0, 0.30]
    for _ in range(15):
        _o, _r, t, _tr, info = env.step([0.3, 0.0])
        assert info['outcome'] != 'fell'
        if t:
            break


def test_action_and_observation_spaces_match_the_other_envs():
    """Parity with ramp_env and mujoco_env, so a policy or a measurement
    can move between them. The action space is on the do-not-touch list."""
    env = CocoYardEnv(route='b', seed=0, max_steps=5)
    assert env.action_space.shape == (2,)
    assert env.observation_space.shape == (8,)
    assert (MAX_LIN, math.isclose(STEP_DT, 0.1)) == (0.4, True)
