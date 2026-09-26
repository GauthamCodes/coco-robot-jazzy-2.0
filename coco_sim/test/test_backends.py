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
One manifest, two engines, no second episode system.

The most important test here is ``test_fixed_gazebo_spawns_are_byte_
identical_to_the_p02_launch``: it carries a VERBATIM copy of the target
spawning code that ``full_world_robo.launch.py`` ran before stage C, and
asserts the Gazebo backend produces the same SDF text and the same
``create`` argv for the default episode. If that holds, the default world
is the P0.2 world, byte for byte.

The rest pin the adapter boundary: both backends read the same bodies
from the same manifest, neither imports ROS or a simulator, each refuses
an episode resolved for the other, and the read-back check catches a
spawned layout that differs from the manifest.
"""

from dataclasses import replace
import math
import subprocess
import sys

from coco_config.robot import (RAMP_ANGLE_DEG, RAMP_RUN, TARGET_MASS,
                               TARGET_ROW_X, TARGETS)
from coco_sim.backends import (ADAPTERS, backend_for, check_instantiation,
                               GazeboBackend, IsaacBackend, ObservedPose,
                               target_body)
from coco_sim.backends.common import TARGET_FRICTION
from coco_sim.backends.isaac import COCO_WORLD_GEOMETRY_FOR_ISAAC
from coco_sim.episode import generate_episode, InvalidEpisode, LEVELS, rebind
import pytest


def _p02_launch_target_spawns(ramp_angle):
    """The pre-stage-C launch file's target spawning, copied VERBATIM.

    From gazebo_models/launch/full_world_robo.launch.py at c40098f, with
    only the Node() wrapper reduced to the argv it was given. Do not
    edit this function to make a test pass; it is the reference.
    """
    rise = RAMP_RUN * math.tan(math.radians(ramp_angle))
    out = []
    for target in TARGETS:
        radius = target.diameter / 2.0
        height = target.height
        i_xx = TARGET_MASS * (3.0 * radius ** 2 + height ** 2) / 12.0
        i_zz = TARGET_MASS * radius ** 2 / 2.0
        target_sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{target.model}">
    <link name="link">
      <inertial><mass>{TARGET_MASS}</mass>
        <inertia><ixx>{i_xx:.6e}</ixx><iyy>{i_xx:.6e}</iyy>
                 <izz>{i_zz:.6e}</izz>
                 <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="c"><geometry><cylinder>
        <radius>{radius}</radius><length>{height}</length></cylinder></geometry>
        <surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction></surface>
      </collision>
      <visual name="v"><geometry><cylinder>
        <radius>{radius}</radius><length>{height}</length></cylinder></geometry>
        <material><ambient>{target.rgb} 1</ambient>
                  <diffuse>{target.rgb} 1</diffuse></material></visual>
    </link>
  </model>
</sdf>'''
        out.append(['-name', target.model, '-string', target_sdf,
                    '-x', str(TARGET_ROW_X), '-y', str(target.lane_y),
                    '-z', str(rise + height / 2.0)])
    magnets = ['--models'] + [t.model for t in TARGETS]
    return out, magnets


# ── H/I. Gazebo spawning, and the fixed-mode regression ──────────────────
def test_fixed_gazebo_spawns_are_byte_identical_to_the_p02_launch():
    spec = generate_episode(seed=0, level='fixed')
    scene = GazeboBackend().translate(spec, ramp_angle_deg=RAMP_ANGLE_DEG)
    reference, magnets = _p02_launch_target_spawns(RAMP_ANGLE_DEG)
    assert [s.arguments() for s in scene.targets] == reference
    assert ['--models'] + list(scene.magnet_models) == magnets


@pytest.mark.parametrize('seed', [0, 1, 1827])
def test_fixed_spawns_do_not_depend_on_seed_or_requested_colour(seed):
    reference, _ = _p02_launch_target_spawns(RAMP_ANGLE_DEG)
    for colour in ('red', 'yellow'):
        spec = generate_episode(seed=seed, level='fixed',
                                requested_colour=colour)
        scene = GazeboBackend().translate(spec, ramp_angle_deg=18)
        assert [s.arguments() for s in scene.targets] == reference


@pytest.mark.parametrize('level', LEVELS)
def test_gazebo_spawns_every_target_exactly_at_its_manifest_pose(level):
    for seed in range(50):
        spec = generate_episode(seed=seed, level=level)
        scene = GazeboBackend().translate(spec)
        assert [s.name for s in scene.targets] == [t.model
                                                   for t in spec.targets]
        for spawn, target in zip(scene.targets, spec.targets):
            assert (spawn.x, spawn.y, spawn.z) == (target.x, target.y,
                                                   target.z)
            # the argv carries the manifest float exactly, not rounded
            argv = spawn.arguments()
            assert float(argv[argv.index('-x') + 1]) == target.x
            assert float(argv[argv.index('-y') + 1]) == target.y


def test_a_moved_target_keeps_its_identity_and_its_magnet():
    """COLOUR moves where red stands, never what red is."""
    spec = generate_episode(seed=3, level='colours')
    scene = GazeboBackend().translate(spec)
    for spawn, target in zip(scene.targets, spec.targets):
        assert spawn.name == f'target_{target.colour}'
        assert f'<model name="target_{target.colour}">' in spawn.sdf
    assert sorted(scene.magnet_models) == sorted(t.model for t in TARGETS)


def test_gazebo_refuses_a_world_built_at_another_grade():
    spec = generate_episode(seed=0)
    with pytest.raises(InvalidEpisode, match='deg'):
        GazeboBackend().translate(spec, ramp_angle_deg=24)


# ── K. the backend abstraction ───────────────────────────────────────────
def test_both_engines_build_the_same_bodies_from_one_manifest():
    for level in LEVELS:
        gz = generate_episode(seed=9, level=level)
        isaac = rebind(gz, backend='isaac')
        g_scene = GazeboBackend().translate(gz)
        i_scene = IsaacBackend().translate(isaac)
        for spawn, prim in zip(g_scene.targets, i_scene.targets):
            assert prim.name == spawn.name
            assert prim.position == (spawn.x, spawn.y, spawn.z)
        assert [target_body(t) for t in gz.targets] == \
            [target_body(t) for t in isaac.targets]


def test_isaac_prims_carry_the_same_physics_as_the_sdf():
    spec = generate_episode(seed=2, level='positions', backend='isaac')
    for prim, target in zip(IsaacBackend().translate(spec).targets,
                            spec.targets):
        body = target_body(target)
        assert prim.mass == TARGET_MASS
        assert prim.diagonal_inertia == (body.ixx, body.ixx, body.izz)
        assert prim.static_friction == prim.dynamic_friction == \
            TARGET_FRICTION
        assert prim.radius == target.diameter / 2.0
        assert prim.path == f'/World/Targets/{target.model}'


def test_isaac_says_what_world_geometry_it_cannot_build():
    """The boundary is honest: targets translate, the arena does not."""
    scene = IsaacBackend().translate(generate_episode(seed=0,
                                                      backend='isaac'))
    assert scene.missing == COCO_WORLD_GEOMETRY_FOR_ISAAC
    assert 'platform' in scene.missing


def test_each_backend_refuses_the_others_episode():
    with pytest.raises(InvalidEpisode, match='not isaac'):
        IsaacBackend().translate(generate_episode(seed=0))
    with pytest.raises(InvalidEpisode, match='not gazebo'):
        GazeboBackend().translate(generate_episode(seed=0, backend='isaac'))


def test_a_backend_revalidates_a_hand_edited_manifest():
    spec = generate_episode(seed=0)
    bad = replace(spec, targets=(replace(spec.targets[0], x=2.0),)
                  + spec.targets[1:])
    with pytest.raises(InvalidEpisode):
        GazeboBackend().translate(bad)


def test_the_registry_names_the_manifest_backends():
    assert set(ADAPTERS) == {'gazebo', 'isaac'}
    assert isinstance(backend_for(generate_episode(seed=0)), GazeboBackend)
    with pytest.raises(KeyError):
        backend_for(generate_episode(seed=0, backend='mujoco'))


def test_the_backend_layer_imports_no_ros_and_no_simulator():
    """Same rule as the MuJoCo env: importable with nothing else around."""
    code = ('import sys; import coco_sim.backends; '
            'bad = [m for m in sys.modules if m.split(".")[0] in '
            '("rclpy", "omni", "pxr", "isaacsim", "gz", "launch")]; '
            'print(bad); sys.exit(1 if bad else 0)')
    result = subprocess.run([sys.executable, '-c', code],
                            capture_output=True, text=True,
                            env={'PYTHONPATH': ':'.join(sys.path)})
    assert result.returncode == 0, result.stdout + result.stderr


# ── read-back ────────────────────────────────────────────────────────────
def _observed(spec, **overrides):
    poses = {t.model: ObservedPose(t.x, t.y, t.z) for t in spec.targets}
    poses.update(overrides)
    return poses


def test_a_faithful_readback_passes():
    spec = generate_episode(seed=4, level='positions')
    report = check_instantiation(spec, _observed(spec))
    assert report['ok'] and report['layout_valid']
    assert not report['missing']


def test_a_missing_model_fails_the_readback():
    spec = generate_episode(seed=4)
    poses = _observed(spec)
    del poses['target_red']
    report = check_instantiation(spec, poses)
    assert not report['ok'] and report['missing'] == ['target_red']


def test_a_tipped_or_displaced_target_fails_the_readback():
    spec = generate_episode(seed=4)
    t = spec.target('blue')
    tipped = check_instantiation(spec, _observed(
        spec, target_blue=ObservedPose(t.x, t.y, t.z, roll=0.5)))
    assert not tipped['models']['target_blue']['upright']
    assert not tipped['ok']
    moved = check_instantiation(spec, _observed(
        spec, target_blue=ObservedPose(t.x + 0.02, t.y, t.z)))
    assert not moved['models']['target_blue']['xy_ok']


def test_readback_revalidates_the_layout_it_actually_saw():
    """A target that slid off its region fails even if xy_tol were loose."""
    spec = generate_episode(seed=4)
    t = spec.target('green')
    report = check_instantiation(
        spec, _observed(spec, target_green=ObservedPose(t.x, t.y + 0.1,
                                                        t.z)),
        xy_tol=1.0)
    assert report['models']['target_green']['xy_ok']
    assert not report['layout_valid']
    assert 'outside region' in report['layout_error']
