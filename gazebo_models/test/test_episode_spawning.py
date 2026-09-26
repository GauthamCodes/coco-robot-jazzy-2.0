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
full_world_robo.launch.py spawns the targets an episode says, and only those.

These run the launch file's REAL ``launch_setup`` against a bare
``LaunchContext`` — no simulator, no ROS graph — and read the argv of
every ``ros_gz_sim create`` it would run. That is the last step before
gz: if the argv is right here, the only thing left for a live run to
prove is that gz honours it, which ``docs/data/p03c_episode_gazebo/``
reads back.

coco_sim's ``test_backends.py`` pins the fixed episode against a
verbatim copy of the pre-episode launch code; this file pins that the
launch file actually USES the backend, for every level, and that the
switch defaults to fixed.
"""

import importlib.util
import json
import os

from coco_sim.backends import GazeboBackend
from coco_sim.episode import generate_episode, InvalidEpisode
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import pytest

LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch',
                      'full_world_robo.launch.py')


def _module():
    spec = importlib.util.spec_from_file_location('coco_full_world', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _declared():
    return {e.name: e for e in _module().generate_launch_description()
            .entities if isinstance(e, DeclareLaunchArgument)}


def _run(**overrides):
    """Run launch_setup with the file's own defaults plus `overrides`."""
    configs = {name: ''.join(s.perform(None) if hasattr(s, 'perform')
                             else str(s) for s in arg.default_value)
               for name, arg in _declared().items()}
    configs.update({k: str(v) for k, v in overrides.items()})
    context = LaunchContext()
    context.launch_configurations.update(configs)
    return context, _module().launch_setup(context)


def _argv(context, node):
    """Return the command-line arguments a Node was given, as strings.

    Read from the Node's own ``arguments`` (name-mangled, private) rather
    than ``cmd``: ``cmd`` also carries the --ros-args substitutions, which
    only resolve inside a running launch service. The launch file passes
    plain strings, so plain strings are what should come back.
    """
    out = []
    for arg in getattr(node, '_Node__arguments') or []:
        if isinstance(arg, str):
            out.append(arg)
        else:
            out.append(''.join(s.perform(context) for s in arg))
    return out


def _target_spawns(context, actions):
    out = []
    for action in actions:
        if isinstance(action, Node):
            argv = _argv(context, action)
            if '-name' in argv and argv[argv.index('-name') + 1] \
                    .startswith('target_'):
                out.append(argv)
    return out


def _magnets(context, actions):
    for action in actions:
        if isinstance(action, Node):
            argv = _argv(context, action)
            if argv and argv[0] == '--models':
                return argv[1:]
    return None


# ── the switch ───────────────────────────────────────────────────────────
def test_the_episode_switch_defaults_to_fixed():
    declared = _declared()
    for name in ('episode_level', 'episode_seed', 'episode_colour',
                 'episode_manifest', 'episode_record'):
        assert name in declared, name
    default = ''.join(s.perform(None) for s in
                      declared['episode_level'].default_value)
    assert default == 'fixed'
    assert declared['episode_level'].choices == ['fixed', 'colours',
                                                 'positions']


def test_the_launch_file_still_has_one_opaque_setup():
    entities = _module().generate_launch_description().entities
    assert sum(isinstance(e, OpaqueFunction) for e in entities) == 1


# ── I. fixed: the default spawns the P0.2 layout ─────────────────────────
def test_default_traverse_spawns_the_fixed_episode_exactly():
    context, actions = _run(traverse='true', gui='false')
    expected = GazeboBackend().translate(generate_episode(seed=0),
                                         ramp_angle_deg=18)
    assert _target_spawns(context, actions) == [
        s.arguments() for s in expected.targets]
    assert _magnets(context, actions) == list(expected.magnet_models)


def test_no_traverse_spawns_no_targets_at_any_grade():
    """The curriculum worlds (no platform) never resolve an episode."""
    for grade in ('12', '18', '24'):
        context, actions = _run(traverse='false', gui='false',
                                ramp_angle=grade)
        assert _target_spawns(context, actions) == []


def test_an_episode_without_the_platform_is_refused():
    with pytest.raises(RuntimeError, match='traverse:=true'):
        _run(traverse='false', gui='false', episode_level='colours')


# ── H. episode levels reach gz as the manifest says ──────────────────────
@pytest.mark.parametrize('level,seed', [('colours', 1827), ('positions', 7),
                                        ('positions', 1827)])
def test_an_episode_spawns_its_manifest(level, seed):
    context, actions = _run(traverse='true', gui='false',
                            episode_level=level, episode_seed=seed,
                            episode_colour='red')
    spec = generate_episode(seed=seed, level=level, requested_colour='red')
    expected = GazeboBackend().translate(spec)
    assert _target_spawns(context, actions) == [
        s.arguments() for s in expected.targets]


def test_colours_moves_targets_off_the_fixed_lanes():
    _, fixed = _run(traverse='true', gui='false')
    context, moved = _run(traverse='true', gui='false',
                          episode_level='colours', episode_seed=1827)
    assert _target_spawns(context, fixed) != _target_spawns(context, moved)


# ── J. the manifest used is recorded, and a recorded one replays ─────────
def test_the_spawned_manifest_is_recorded(tmp_path):
    record = tmp_path / 'run' / 'spawned.json'
    _run(traverse='true', gui='false', episode_level='positions',
         episode_seed=42, episode_colour='blue', episode_record=record)
    spec = generate_episode(seed=42, level='positions',
                            requested_colour='blue')
    assert record.read_text() == spec.to_json() + '\n'


def test_a_recorded_manifest_is_spawned_exactly(tmp_path):
    spec = generate_episode(seed=5, level='positions',
                            requested_colour='yellow')
    path = tmp_path / 'm.json'
    path.write_text(spec.to_json())
    context, actions = _run(traverse='true', gui='false',
                            episode_manifest=path,
                            episode_level='fixed', episode_seed=0)
    assert _target_spawns(context, actions) == [
        s.arguments() for s in GazeboBackend().translate(spec).targets]


def test_a_manifest_for_another_grade_is_refused(tmp_path):
    spec = generate_episode(seed=5, level='colours')
    path = tmp_path / 'm.json'
    path.write_text(spec.to_json())
    with pytest.raises(InvalidEpisode, match='deg'):
        _run(traverse='true', gui='false', ramp_angle='24',
             episode_manifest=path)


def test_a_manifest_that_breaks_the_envelope_is_refused(tmp_path):
    data = json.loads(generate_episode(seed=5, level='colours').to_json())
    data['targets'][0]['x'] = 3.0
    path = tmp_path / 'm.json'
    path.write_text(json.dumps(data))
    with pytest.raises(InvalidEpisode):
        _run(traverse='true', gui='false', episode_manifest=path)
