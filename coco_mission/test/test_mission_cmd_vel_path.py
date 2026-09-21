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
C2-NAV.42: the mission's Nav2 commands reach the wheels through the monitor.

mission.launch.py is topology B, the path the robot ships in. It does not
wire topics itself: it includes nav.launch.py with an `arbiter` value and
includes arbiter.launch.py. So the check is the one that matters for the
mission -- take the value mission.launch.py ACTUALLY passes, resolve both
included launch files with it the way `ros2 launch` would, and require that
the relay's output is the arbiter's nav input and neither is /cmd_vel_nav,
the controller's raw command. gazebo_models/test/test_cmd_vel_wiring.py
proves on a live graph what that wiring does.

The launch files are parsed, never run: nothing here starts a node.
"""

import ast
import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, '..', '..')
MISSION_LAUNCH = os.path.join(HERE, '..', 'launch', 'mission.launch.py')
NAV_LAUNCH = os.path.join(REPO, 'gazebo_models', 'launch', 'nav.launch.py')
ARBITER_LAUNCH = os.path.join(REPO, 'custom_teleop', 'launch',
                              'arbiter.launch.py')
WEB_LAUNCH = os.path.join(REPO, 'coco_web', 'launch', 'web.launch.py')
PLATFORM_LAUNCH = os.path.join(REPO, 'coco_web', 'launch',
                               'platform.launch.py')

RAW_NAV_TOPIC = '/cmd_vel_nav'

#: The two web layers mission.launch.py can start, selected by platform:=.
#: Exactly one must ever be chosen: both start web_video_server on 8081,
#: and both would start an arbiter if their arbiter argument were not
#: false -- which would put two publishers on the wheel topic.
WEB_LAYERS = ('web.launch.py', 'platform.launch.py')


def _web_includes_chosen(platform, web='true'):
    """Which web launch files mission.launch.py selects for platform:=.

    Builds the real launch description and evaluates each include's
    condition, rather than reading the source: the conditions are
    PythonExpression substitutions, and only evaluating them proves what
    `ros2 launch` would actually do.
    """
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

    spec = importlib.util.spec_from_file_location('mission_launch',
                                                  MISSION_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()

    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)
    context.launch_configurations['platform'] = platform
    context.launch_configurations['web'] = web

    chosen = []
    for entity in description.entities:
        if not isinstance(entity, IncludeLaunchDescription):
            continue
        source = entity.launch_description_source
        location = getattr(
            source, '_LaunchDescriptionSource__location', None) or []
        name = os.path.basename(''.join(
            getattr(part, 'text', '') for part in location))
        if name not in WEB_LAYERS:
            continue
        if entity.condition is None or entity.condition.evaluate(context):
            chosen.append(name)
    return chosen


def _tree(path):
    with open(path) as f:
        return ast.parse(f.read())


def _includes(path):
    """[(launch file name, {argument: value}), ...] from include(...) calls."""
    found = []
    for call in ast.walk(_tree(path)):
        if not (isinstance(call, ast.Call)
                and getattr(call.func, 'id', None) == 'include'):
            continue
        name = next((a.value for a in call.args
                     if isinstance(a, ast.Constant)
                     and str(a.value).endswith('.launch.py')), None)
        arguments = {}
        for arg in call.args:
            if isinstance(arg, ast.Dict):
                for key, value in zip(arg.keys, arg.values):
                    if isinstance(key, ast.Constant):
                        arguments[key.value] = (
                            value.value if isinstance(value, ast.Constant)
                            else None)
        found.append((name, arguments))
    return found


def _code_strings(path):
    """Every string literal that is code, not a docstring."""
    tree = _tree(path)
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.ClassDef)):
            body = node.body
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)):
                docstrings.add(id(body[0].value))
    return [node.value for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
            and id(node) not in docstrings]


def _resolved(path, executable, **configurations):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    spec = importlib.util.spec_from_file_location('launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(configurations)
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)
    nodes = [e for e in description.entities
             if isinstance(e, Node) and e.node_executable == executable]
    assert len(nodes) == 1
    merged = {}
    for params in evaluate_parameters(context, nodes[0]._Node__parameters):
        merged.update(params)
    return merged


def _mission_arbiter_value():
    navs = [args for name, args in _includes(MISSION_LAUNCH)
            if name == 'nav.launch.py']
    assert len(navs) == 1
    return navs[0]['arbiter']


class TestMissionCommandPath:
    """What mission.launch.py wires, resolved through what it includes."""

    def test_the_mission_runs_nav2_through_the_arbiter(self):
        assert _mission_arbiter_value() == 'true'

    def test_the_mission_starts_exactly_one_arbiter(self):
        names = [name for name, _ in _includes(MISSION_LAUNCH)]
        assert names.count('arbiter.launch.py') == 1
        # EITHER web layer's own arbiter would be a second wheel
        # publisher, so both must be included with arbiter:=false --
        # whichever one platform:= ends up selecting.
        webs = [args for name, args in _includes(MISSION_LAUNCH)
                if name in WEB_LAYERS]
        assert len(webs) == len(WEB_LAYERS)
        assert [args.get('arbiter') for args in webs] == ['false'] * len(webs)

    @pytest.mark.parametrize('platform,expected', [
        ('true', 'platform.launch.py'),
        ('false', 'web.launch.py'),
    ])
    def test_platform_selects_exactly_one_web_layer(self, platform, expected):
        """Never two: they collide on 8081 and each can start an arbiter."""
        assert _web_includes_chosen(platform) == [expected]

    def test_web_false_starts_no_web_layer_at_all(self):
        """An evaluation sweep runs headless and wants neither."""
        assert _web_includes_chosen('true', web='false') == []
        assert _web_includes_chosen('false', web='false') == []

    def test_the_mission_relay_feeds_the_mission_arbiter(self):
        arbiter = _mission_arbiter_value()
        relay = _resolved(NAV_LAUNCH, 'cmd_vel_relay', arbiter=arbiter)
        arb = _resolved(ARBITER_LAUNCH, 'cmd_vel_arbiter')
        assert relay['output_topic'] == arb['nav_topic']

    def test_neither_end_is_the_raw_controller_topic(self):
        arbiter = _mission_arbiter_value()
        relay = _resolved(NAV_LAUNCH, 'cmd_vel_relay', arbiter=arbiter)
        arb = _resolved(ARBITER_LAUNCH, 'cmd_vel_arbiter')
        assert relay['output_topic'] != RAW_NAV_TOPIC
        assert arb['nav_topic'] != RAW_NAV_TOPIC

    @pytest.mark.parametrize('path', [MISSION_LAUNCH, WEB_LAUNCH,
                                      PLATFORM_LAUNCH, NAV_LAUNCH,
                                      ARBITER_LAUNCH])
    def test_no_launch_file_on_the_path_names_the_raw_topic_in_code(
            self, path):
        # A remap or a parameter naming /cmd_vel_nav anywhere on the
        # mission's launch path would reopen the loop. Docstrings may still
        # explain why it is absent.
        assert not [s for s in _code_strings(path) if 'cmd_vel_nav' in s]


def _platform_expected_components(executive):
    """
    Evaluate the expected_components mission.launch.py hands the platform.

    Evaluated, not read: the value is a PythonExpression over
    executive:=, and only performing it proves what the platform receives.
    """
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.utilities import (normalize_to_list_of_substitutions,
                                  perform_substitutions)

    def perform(value):
        return perform_substitutions(
            context, normalize_to_list_of_substitutions(value))

    spec = importlib.util.spec_from_file_location('mission_launch_expected',
                                                  MISSION_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)
    context.launch_configurations['executive'] = executive
    for entity in description.entities:
        if not isinstance(entity, IncludeLaunchDescription):
            continue
        location = getattr(entity.launch_description_source,
                           '_LaunchDescriptionSource__location', None) or []
        name = os.path.basename(''.join(
            getattr(part, 'text', '') for part in location))
        if name != 'platform.launch.py':
            continue
        for key, value in entity.launch_arguments:
            if perform(key) == 'expected_components':
                return perform(value)
    return None


@pytest.mark.parametrize('executive,expected', [
    ('true', 'lidar,navigation,perception,mission'),
    ('false', 'lidar,navigation,perception'),
])
def test_the_mission_stack_declares_what_the_platform_should_expect(
        executive, expected):
    """
    Health DEGRADED must mean something the mission launch actually started.

    Nav2 and perception are always started here; the executive only with
    executive:=true. Declaring the executive expected on a traverse_demo
    run (executive:=false) would report a permanent fault that is not one.
    """
    assert _platform_expected_components(executive) == expected
