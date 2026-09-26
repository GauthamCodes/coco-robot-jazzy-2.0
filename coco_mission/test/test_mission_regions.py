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
Stage C, the mission side: requested colour -> episode region -> lane.

The mission is not rewritten. It gains one input — an episode's
colour -> region NAME map — and uses it where it used to call
``lane_for_colour``. These tests pin three things:

1. **Fixed is unchanged.** No map (the default) gives exactly the frozen
   lanes, through the plan, the executive and the launch file.
2. **The map is honoured everywhere a lane is chosen**: the pre-ramp
   goal, the colour-change path, the ALIGN/CLIMB checks' datum.
3. **Only names cross the boundary.** The launch file reduces an episode
   to a colour and a region map; no node parameter carries a coordinate.
"""

import importlib.util
import json
import os
import sys

from coco_config.robot import (FIXED_REGION_MAP, format_region_map,
                               lane_for_colour, lane_for_region,
                               TARGET_COLOURS)
from coco_sim.episode import compat_mission_inputs, generate_episode
from launch import LaunchContext
from launch.actions import SetLaunchConfiguration
import pytest
import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import mission_executive as mx  # noqa: E402, I100
import mission_states as ms  # noqa: E402

from std_msgs.msg import String  # noqa: E402, I100

LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch',
                      'mission.launch.py')

#: An assignment that moves every colour off its frozen lane.
ROTATED = {'red': 'lane_2', 'green': 'lane_3', 'blue': 'lane_4',
           'yellow': 'lane_1'}


# ── the plan ─────────────────────────────────────────────────────────────
@pytest.mark.parametrize('colour', TARGET_COLOURS)
def test_a_plan_with_no_map_is_the_frozen_table(colour):
    for plan in (ms.MissionPlan(colour),
                 ms.MissionPlan(colour, region_map={}),
                 ms.MissionPlan(colour, region_map=FIXED_REGION_MAP)):
        assert plan.lane == lane_for_colour(colour)
        assert plan.pre_ramp == (ms.PRE_RAMP_X, lane_for_colour(colour))


@pytest.mark.parametrize('colour', TARGET_COLOURS)
def test_a_plan_climbs_the_lane_of_the_episode_region(colour):
    plan = ms.MissionPlan(colour, region_map=ROTATED)
    assert plan.region == ROTATED[colour]
    assert plan.lane == lane_for_region(ROTATED[colour])
    assert plan.lane != lane_for_colour(colour)
    assert plan.pre_ramp == (ms.PRE_RAMP_X, plan.lane)


def test_an_explicit_lane_still_wins():
    """--lane / the lane parameter is an operator override; keep it."""
    plan = ms.MissionPlan('red', lane=0.4, region_map=ROTATED)
    assert plan.lane == 0.4


def test_the_align_check_measures_against_the_episode_lane():
    """ALIGN_FOR_CLIMB's datum is plan.lane, so it follows the map."""
    plan = ms.MissionPlan('yellow', region_map=ROTATED)
    assert plan.lane == lane_for_region('lane_1')
    # the frozen yellow lane is 1.5 m away: far outside the tolerance
    assert abs(lane_for_colour('yellow') - plan.lane) > plan.lane_tolerance


# ── the executive ────────────────────────────────────────────────────────
@pytest.fixture
def mapped():
    """Yield a ROS context whose nodes are all given ROTATED as region_map."""
    rclpy.init(args=['--ros-args', '-p',
                     f'region_map:={format_region_map(ROTATED)}'])
    yield
    rclpy.shutdown()


@pytest.fixture
def plain():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_the_executive_defaults_to_the_frozen_table(plain):
    node = mx.MissionExecutive(colour='blue')
    try:
        assert node.region_map == {}
        assert node.plan.lane == lane_for_colour('blue')
    finally:
        node.destroy_node()


def test_the_executive_takes_its_lane_from_the_region_map(mapped):
    node = mx.MissionExecutive(colour='blue')
    try:
        assert node.region_map == ROTATED
        assert node.plan.lane == lane_for_region('lane_4')
        assert node.plan.pre_ramp[1] == lane_for_region('lane_4')
    finally:
        node.destroy_node()


def test_a_colour_change_keeps_the_region_map(mapped):
    """_on_colour rebuilds the plan; it must not fall back to the table."""
    node = mx.MissionExecutive()
    try:
        node._on_colour(String(data='red'))
        assert node.plan.lane == lane_for_region('lane_2')
        assert node.plan.region_map == ROTATED
    finally:
        node.destroy_node()


def test_an_illegal_region_map_refuses_to_start():
    rclpy.init(args=['--ros-args', '-p', 'region_map:=red=lane_9'])
    try:
        with pytest.raises(ValueError, match='unknown region'):
            mx.MissionExecutive(colour='red')
    finally:
        rclpy.shutdown()


def test_the_executive_still_publishes_no_velocity(mapped):
    """Stage C adds an input; it must not add a wheel publisher."""
    node = mx.MissionExecutive(colour='green')
    try:
        topics = {name for name, _ in
                  node.get_publisher_names_and_types_by_node(
                      node.get_name(), node.get_namespace())}
        assert '/diff_drive_controller/cmd_vel' not in topics
        assert not any('cmd_vel' in t for t in topics)
    finally:
        node.destroy_node()


# ── the launch file ──────────────────────────────────────────────────────
def _launch_module():
    spec = importlib.util.spec_from_file_location('coco_mission_launch',
                                                  LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _resolve(**configs):
    context = LaunchContext()
    base = {'episode_level': 'fixed', 'episode_seed': '0',
            'episode_manifest': '', 'target_colour': 'blue'}
    base.update({k: str(v) for k, v in configs.items()})
    context.launch_configurations.update(base)
    actions = _launch_module().resolve_mission_episode(context)
    for action in actions:
        if isinstance(action, SetLaunchConfiguration):
            action.execute(context)
    return context, actions


def test_the_launch_file_changes_nothing_by_default():
    context, actions = _resolve()
    assert actions == []
    assert 'coco_episode_region_map' not in context.launch_configurations
    assert context.launch_configurations['target_colour'] == 'blue'


def test_the_launch_file_hands_the_mission_names_for_an_episode():
    context, _ = _resolve(episode_level='colours', episode_seed='1827',
                          target_colour='red')
    spec = generate_episode(seed=1827, level='colours',
                            requested_colour='red')
    region_map = context.launch_configurations['coco_episode_region_map']
    assert region_map == compat_mission_inputs(spec)['region_map']
    assert context.launch_configurations['target_colour'] == 'red'
    for t in spec.targets:
        assert repr(t.x) not in region_map and repr(t.y) not in region_map


def test_a_manifest_sets_the_requested_colour(tmp_path):
    spec = generate_episode(seed=3, level='positions',
                            requested_colour='yellow')
    path = tmp_path / 'm.json'
    path.write_text(spec.to_json())
    context, _ = _resolve(episode_manifest=path, target_colour='blue')
    assert context.launch_configurations['target_colour'] == 'yellow'
    assert (context.launch_configurations['coco_episode_region_map']
            == format_region_map(spec.region_map()))


def test_only_the_executive_and_ramp_driver_are_given_the_map():
    body = open(LAUNCH).read()
    assert body.count("'region_map': ParameterValue(") == 2
    assert 'value_type=str' in body
    # no Node is ever handed the manifest path or the manifest itself
    for node_block in body.split('Node(')[1:]:
        assert 'episode_manifest' not in node_block
        assert 'manifest' not in node_block


def test_the_manifest_never_reaches_a_launch_configuration(tmp_path):
    """What the resolver sets is a colour and names; no pose survives."""
    spec = generate_episode(seed=9, level='positions')
    path = tmp_path / 'm.json'
    path.write_text(spec.to_json())
    context, _ = _resolve(episode_manifest=path)
    changed = {k: context.launch_configurations[k]
               for k in ('target_colour', 'coco_episode_region_map')}
    text = json.dumps(changed)
    for t in spec.targets:
        for value in (t.x, t.y, t.z):
            assert repr(value) not in text
            assert f'{value:.2f}' not in text
