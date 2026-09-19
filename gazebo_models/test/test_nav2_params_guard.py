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

"""Guard the shipped nav2_params.yaml against silent regressions (C2-NAV.39).

The default parameter file once carried a value measured and REJECTED in
C2-NAV.2 (BaseObstacle.scale 2.0) for 36 experiments, while the validated
values lived only in derivative files under docs/data/. Every assertion
below is either a value accepted by a live measurement or a safety gate
that must not move without an explicit decision.
"""
import math
import os

import yaml

PARAMS = os.path.join(os.path.dirname(__file__), '..', 'config',
                      'nav2_params.yaml')


def _params():
    with open(PARAMS) as f:
        return yaml.safe_load(f)


def _node(doc, *path):
    cur = doc
    for key in path:
        cur = cur[key]
    return cur


def test_local_costmap_cost_scaling_factor_is_the_validated_65():
    doc = _params()
    infl = _node(doc, 'local_costmap', 'local_costmap', 'ros__parameters',
                 'inflation_layer')
    assert infl['cost_scaling_factor'] == 65.0
    assert infl['inflation_radius'] == 0.5


def test_global_costmap_inflation_is_unchanged():
    infl = _node(_params(), 'global_costmap', 'global_costmap',
                 'ros__parameters', 'inflation_layer')
    assert infl['cost_scaling_factor'] == 5.0
    assert infl['inflation_radius'] == 0.5


def test_navigate_through_poses_uses_the_multi_pose_tree():
    bt = _node(_params(), 'bt_navigator', 'ros__parameters')
    assert bt['default_nav_through_poses_bt_xml'].endswith(
        '/navigate_through_poses_w_replanning_and_recovery.xml')
    assert bt['default_nav_to_pose_bt_xml'].endswith(
        '/navigate_to_pose_w_replanning_and_recovery.xml')


def test_base_obstacle_scale_is_not_the_rejected_value():
    follow = _node(_params(), 'controller_server', 'ros__parameters',
                   'FollowPath')
    assert follow['BaseObstacle.scale'] == 8.0


def test_goal_tolerances_are_not_widened():
    ctrl = _node(_params(), 'controller_server', 'ros__parameters')
    assert ctrl['FollowPath']['xy_goal_tolerance'] == 0.05
    assert ctrl['goal_checker']['xy_goal_tolerance'] == 0.25
    assert ctrl['goal_checker']['yaw_goal_tolerance'] == 0.25


def test_collision_monitor_safety_gates_are_at_baseline():
    cm = _node(_params(), 'collision_monitor', 'ros__parameters')
    assert cm['polygons'] == ['PolygonStop', 'PolygonSlow', 'PolygonLimit',
                              'FootprintApproach']
    stop = cm['PolygonStop']
    assert stop['type'] == 'circle'
    assert stop['radius'] == 0.25
    assert stop['action_type'] == 'stop'
    assert stop['min_points'] == 4
    assert stop['enabled'] is True
    slow = cm['PolygonSlow']
    assert slow['action_type'] == 'slowdown'
    assert slow['min_points'] == 4
    assert slow['slowdown_ratio'] == 0.3
    assert slow['enabled'] is True
    limit = cm['PolygonLimit']
    assert limit['linear_limit'] == 0.4
    assert limit['angular_limit'] == 0.5
    assert limit['enabled'] is True
    approach = cm['FootprintApproach']
    assert approach['min_points'] == 6
    assert approach['time_before_collision'] == 2.0
    assert approach['enabled'] is True


def test_amcl_resamples_every_update():
    amcl = _node(_params(), 'amcl', 'ros__parameters')
    assert amcl['resample_interval'] == 1


# C2-NAV.48. Costmap2DROS builds a 16-gon of circumradius robot_radius, pads it
# by footprint_padding, and LayeredCostmap takes the apothem, so the inflation
# layer's inscribed radius is not robot_radius itself. C2-NAV.0 measured it as
# 0.205879 m for robot_radius 0.20.
NAV2_FOOTPRINT_PADDING_DEFAULT = 0.01
_APOTHEM_16GON = math.cos(math.pi / 16)


def _inscribed_radius(robot_radius, padding=NAV2_FOOTPRINT_PADDING_DEFAULT):
    return (robot_radius + padding) * _APOTHEM_16GON


def test_apothem_model_matches_the_c2nav0_measurement():
    assert abs(_inscribed_radius(0.20) - 0.205879) < 1e-4


def test_local_costmap_inscribed_radius_exceeds_polygonstop_radius():
    # The two must agree on which poses the planner may choose. Where the
    # inscribed radius is the smaller of the two, poses in between are cheap to
    # the planner and held by the monitor: C2-NAV.46 r2_blue sat at 0.2486 m for
    # 595.5 s, r3_blue passed the same obstacle at 0.2724 m and completed.
    doc = _params()
    local = _node(doc, 'local_costmap', 'local_costmap', 'ros__parameters')
    stop = _node(doc, 'collision_monitor', 'ros__parameters', 'PolygonStop')
    inscribed = _inscribed_radius(local['robot_radius'])
    assert inscribed > stop['radius'], (
        f'robot_radius {local["robot_radius"]} gives inscribed radius '
        f'{inscribed:.6f} m, below PolygonStop {stop["radius"]} m')


def test_local_costmap_robot_radius_is_the_c2nav48_value():
    local = _node(_params(), 'local_costmap', 'local_costmap',
                  'ros__parameters')
    assert local['robot_radius'] == 0.25
    assert abs(_inscribed_radius(0.25) - 0.255004) < 1e-5
    # the measured r2_blue pose is covered at 0.25 and was not at 0.20
    assert 0.2486 < _inscribed_radius(0.25)
    assert 0.2486 > _inscribed_radius(0.20)


def test_global_costmap_robot_radius_is_unchanged_by_c2nav48():
    # Measured asymmetry, not an oversight. At cost_scaling_factor 5.0 the
    # global costmap already prices r2_blue's 0.2486 m pose at ~203 of 254; the
    # local costmap's 65.0 prices the same pose at ~16.
    doc = _params()
    glob = _node(doc, 'global_costmap', 'global_costmap', 'ros__parameters')
    assert glob['robot_radius'] == 0.20
    assert glob['inflation_layer']['cost_scaling_factor'] == 5.0
