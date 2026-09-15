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

"""Experiment-file resolution, its guards, and live readback helpers (C2-NAV.39)."""
import json
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from nav_params_overlay import (expected_value, ExperimentError,  # noqa: E402
                                LIVE_CHECKS, load_experiment, lookup_param,
                                merge_overrides, MERGED_NAME, parse_param_get,
                                resolve, RESOLVED_NAME, values_match)

HERE = os.path.dirname(__file__)
BASE = os.path.join(HERE, '..', 'config', 'nav2_params.yaml')
EXPERIMENTS = os.path.join(HERE, '..', 'config', 'experiments')

BASE_DOC = {
    'controller_server': {'ros__parameters': {
        'FollowPath': {'BaseObstacle.scale': 8.0, 'critics': ['a', 'b']},
        'controller_frequency': 20.0,
    }},
    'collision_monitor': {'ros__parameters': {'PolygonStop': {'min_points': 4}}},
}


def _write(tmp_path, name, doc):
    path = tmp_path / name
    path.write_text(yaml.safe_dump(doc))
    return str(path)


def test_override_changes_only_the_named_value():
    merged, changes = merge_overrides(BASE_DOC, {
        'controller_server': {'ros__parameters': {'FollowPath': {'BaseObstacle.scale': 4}}}})
    assert changes == [('controller_server.ros__parameters.FollowPath.BaseObstacle.scale',
                        8.0, 4)]
    assert merged['controller_server']['ros__parameters']['controller_frequency'] == 20.0
    assert BASE_DOC['controller_server']['ros__parameters']['FollowPath'][
        'BaseObstacle.scale'] == 8.0


def test_unknown_key_is_an_error_not_a_silent_no_op():
    with pytest.raises(ExperimentError, match='does not exist'):
        merge_overrides(BASE_DOC, {'controller_server': {'ros__parameters': {
            'FollowPath': {'BaseObstacle.scael': 4.0}}}})


def test_safety_gate_requires_explicit_permission():
    override = {'collision_monitor': {'ros__parameters': {'PolygonStop': {'min_points': 7}}}}
    with pytest.raises(ExperimentError, match='safety gate'):
        merge_overrides(BASE_DOC, override)
    _, changes = merge_overrides(BASE_DOC, override, allow_safety_change=True)
    assert len(changes) == 1


@pytest.mark.parametrize('override,match', [
    ({'controller_server': {'ros__parameters': {'FollowPath': 1.0}}}, 'block with a scalar'),
    ({'controller_server': {'ros__parameters': {'controller_frequency': {'x': 1}}}},
     'scalar with a block'),
    ({'controller_server': {'ros__parameters': {'controller_frequency': 'fast'}}},
     'changes type'),
])
def test_structure_and_type_changes_are_refused(override, match):
    with pytest.raises(ExperimentError, match=match):
        merge_overrides(BASE_DOC, override)


def test_experiment_rejects_unknown_keys_and_unsafe_names(tmp_path):
    with pytest.raises(ExperimentError, match='unknown key'):
        load_experiment(_write(tmp_path, 'a.yaml', {'name': 'x', 'goal': 'here'}))
    with pytest.raises(ExperimentError, match='unknown key'):
        load_experiment(_write(tmp_path, 'b.yaml', {'name': 'x', 'bench': {'waypoint': 1}}))
    with pytest.raises(ExperimentError, match='nav2_'):
        load_experiment(_write(tmp_path, 'c.yaml', {'name': 'my_nav2_test'}))


def test_resolve_without_overrides_points_at_the_base_file(tmp_path):
    exp = _write(tmp_path, 'e.yaml', {'name': 'plain'})
    base = _write(tmp_path, 'base.yaml', BASE_DOC)
    out = tmp_path / 'run'
    resolved = resolve(exp, base, str(out))
    assert resolved['params_file'] == os.path.abspath(base)
    assert resolved['changes'] == []
    assert not (out / MERGED_NAME).exists()
    assert json.loads((out / RESOLVED_NAME).read_text())['name'] == 'plain'


def test_resolve_with_overrides_writes_a_merged_file(tmp_path):
    exp = _write(tmp_path, 'e.yaml', {'name': 'variant', 'nav2_overrides': {
        'controller_server': {'ros__parameters': {'controller_frequency': 10.0}}}})
    base = _write(tmp_path, 'base.yaml', BASE_DOC)
    resolved = resolve(exp, base, str(tmp_path / 'run'))
    assert resolved['params_file'].endswith(MERGED_NAME)
    with open(resolved['params_file']) as f:
        merged = yaml.safe_load(f)
    assert merged['controller_server']['ros__parameters']['controller_frequency'] == 10.0
    assert resolved['params_sha256'] != resolved['base_params_sha256']


@pytest.mark.parametrize('name,diag', [('baseline', False), ('baseline_amcl_diag', True)])
def test_committed_experiments_resolve_against_the_shipped_params(tmp_path, name, diag):
    resolved = resolve(os.path.join(EXPERIMENTS, f'{name}.yaml'), BASE, str(tmp_path))
    assert resolved['name'] == name
    assert resolved['changes'] == []
    assert resolved['amcl_diag']['enabled'] is diag
    assert resolved['bench'] == {'repeats': 1, 'timeout': 75, 'only': None}


def test_lookup_param_handles_dots_inside_keys():
    block = {'FollowPath': {'BaseObstacle.scale': 8.0, 'xy_goal_tolerance': 0.05}}
    assert lookup_param(block, 'FollowPath.BaseObstacle.scale') == 8.0
    assert lookup_param(block, 'FollowPath.xy_goal_tolerance') == 0.05
    with pytest.raises(KeyError):
        lookup_param(block, 'FollowPath.missing')


def test_every_live_check_resolves_in_the_shipped_params():
    with open(BASE) as f:
        doc = yaml.safe_load(f)
    for _, yaml_path, param in LIVE_CHECKS:
        expected_value(doc, yaml_path, param)


@pytest.mark.parametrize('expected,live,ok', [
    (65.0, 65.0, True),
    (4, 4, True),
    (0.3, 1.0, False),
    ('$(find-pkg-share nav2_bt_navigator)/behavior_trees/navigate_through_poses_w_replanning'
     '_and_recovery.xml',
     '/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_'
     'replanning_and_recovery.xml', True),
    ('$(find-pkg-share nav2_bt_navigator)/behavior_trees/navigate_through_poses_w_replanning'
     '_and_recovery.xml',
     '/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_'
     'and_recovery.xml', False),
    (8.0, None, False),
])
def test_values_match(expected, live, ok):
    assert values_match(expected, live) is ok


def test_parse_param_get():
    assert parse_param_get('Double value is: 65.0\n') == 65.0
    assert parse_param_get('Integer value is: 4\n') == 4
    assert parse_param_get('Parameter not set.') is None
