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
"""Pure helpers of amcl_diag_swap.py; no ROS graph needed."""
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from amcl_diag_swap import (AMCL_NAME, amcl_params, flatten,  # noqa: E402, I100
                            load_command, main, param_arg, parse_components,
                            parse_lifecycle_state, parse_param_get, PLUGIN_CLASS,
                            SwapError)


@pytest.mark.parametrize('text,want', [
    ('active [3]\n', 'active'),
    ('unconfigured [1]\n', 'unconfigured'),
    ('inactive [2]\n', 'inactive'),
    ('configuring [10]\n', 'configuring'),
    ('Node not found\n', None),
])
def test_parse_lifecycle_state(text, want):
    assert parse_lifecycle_state(text) == want


@pytest.mark.parametrize('value', [
    True, False, 0, 2000, 0.2, 2.0, 1e-05, -3.5e+20, 'likelihood_field',
    'true', '1.0', 'nav2_amcl::DifferentialMotionModel', '', [1.0, 2.0],
])
def test_param_arg_round_trips_value_and_type(value):
    back = yaml.safe_load(param_arg(value))
    assert back == value
    assert type(back) is type(value)


def test_flatten_uses_dotted_names_for_nested_blocks():
    flat = flatten({'alpha1': 0.2, 'initial_pose': {'x': -2.0, 'yaw': 0.0}})
    assert flat == {'alpha1': 0.2, 'initial_pose.x': -2.0, 'initial_pose.yaw': 0.0}


def test_parse_components_reads_both_list_formats():
    all_containers = '/nav2_container\n  1  /map_server\n  3  /amcl\n'
    one_container = '1  /map_server\n3  /amcl\n'
    want = [(1, '/map_server'), (3, '/amcl')]
    assert parse_components(all_containers) == want
    assert parse_components(one_container) == want


@pytest.mark.parametrize('text,want', [
    ('Boolean value is: True\n', True),
    ('Integer value is: 1\n', 1),
    ('Double value is: 0.25\n', 0.25),
    ('String value is: /tmp/diag.jsonl\n', '/tmp/diag.jsonl'),
    ('String value is: \n', ''),
])
def test_parse_param_get(text, want):
    assert parse_param_get(text) == want


def test_parse_param_get_rejects_unknown_output():
    with pytest.raises(SwapError):
        parse_param_get('Parameter not set.')


def test_load_command_is_deterministic_and_names_the_fork():
    cmd = load_command('/nav2_container', {'b': 1, 'a': True})
    assert cmd[:6] == ['ros2', 'component', 'load', '/nav2_container',
                       'coco_nav_diag', PLUGIN_CLASS]
    assert cmd[6:] == ['-p', 'a:=true', '-p', 'b:=1']


def test_amcl_params_reads_the_amcl_block(tmp_path):
    path = tmp_path / 'p.yaml'
    path.write_text(yaml.safe_dump({
        'amcl': {'ros__parameters': {'resample_interval': 1,
                                     'initial_pose': {'x': -2.0}}},
        'map_server': {'ros__parameters': {'yaml_filename': 'x'}},
    }))
    assert amcl_params(str(path)) == {'resample_interval': 1, 'initial_pose.x': -2.0}


def test_amcl_params_without_block_fails(tmp_path):
    path = tmp_path / 'p.yaml'
    path.write_text('map_server: {}\n')
    with pytest.raises(SwapError):
        amcl_params(str(path))


def test_repo_params_file_loads_every_amcl_parameter():
    params = os.path.join(os.path.dirname(__file__), '..', '..', 'gazebo_models',
                          'config', 'nav2_params.yaml')
    if not os.path.exists(params):
        pytest.skip('gazebo_models source tree not beside this package')
    flat = amcl_params(params)
    assert flat['resample_interval'] == 1
    for key, value in flat.items():
        assert yaml.safe_load(param_arg(value)) == value, key


def test_enabled_load_without_output_path_is_refused(tmp_path, capsys):
    path = tmp_path / 'p.yaml'
    path.write_text(yaml.safe_dump({'amcl': {'ros__parameters': {'a': 1}}}))
    assert main(['load', '--params', str(path), '--dry-run']) == 1
    assert 'diag-output' in capsys.readouterr().err


def test_dry_run_prints_the_full_procedure(tmp_path, capsys):
    path = tmp_path / 'p.yaml'
    path.write_text(yaml.safe_dump({'amcl': {'ros__parameters': {'a': 1}}}))
    assert main(['load', '--params', str(path), '--diag-output', '/tmp/d.jsonl',
                 '--dry-run']) == 0
    out = capsys.readouterr().out
    for fragment in ('component unload', 'component load', 'lifecycle set /amcl configure',
                     'lifecycle set /amcl activate', f'param get {AMCL_NAME} diag_enabled'):
        assert fragment in out
