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
import subprocess
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from nav_params_overlay import (apply_goals, check_command_path,  # noqa: E402
                                check_topology, CHAIN_TOPICS, GATED_TOPIC,
                                parse_topic_endpoints, RAW_NAV_TOPIC,
                                expected_value, ExperimentError, LIVE_CHECKS,
                                load_experiment, lookup_param, merge_overrides,
                                MERGED_NAME, parse_arbiter_status, parse_param_get,
                                parse_topic_info, resolve, RESOLVED_NAME,
                                TOPOLOGIES, tour_goals, values_match, verify_goals,
                                WHEEL_TOPIC)

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


@pytest.mark.parametrize('name,diag', [('baseline', False), ('baseline_topology_b', False)])
def test_committed_experiments_resolve_against_the_shipped_params(tmp_path, name, diag):
    resolved = resolve(os.path.join(EXPERIMENTS, f'{name}.yaml'), BASE, str(tmp_path))
    assert resolved['name'] == name
    assert resolved['changes'] == []
    assert resolved['amcl_diag']['enabled'] is diag
    assert resolved['bench'] == {'repeats': 1, 'timeout': 75, 'only': None, 'goals': {}}
    assert resolved['goal_changes'] == [] and resolved['goal_args'] == []
    assert resolved['tour_goals'] == tour_goals()


# C2-NAV.40 -- bench.goals ------------------------------------------------

def test_tour_goals_reads_the_committed_tour_without_ros():
    goals = tour_goals()
    assert list(goals) == ['open_space', 'wall_adjacent', 'wall_parallel', 'obstacle_corner',
                           'corridor_gate', 'enclosure_entry', 'enclosure_exit']
    assert goals['enclosure_entry'] == [-3.45, 2.95]
    # other test modules import rclpy into this process, so prove it in a
    # fresh interpreter where importing rclpy is an error
    scripts = os.path.join(HERE, '..', 'scripts')
    code = ("import sys; sys.modules['rclpy'] = None; sys.path.insert(0, sys.argv[1]); "
            "import nav_params_overlay as m; print(m.tour_goals()['enclosure_entry'])")
    res = subprocess.run([sys.executable, '-P', '-c', code, scripts],
                         capture_output=True, text=True, timeout=60)
    assert res.returncode == 0, res.stderr
    assert res.stdout.strip() == '[-3.45, 2.95]'


# C2-NAV.40's entry_corridor_centre.yaml, REJECTED in simulation and removed
# from experiments/ (it is in git at 321df0e). Kept here verbatim so the goal
# override path stays covered.
ENTRY_CORRIDOR_CENTRE = {
    'name': 'entry_corridor_centre',
    'description': 'baseline tour with enclosure_entry\'s goal at the C2-NAV.7 corridor centre',
    'bench': {'goals': {'enclosure_entry': [-3.575, 2.95]}},
}


def _entry_corridor_centre(tmp_path):
    return _write(tmp_path, 'entry_corridor_centre.yaml', ENTRY_CORRIDOR_CENTRE)


def test_entry_corridor_centre_moves_only_the_entry_goal(tmp_path):
    exp = _entry_corridor_centre(tmp_path)
    with open(exp) as f:
        doc = yaml.safe_load(f)
    # the file carries the goal and nothing else that could change a run
    assert set(doc) == {'name', 'description', 'bench'}
    assert doc['bench'] == {'goals': {'enclosure_entry': [-3.575, 2.95]}}

    resolved = resolve(exp, BASE, str(tmp_path))
    baseline = resolve(os.path.join(EXPERIMENTS, 'baseline.yaml'), BASE, str(tmp_path / 'b'))
    assert resolved['changes'] == []
    assert resolved['params_file'] == os.path.abspath(BASE)
    assert resolved['params_sha256'] == baseline['params_sha256']
    assert not (tmp_path / MERGED_NAME).exists()
    assert resolved['amcl_diag'] == baseline['amcl_diag'] == {'enabled': False}
    assert resolved['nav2_overrides'] == {} and resolved['allow_safety_change'] is False
    assert {k: v for k, v in resolved['bench'].items() if k != 'goals'} == \
        {k: v for k, v in baseline['bench'].items() if k != 'goals'}
    assert resolved['goal_args'] == ['enclosure_entry:-3.575,2.95']
    assert resolved['goal_changes'] == [
        {'scenario': 'enclosure_entry', 'old': [-3.45, 2.95], 'new': [-3.575, 2.95]}]
    differ = {n for n in resolved['tour_goals']
              if resolved['tour_goals'][n] != baseline['tour_goals'][n]}
    assert differ == {'enclosure_entry'}


def test_goal_args_are_what_nav_bench_parses():
    # nav_bench's own parser, re-stated: NAME:X,Y with float() on each half.
    _, args = apply_goals({'enclosure_entry': [-3.575, 2.95]}, tour_goals())
    name, _, xy = args[0].partition(':')
    assert name == 'enclosure_entry'
    assert [float(v) for v in xy.split(',')] == [-3.575, 2.95]


@pytest.mark.parametrize('goals,match', [
    ({'enclosure_entrance': [-3.575, 2.95]}, 'not in TOUR'),
    ({'enclosure_entry': [-3.575]}, r'\[x, y\]'),
    ({'enclosure_entry': [-3.575, 'north']}, r'\[x, y\]'),
    ({'enclosure_entry': [True, 2.95]}, r'\[x, y\]'),
    ({'enclosure_entry': [float('nan'), 2.95]}, r'\[x, y\]'),
    ({'enclosure_entry': '-3.575,2.95'}, r'\[x, y\]'),
    (['enclosure_entry'], 'mapping'),
])
def test_bad_goals_are_refused_before_launch(tmp_path, goals, match):
    exp = _write(tmp_path, 'g.yaml', {'name': 'g', 'bench': {'goals': goals}})
    with pytest.raises(ExperimentError, match=match):
        resolve(exp, BASE, str(tmp_path / 'run'))


def _legs(goal_entry):
    committed = tour_goals()
    legs = [{'scenario': n, 'rep': 0, 'goal_world': list(xy)} for n, xy in committed.items()]
    for leg in legs:
        if leg['scenario'] == 'enclosure_entry':
            leg['goal_world'] = goal_entry
    return {'legs': legs}


def test_verify_goals_accepts_the_requested_goal(tmp_path):
    resolved = resolve(_entry_corridor_centre(tmp_path), BASE, str(tmp_path / 'run'))
    mismatches, lines = verify_goals(resolved, _legs([-3.575, 2.95]))
    assert mismatches == 0
    assert any('enclosure_entry' in ln and '[overridden]' in ln for ln in lines)


def test_verify_goals_catches_an_override_that_was_not_driven(tmp_path):
    resolved = resolve(_entry_corridor_centre(tmp_path), BASE, str(tmp_path / 'run'))
    # the committed goal was driven: that leg mismatches, AND the override
    # was never exercised
    mismatches, lines = verify_goals(resolved, _legs([-3.45, 2.95]))
    assert mismatches == 2
    assert sum(ln.startswith('MISMATCH') for ln in lines) == 2
    # a leg that never started proves nothing about the override
    doc = _legs([-3.575, 2.95])
    del doc['legs'][5]['goal_world']
    mismatches, lines = verify_goals(resolved, doc)
    assert mismatches == 1
    assert any('overridden but no leg' in ln for ln in lines)


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


# C2-NAV.41 -- topology ----------------------------------------------------

TOPIC_INFO_ARBITER = """Type: geometry_msgs/msg/TwistStamped

Publisher count: 1

Node name: cmd_vel_arbiter
Node namespace: /
Topic type: geometry_msgs/msg/TwistStamped
Endpoint type: PUBLISHER
GID: 01.0f.ab.cd
QoS profile:
  Reliability: RELIABLE
  Durability: VOLATILE

Subscription count: 1

Node name: diff_drive_controller
Node namespace: /
Topic type: geometry_msgs/msg/TwistStamped
Endpoint type: SUBSCRIPTION
GID: 01.0f.ef.01
QoS profile:
  Reliability: RELIABLE
"""


def test_parse_topic_info_returns_publishers_not_subscribers():
    # the same 'Node name:' key introduces both blocks, so a parser that
    # keys off the 'Publisher count:' header alone reports the controller
    # as a second publisher and the sole-publisher check passes wrongly
    assert parse_topic_info(TOPIC_INFO_ARBITER) == ['cmd_vel_arbiter']
    assert parse_topic_info('') == []


def test_parse_topic_info_sees_two_publishers():
    both = TOPIC_INFO_ARBITER.replace(
        'Subscription count: 1',
        'Node name: cmd_vel_relay\nNode namespace: /\nEndpoint type: PUBLISHER\n\n'
        'Subscription count: 1')
    assert parse_topic_info(both) == ['cmd_vel_arbiter', 'cmd_vel_relay']


@pytest.mark.parametrize('text,want', [
    ('data: mode=nav active=nav teleop=-- nav=0.05 rl=-- approach=--\n---',
     {'mode': 'nav', 'active': 'nav', 'teleop': '--', 'nav': '0.05',
      'rl': '--', 'approach': '--'}),
    ("data: 'mode=idle active=none'\n---", {'mode': 'idle', 'active': 'none'}),
    ('mode=nav active=nav', {'mode': 'nav', 'active': 'nav'}),
    ('', {}),
])
def test_parse_arbiter_status(text, want):
    assert parse_arbiter_status(text) == want


def test_check_topology_accepts_each_wired_path():
    bad, _ = check_topology('A', ['cmd_vel_relay'], {})
    assert bad == 0
    bad, _ = check_topology('B', ['cmd_vel_arbiter'], {'mode': 'nav', 'active': 'nav'})
    assert bad == 0


@pytest.mark.parametrize('topology,publishers,status,match', [
    # an arbiter that never started leaves the relay driving: a topology-A
    # tour wearing topology B's label
    ('B', ['cmd_vel_relay'], {}, 'owner'),
    # the failure CLAUDE.md rule 5 is about: the robot tracks the average
    ('B', ['cmd_vel_arbiter', 'cmd_vel_relay'], {'mode': 'nav'}, 'publisher count'),
    ('A', ['cmd_vel_relay', 'cmd_vel_arbiter'], {}, 'publisher count'),
    # idle forwards nothing at all -- C2-NAV.21 measured 0.000 m travelled
    ('B', ['cmd_vel_arbiter'], {'mode': 'idle'}, 'arbiter mode'),
    ('B', ['cmd_vel_arbiter'], {}, 'arbiter mode'),
    # a stray arbiter in what claims to be topology A
    ('A', ['cmd_vel_relay'], {'mode': 'nav'}, 'no arbiter running'),
])
def test_check_topology_catches_a_miswired_path(topology, publishers, status, match):
    bad, lines = check_topology(topology, publishers, status)
    assert bad >= 1
    assert any(ln.startswith('MISMATCH') and match in ln for ln in lines), lines


def test_topology_a_check_has_a_positive_control():
    # "no arbiter" succeeds on seeing NOTHING, so a graph that cannot be read
    # must fail rather than pass: an empty publisher list fails the first two
    # checks (CLAUDE.md -- any check whose success condition is "we saw
    # nothing" must first prove it can see something)
    bad, _ = check_topology('A', [], {})
    assert bad == 2


def test_topology_defaults_to_a_and_normalises(tmp_path):
    assert load_experiment(_write(tmp_path, 'a.yaml', {'name': 'x'}))['topology'] == 'A'
    assert load_experiment(
        _write(tmp_path, 'b.yaml', {'name': 'x', 'topology': 'b'}))['topology'] == 'B'
    assert set(TOPOLOGIES) == {'A', 'B'}


@pytest.mark.parametrize('topology', ['C', '', 'arbiter', 1, True, None])
def test_bad_topology_is_refused_before_launch(tmp_path, topology):
    exp = _write(tmp_path, 't.yaml', {'name': 't', 'topology': topology})
    with pytest.raises(ExperimentError, match='topology'):
        load_experiment(exp)


def test_baseline_topology_b_differs_from_baseline_only_in_topology(tmp_path):
    b = resolve(os.path.join(EXPERIMENTS, 'baseline_topology_b.yaml'),
                BASE, str(tmp_path / 'b'))
    a = resolve(os.path.join(EXPERIMENTS, 'baseline.yaml'), BASE, str(tmp_path / 'a'))
    assert (a['topology'], b['topology']) == ('A', 'B')
    # not one navigation input moves, so the two arms are comparable leg by leg
    assert b['changes'] == [] and b['goal_changes'] == [] and b['goal_args'] == []
    assert b['params_file'] == a['params_file'] == os.path.abspath(BASE)
    assert b['params_sha256'] == a['params_sha256']
    assert b['bench'] == a['bench']
    assert b['tour_goals'] == a['tour_goals'] == tour_goals()
    assert b['amcl_diag'] == a['amcl_diag'] == {'enabled': False}
    assert b['nav2_overrides'] == {} and b['allow_safety_change'] is False
    assert not (tmp_path / 'b' / MERGED_NAME).exists()
    assert json.loads((tmp_path / 'b' / RESOLVED_NAME).read_text())['topology'] == 'B'


def test_wheel_topic_is_the_one_the_arbiter_publishes():
    # the arbiter's output_topic default and this check must not drift apart
    arbiter = os.path.join(HERE, '..', '..', 'custom_teleop', 'custom_teleop',
                           'cmd_vel_arbiter.py')
    with open(arbiter) as f:
        assert f"declare_parameter('output_topic', '{WHEEL_TOPIC}')" in f.read()


# ── C2-NAV.42: the command chain, link by link ──────────────────────────────
def _endpoints(topology='B', looped=False):
    """The live graph's chain as C2-M5.0 recorded it, rewired per topology."""
    raw_pubs = ['behavior_server', 'controller_server']
    raw_subs = ['velocity_smoother']
    gated = ([], [])
    if topology == 'B' and looped:
        raw_pubs.append('cmd_vel_relay')
        raw_subs.append('cmd_vel_arbiter')
    elif topology == 'B':
        gated = (['cmd_vel_relay'], ['cmd_vel_arbiter'])
    return {
        RAW_NAV_TOPIC: (sorted(raw_pubs), sorted(raw_subs)),
        '/cmd_vel_smoothed': (['velocity_smoother'], ['collision_monitor']),
        '/cmd_vel': (['collision_monitor', 'docking_server'], ['cmd_vel_relay']),
        GATED_TOPIC: gated,
    }


TOPIC_INFO_RAW_NAV = """Type: geometry_msgs/msg/TwistStamped

Publisher count: 3

Node name: controller_server
Node namespace: /
Endpoint type: PUBLISHER

Node name: behavior_server
Node namespace: /
Endpoint type: PUBLISHER

Node name: behavior_server
Node namespace: /
Endpoint type: PUBLISHER

Subscription count: 1

Node name: velocity_smoother
Node namespace: /
Endpoint type: SUBSCRIPTION
"""


def test_the_chain_covers_every_link_to_the_relay_output():
    assert CHAIN_TOPICS == (RAW_NAV_TOPIC, '/cmd_vel_smoothed', '/cmd_vel', GATED_TOPIC)


def test_parse_topic_endpoints_splits_and_deduplicates():
    pubs, subs = parse_topic_endpoints(TOPIC_INFO_RAW_NAV)
    assert pubs == ['behavior_server', 'controller_server']
    assert subs == ['velocity_smoother']
    assert parse_topic_endpoints('') == ([], [])


@pytest.mark.parametrize('topology', ['A', 'B'])
def test_check_command_path_accepts_the_fixed_wiring(topology):
    bad, lines = check_command_path(topology, _endpoints(topology))
    assert bad == 0, lines


def test_check_command_path_catches_the_cmd_vel_nav_loop():
    bad, lines = check_command_path('B', _endpoints('B', looped=True))
    mismatched = [ln for ln in lines if ln.startswith('MISMATCH')]
    assert any('relay does not publish' in ln for ln in mismatched), lines
    assert any('arbiter does not read' in ln for ln in mismatched), lines
    assert any(GATED_TOPIC in ln for ln in mismatched), lines


def test_check_command_path_has_a_positive_control():
    # "nothing of ours on /cmd_vel_nav" succeeds on seeing nothing, so an
    # unreadable graph must fail the controller and smoother checks first.
    bad, lines = check_command_path('B', {})
    assert bad >= 2
    assert any('controller publishes' in ln and ln.startswith('MISMATCH') for ln in lines)


def test_check_command_path_names_an_unknown_monitor_peer():
    endpoints = _endpoints('B')
    endpoints['/cmd_vel'] = (['collision_monitor', 'teleop_wheels_node'], ['cmd_vel_relay'])
    bad, lines = check_command_path('B', endpoints)
    assert bad == 1
    assert any(ln.startswith('MISMATCH') and '/cmd_vel publishers' in ln for ln in lines)


def test_check_command_path_rejects_a_smoother_bypass():
    endpoints = _endpoints('B')
    endpoints['/cmd_vel_smoothed'] = (['controller_server', 'velocity_smoother'],
                                      ['collision_monitor'])
    bad, _ = check_command_path('B', endpoints)
    assert bad == 1

