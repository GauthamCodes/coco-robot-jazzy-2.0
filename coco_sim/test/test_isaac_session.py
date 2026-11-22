# Copyright 2026 Gautham Anil
# SPDX-License-Identifier: Apache-2.0
"""Ownership and configuration guards for the Isaac-only runtime harness."""
import copy
from pathlib import Path

from coco_sim.isaac.session import (
    classify_nav,
    nav_parameters,
    process_identity,
    same_process,
    session_members,
)


def fake_process(root, pid, start, session, state='S'):
    directory = root / str(pid)
    directory.mkdir(exist_ok=True)
    # proc stat fields 3..22; comm deliberately contains spaces and parentheses.
    fields = [state, '1', str(session), str(session)] + ['0'] * 15 + [str(start)]
    (directory / 'stat').write_text(f'{pid} (owned (test) process) ' + ' '.join(fields))


def test_pid_reuse_is_not_owned(tmp_path):
    fake_process(tmp_path, 50, 123, 50)
    record = process_identity(50, tmp_path)
    assert same_process(record, tmp_path)
    fake_process(tmp_path, 50, 124, 50)
    assert not same_process(record, tmp_path)


def test_members_include_orphan_children_but_not_other_sessions(tmp_path):
    fake_process(tmp_path, 51, 123, 50)
    fake_process(tmp_path, 52, 124, 999)
    fake_process(tmp_path, 53, 125, 50, state='Z')
    assert [x['pid'] for x in session_members(50, tmp_path)] == [51]
    assert process_identity(50, tmp_path) is None


def test_parameter_override_changes_only_map_and_ack_budget():
    import yaml

    source = Path(__file__).parents[2] / 'gazebo_models/config/nav2_params.yaml'
    base = yaml.safe_load(source.read_text())
    saved = copy.deepcopy(base)
    result = nav_parameters(base, '/tmp/fixture.yaml', 200)
    assert base == saved
    assert result['map_server']['ros__parameters'].pop('yaml_filename') == '/tmp/fixture.yaml'
    assert result['bt_navigator']['ros__parameters'].pop('default_server_timeout') == 200
    saved['map_server']['ros__parameters'].pop('yaml_filename')
    assert result == saved
    assert (
        'default_server_timeout'
        not in nav_parameters(base, '/tmp/map', None)['bt_navigator']['ros__parameters']
    )


def test_acknowledgement_failures_are_not_mislabelled_startup():
    assert (
        classify_nav('acknowledge goal request for compute_path_to_pose', 'unknown')
        == 'planner failure'
    )
    assert (
        classify_nav('acknowledge goal request for follow_path', 'unknown') == 'controller failure'
    )
    assert classify_nav('', 'TF readiness failure') == 'TF readiness failure'
