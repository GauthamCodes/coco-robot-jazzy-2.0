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
The coco_episode CLI and the gz read-back parser.

The live evidence for stage C is produced with these, so they are tested
without a simulator: ``gz model -p`` output is fed in as text, and
``read_gz_poses`` takes an injected ``run``.
"""

import json
from types import SimpleNamespace

from coco_sim import episode_cli
from coco_sim.backends.gazebo import parse_gz_model_pose, read_gz_poses
from coco_sim.episode import (episode_from_json, generate_episode,
                              result_from_json)
import pytest

GZ_POSE = """Requesting state for world [coco_world]...

Model: [9]
  - Name: target_red
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [4.050000 -0.750000 0.728829]
    [0.000100 -0.000200 0.000000]
"""


def test_the_gz_pose_block_parses():
    pose = parse_gz_model_pose(GZ_POSE)
    assert (pose.x, pose.y, pose.z) == (4.05, -0.75, 0.728829)
    assert (pose.roll, pose.pitch) == (0.0001, -0.0002)


@pytest.mark.parametrize('text', ['', 'Model not found', 'Pose\n[1 2]\n[0 0 0]',
                                  'Pose\n[a b c]\n[0 0 0]'])
def test_an_unreadable_reply_is_none_not_zero(text):
    assert parse_gz_model_pose(text) is None


def test_read_gz_poses_skips_what_gz_cannot_report():
    def fake(cmd, **_):
        name = cmd[3]
        if name == 'target_red':
            return SimpleNamespace(returncode=0, stdout=GZ_POSE)
        return SimpleNamespace(returncode=255, stdout='')
    poses = read_gz_poses(['target_red', 'target_blue'], run=fake)
    assert set(poses) == {'target_red'}


def test_generate_writes_the_manifest_the_launch_files_replay(tmp_path):
    out = tmp_path / 'm.json'
    assert episode_cli.main(['generate', '--level', 'colours', '--seed', '1',
                             '--colour', 'yellow', '--out', str(out)]) == 0
    spec = episode_from_json(out.read_text())
    assert spec == generate_episode(seed=1, level='colours',
                                    requested_colour='yellow')


def test_inputs_prints_names_only(tmp_path, capsys):
    out = tmp_path / 'm.json'
    episode_cli.main(['generate', '--level', 'positions', '--seed', '4',
                      '--out', str(out)])
    episode_cli.main(['inputs', '--manifest', str(out)])
    printed = json.loads(capsys.readouterr().out)
    assert set(printed) == {'target_colour', 'region_map'}
    spec = episode_from_json(out.read_text())
    for t in spec.targets:
        assert repr(t.x) not in json.dumps(printed)


def test_check_says_reproducible_then_drifted(tmp_path, capsys):
    out = tmp_path / 'm.json'
    episode_cli.main(['generate', '--level', 'positions', '--seed', '2',
                      '--out', str(out)])
    assert episode_cli.main(['check', '--manifest', str(out)]) == 0
    data = json.loads(out.read_text())
    data['targets'][0]['x'] += 0.001        # a legal pose the seed never made
    out.write_text(json.dumps(data))
    assert episode_cli.main(['check', '--manifest', str(out)]) == 1
    assert 'DRIFTED' in capsys.readouterr().out


def test_result_records_a_validated_run(tmp_path):
    manifest = tmp_path / 'm.json'
    episode_cli.main(['generate', '--level', 'colours', '--seed', '2',
                      '--colour', 'red', '--out', str(manifest)])
    out = tmp_path / 'r.json'
    assert episode_cli.main([
        'result', '--manifest', str(manifest), '--outcome', 'failed',
        '--reason', 'RETURN_FAILED', '--timing', 'mission_sim_s=12.5',
        '--commit', 'abc', '--measurements', '{"bypass": 0}',
        '--out', str(out)]) == 0
    result = result_from_json(out.read_text())
    assert result.outcome == 'failed'
    assert result.episode().requested_colour == 'red'
    assert result.measurements == {'bypass': 0}


def test_an_invalid_result_is_refused_with_exit_2(tmp_path):
    manifest = tmp_path / 'm.json'
    episode_cli.main(['generate', '--out', str(manifest)])
    assert episode_cli.main([
        'result', '--manifest', str(manifest), '--outcome', 'failed',
        '--out', str(tmp_path / 'r.json')]) == 2       # no reason given
