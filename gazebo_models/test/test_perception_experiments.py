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

"""C2-NAV.43: the depth-fusion experiment changes perception and nothing else.

The two arms, baseline_lidar_only.yaml and depth_fusion.yaml, must differ only
in the local costmap voxel layer's observation sources. These tests pin that,
and pin what the `perception` experiment key can never do: drop the 2D LiDAR,
reach the collision monitor, the global costmap, the obstacle layer or
inflation, or add a sensor the robot does not already carry.
"""
import copy
import importlib.util
import os
import re
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from nav_params_overlay import (apply_perception, check_perception_sources,  # noqa: E402
                                DEPTH_CLOUD_TOPIC, expected_value, ExperimentError,
                                LIDAR_SOURCE, load_experiment, load_perception,
                                PERCEPTION_LAYER, perception_launch_args,
                                perception_live_checks, resolve)

HERE = os.path.dirname(__file__)
PKG = os.path.join(HERE, '..')
BASE = os.path.join(PKG, 'config', 'nav2_params.yaml')
EXPERIMENTS = os.path.join(PKG, 'config', 'experiments')
BASELINE = os.path.join(EXPERIMENTS, 'baseline_lidar_only.yaml')
FUSION = os.path.join(EXPERIMENTS, 'depth_fusion.yaml')
LAYER = '.'.join(PERCEPTION_LAYER)


def _yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _diff(a, b, path=''):
    """Dotted paths at which two parameter documents differ."""
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for key in sorted(set(a) | set(b), key=str):
            here = f'{path}.{key}' if path else str(key)
            if key not in a or key not in b:
                out.append(here)
            else:
                out.extend(_diff(a[key], b[key], here))
        return out
    return [] if a == b else [path]


def test_the_two_arms_differ_only_in_perception():
    base, fusion = _yaml(BASELINE), _yaml(FUSION)
    for doc in (base, fusion):
        doc.pop('name')
        doc.pop('description')
    assert base.pop('perception') != fusion.pop('perception')
    assert base == fusion
    assert base['topology'] == 'B'


def test_baseline_resolves_to_the_shipped_file_byte_for_byte(tmp_path):
    resolved = resolve(BASELINE, BASE, str(tmp_path))
    assert resolved['changes'] == []
    assert resolved['params_file'] == os.path.abspath(BASE)
    assert resolved['params_sha256'] == resolved['base_params_sha256']
    assert resolved['perception'] == {'local_voxel_sources': ['scan'], 'sources': {}}


def test_fusion_changes_only_the_local_voxel_layer(tmp_path):
    resolved = resolve(FUSION, BASE, str(tmp_path))
    merged = _yaml(resolved['params_file'])
    shipped = _yaml(BASE)
    changed = _diff(shipped, merged)
    prefix = 'local_costmap.local_costmap.ros__parameters.voxel_layer.'
    assert changed and all(p.startswith(prefix) for p in changed), changed
    assert sorted(changed) == sorted([prefix + 'observation_sources', prefix + 'depth'])
    assert [c['path'] for c in resolved['changes']] == [
        f'{LAYER}.observation_sources', f'{LAYER}.depth']


def test_fusion_leaves_safety_global_obstacle_and_inflation_untouched(tmp_path):
    merged = _yaml(resolve(FUSION, BASE, str(tmp_path))['params_file'])
    shipped = _yaml(BASE)
    for top in ('collision_monitor', 'global_costmap', 'velocity_smoother', 'controller_server',
                'planner_server', 'bt_navigator', 'amcl', 'behavior_server'):
        assert merged[top] == shipped[top], top
    lm = merged['local_costmap']['local_costmap']['ros__parameters']
    ls = shipped['local_costmap']['local_costmap']['ros__parameters']
    for key in set(ls) | set(lm):
        if key != 'voxel_layer':
            assert lm.get(key) == ls.get(key), key
    assert merged['collision_monitor']['ros__parameters']['observation_sources'] == ['scan']


def test_fusion_keeps_the_lidar_and_adds_the_depth_source(tmp_path):
    merged = _yaml(resolve(FUSION, BASE, str(tmp_path))['params_file'])
    voxel = merged['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']
    assert voxel['observation_sources'].split() == ['scan', 'depth']
    shipped_voxel = _yaml(BASE)['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']
    assert voxel['scan'] == shipped_voxel['scan']
    depth = voxel['depth']
    assert depth['data_type'] == 'PointCloud2'
    # The depth source's reach is the LiDAR source's, so the two are compared
    # over the same volume rather than one seeing farther.
    for key in ('max_obstacle_height', 'obstacle_max_range', 'obstacle_min_range',
                'raytrace_max_range', 'raytrace_min_range', 'marking', 'clearing'):
        assert depth[key] == voxel['scan'][key], key
    for key in ('min_obstacle_height', 'max_obstacle_height', 'obstacle_max_range',
                'raytrace_max_range', 'obstacle_min_range', 'raytrace_min_range'):
        assert isinstance(depth[key], float), key


def test_shipped_local_voxel_layer_is_lidar_only():
    voxel = _yaml(BASE)['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']
    assert voxel['observation_sources'] == 'scan'
    assert 'depth' not in voxel


def _load_launch(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(PKG, 'launch', f'{name}.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_depth_source_is_the_depth_cloud_launch_output():
    topic = _yaml(FUSION)['perception']['sources']['depth']['topic']
    launch = _load_launch('depth_cloud.launch')
    assert topic == launch.DEPTH_CLOUD_TOPIC == DEPTH_CLOUD_TOPIC
    # NOT the bridged gz cloud, whose points are in the link convention.
    assert topic != '/camera/points'


def test_the_depth_cloud_is_computed_from_the_existing_bridged_camera():
    launch = _load_launch('depth_cloud.launch')
    bridge = {b['ros_topic_name']: b for b in _yaml(os.path.join(PKG, 'config', 'bridge.yaml'))}
    for topic, ros_type in ((launch.DEPTH_IMAGE_TOPIC, 'sensor_msgs/msg/Image'),
                            (launch.CAMERA_INFO_TOPIC, 'sensor_msgs/msg/CameraInfo')):
        assert bridge[topic]['ros_type_name'] == ros_type
        assert bridge[topic]['direction'] == 'GZ_TO_ROS'
    assert launch.DEPTH_CLOUD_TOPIC not in bridge
    nodes = [a for a in launch.generate_launch_description().entities
             if type(a).__name__ == 'Node']
    assert len(nodes) == 1
    node = nodes[0]
    assert node.node_package == 'depth_image_proc'
    assert node.node_executable == 'point_cloud_xyz_node'
    remaps = {}
    for src, dst in node._Node__remappings:
        remaps[''.join(getattr(s, 'text', str(s)) for s in src) if not isinstance(src, str) else src] = (
            ''.join(getattr(s, 'text', str(s)) for s in dst) if not isinstance(dst, str) else dst)
    assert remaps == {'image_rect': launch.DEPTH_IMAGE_TOPIC,
                      '/camera/depth/camera_info': launch.CAMERA_INFO_TOPIC,
                      'points': launch.DEPTH_CLOUD_TOPIC}


def test_the_depth_cloud_is_off_unless_asked_for(tmp_path):
    with open(os.path.join(PKG, 'launch', 'nav.launch.py')) as f:
        nav = f.read()
    assert re.search(r"'depth_cloud', default_value='false'", nav)
    assert "IfCondition(LaunchConfiguration('depth_cloud'))" in nav
    assert resolve(BASELINE, BASE, str(tmp_path / 'b'))['nav_launch_args'] == []
    assert resolve(FUSION, BASE, str(tmp_path / 'f'))['nav_launch_args'] == ['depth_cloud:=true']
    exp = load_experiment(os.path.join(EXPERIMENTS, 'baseline_topology_b.yaml'))
    assert perception_launch_args(exp['perception']) == []


def test_ros_clean_sweeps_what_depth_cloud_starts():
    with open(os.path.join(PKG, 'scripts', 'ros_clean.sh')) as f:
        text = f.read()
    assert "'depth_cloud[.]launch.py'" in text
    assert "'depth_image_proc/point_cloud_xyz_nod[e]'" in text
    assert "'c2nav43_perceptio[n]'" in text


def _endpoints(depth_pubs=('depth_cloud_xyz',), depth_subs=('local_costmap',)):
    return {DEPTH_CLOUD_TOPIC: (list(depth_pubs), list(depth_subs))}


def test_check_perception_sources_accepts_a_delivering_fusion(tmp_path):
    p = resolve(FUSION, BASE, str(tmp_path))['perception']
    bad, lines = check_perception_sources(p, _endpoints(),
                                          {DEPTH_CLOUD_TOPIC: 'camera_optical_frame'})
    assert bad == 0, lines


@pytest.mark.parametrize('endpoints,frames,match', [
    (_endpoints(depth_pubs=()), {DEPTH_CLOUD_TOPIC: None}, 'publisher'),
    (_endpoints(depth_subs=('c2nav43_record',)), {DEPTH_CLOUD_TOPIC: 'camera_optical_frame'},
     'read by local_costmap'),
    # matched but silent: the "we saw nothing" trap
    (_endpoints(), {DEPTH_CLOUD_TOPIC: None}, 'delivers a message'),
    # the gz cloud's mislabelled frame would not pass either
    (_endpoints(), {DEPTH_CLOUD_TOPIC: 'camera_link'}, 'delivers a message'),
])
def test_check_perception_sources_catches_a_blind_fusion(tmp_path, endpoints, frames, match):
    p = resolve(FUSION, BASE, str(tmp_path))['perception']
    bad, lines = check_perception_sources(p, endpoints, frames)
    assert bad >= 1
    assert any(match in ln for ln in lines if ln.startswith('MISMATCH')), lines


def test_check_perception_sources_baseline_runs_no_depth_cloud(tmp_path):
    p = resolve(BASELINE, BASE, str(tmp_path))['perception']
    assert check_perception_sources(p, {DEPTH_CLOUD_TOPIC: ([], [])}, {})[0] == 0
    bad, lines = check_perception_sources(p, _endpoints(depth_subs=()), {})
    assert bad == 1 and 'not running without a depth source' in lines[0]


def test_the_robot_still_carries_one_camera_and_one_lidar():
    with open(os.path.join(PKG, 'urdf', 'coco_robo2.xacro')) as f:
        xacro = f.read()
    types = re.findall(r'<sensor\s+name="[^"]+"\s+type="([^"]+)"', xacro)
    assert sorted(types) == ['gpu_lidar', 'imu', 'rgbd_camera']


def test_live_readback_proves_which_sources_loaded(tmp_path):
    shipped = _yaml(BASE)
    assert [c[2] for c in perception_live_checks(shipped)] == ['voxel_layer.observation_sources']
    merged = _yaml(resolve(FUSION, BASE, str(tmp_path))['params_file'])
    checks = perception_live_checks(merged)
    params = [c[2] for c in checks]
    assert params[0] == 'voxel_layer.observation_sources'
    depth_keys = sorted(merged['local_costmap']['local_costmap']['ros__parameters']
                        ['voxel_layer']['depth'])
    assert params[1:] == [f'voxel_layer.depth.{k}' for k in depth_keys]
    for node, yaml_path, param in checks:
        assert node == '/local_costmap/local_costmap'
        expected_value(merged, yaml_path, param)


GOOD_DEPTH = {'topic': '/camera/points', 'data_type': 'PointCloud2', 'min_obstacle_height': 0.05}


@pytest.mark.parametrize('block,match', [
    ({'local_voxel_sources': ['depth'], 'sources': {'depth': GOOD_DEPTH}}, 'never remove'),
    ({'local_voxel_sources': []}, 'non-empty'),
    ({'local_voxel_sources': ['scan', 'scan']}, 'distinct'),
    ({'local_voxel_sources': ['scan', 'depth']}, 'undefined'),
    ({'local_voxel_sources': ['scan'], 'sources': {'depth': GOOD_DEPTH}}, 'unlisted'),
    ({'local_voxel_sources': ['scan'], 'sources': {}, 'obstacle_layer': {}}, 'unknown key'),
    ({'local_voxel_sources': ['scan', 'scan2'],
      'sources': {'scan2': dict(GOOD_DEPTH, colour='red')}}, 'unknown key'),
    ({'local_voxel_sources': ['scan', 'depth'],
      'sources': {'depth': dict(GOOD_DEPTH, data_type='Image')}}, 'data_type'),
    ({'local_voxel_sources': ['scan', 'depth'],
      'sources': {'depth': dict(GOOD_DEPTH, topic='camera/points')}}, 'absolute'),
    ({'local_voxel_sources': ['scan', 'depth'],
      'sources': {'depth': {'topic': '/camera/points'}}}, 'required'),
    ({'local_voxel_sources': ['scan', 'depth'],
      'sources': {'depth': dict(GOOD_DEPTH, marking='yes')}}, 'bool'),
    ({'local_voxel_sources': ['scan'], 'sources': {'scan': GOOD_DEPTH}}, 'cannot be redefined'),
])
def test_bad_perception_is_refused(block, match):
    with pytest.raises(ExperimentError, match=match):
        load_perception(block)


def test_integer_heights_are_written_as_doubles():
    p = load_perception({'local_voxel_sources': ['scan', 'depth'],
                         'sources': {'depth': dict(GOOD_DEPTH, max_obstacle_height=2)}})
    assert p['sources']['depth']['max_obstacle_height'] == 2.0
    assert isinstance(p['sources']['depth']['max_obstacle_height'], float)


def test_perception_cannot_overwrite_an_existing_block():
    doc = _yaml(BASE)
    p = load_perception({'local_voxel_sources': ['scan', 'depth'], 'sources': {'depth': GOOD_DEPTH}})
    voxel = doc['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']
    voxel['depth'] = {'topic': '/elsewhere'}
    with pytest.raises(ExperimentError, match='already exists'):
        apply_perception(doc, p)


def test_perception_and_overrides_on_the_same_layer_are_refused(tmp_path):
    exp = _yaml(FUSION)
    exp['nav2_overrides'] = {'local_costmap': {'local_costmap': {'ros__parameters': {
        'voxel_layer': {'z_voxels': 10}}}}}
    path = tmp_path / 'both.yaml'
    path.write_text(yaml.safe_dump(exp))
    with pytest.raises(ExperimentError, match='both touch'):
        resolve(str(path), BASE, str(tmp_path / 'run'))


def test_experiments_without_perception_are_unchanged(tmp_path):
    exp = load_experiment(os.path.join(EXPERIMENTS, 'baseline_topology_b.yaml'))
    assert exp['perception'] is None
    assert apply_perception(copy.deepcopy(_yaml(BASE)), None) == []
    assert LIDAR_SOURCE == 'scan'
