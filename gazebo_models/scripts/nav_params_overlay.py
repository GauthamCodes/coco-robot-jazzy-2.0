#!/usr/bin/env python3
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
Resolve a navigation experiment file into the inputs of one tour run.

An experiment file names ONE controlled variant: the nav_bench arguments,
whether the instrumented AMCL is swapped in, and which nav2 parameters
differ from the shipped config/nav2_params.yaml. Only the differences are
written down, so an experiment's variable is visible at a glance instead of
buried in a full copy of the parameter file (the docs/data/c2navN_*_params
pattern every C2-NAV experiment used until C2-NAV.39).

Guards, each of which an earlier experiment would have tripped:
  * an override key must already exist in the base file, so a typo is an
    error rather than a parameter nav2 silently ignores;
  * a scalar cannot replace a block, a block cannot replace a scalar, and a
    value cannot change type (int and float are interchangeable);
  * anything under collision_monitor is a safety gate and is refused unless
    the experiment sets allow_safety_change: true explicitly.

  nav_params_overlay.py resolve EXPERIMENT.yaml --base BASE.yaml --out-dir RUN_DIR
  nav_params_overlay.py verify-live --params PARAMS.yaml --out RUN_DIR/params_live.txt
  nav_params_overlay.py verify-goals --resolved RUN_DIR/experiment_resolved.json \
      --bench RUN_DIR/<run>.json --out RUN_DIR/goals_check.txt
  nav_params_overlay.py verify-topology --resolved RUN_DIR/experiment_resolved.json \
      --out RUN_DIR/topology_live.txt

`resolve` writes RUN_DIR/experiment_resolved.json and, only when something
changes, RUN_DIR/params_merged.yaml (neither name contains ros_clean.sh's
'nav2_'). `verify-live` reads the accepted and safety-critical parameters
back off the RUNNING nodes and compares them with the file: a file that was
edited and a parameter that was loaded are different claims.

C2-NAV.40: `bench.goals` moves a scenario goal through nav_bench.py's
existing `--goal NAME:X,Y` override. It is a benchmark change, not a nav2
parameter, so it never produces a merged parameter file. Scenario names are
checked against nav_bench.py's TOUR before anything launches, and
`verify-goals` compares the goal every leg RECORDED (`goal_world`) with the
goal the experiment asked for -- an override that was requested and one that
was driven are, again, different claims.

C2-NAV.41: `topology` selects WHICH PROCESSES OWN THE WHEELS -- A for
nav.launch.py alone, B for the cmd_vel_arbiter path mission.launch.py runs.
It is neither a nav2 parameter nor a benchmark goal, so it too produces no
merged file. `verify-topology` reads the wheel topic's publishers off the
LIVE graph: an arbiter that failed to start leaves cmd_vel_relay driving and
the tour is then topology A wearing topology B's label, which is the third
instance of the same "requested" / "actually loaded" distinction.
"""
import argparse
import ast
import copy
import hashlib
import json
import math
import os
import re
import subprocess
import sys

import yaml

TOP_KEYS = {'name', 'description', 'bench', 'amcl_diag', 'nav2_overrides',
            'allow_safety_change', 'topology', 'perception'}

# C2-NAV.43. `perception` adds observation sources to the LOCAL costmap's
# voxel layer, and nothing else. It is the one experiment key allowed to
# create parameters that do not exist in the base file, so it is narrow on
# purpose: it cannot reach the obstacle layer, the global costmap, inflation
# or the collision monitor, and it cannot drop the 2D LiDAR ('scan'). The
# sources are ordinary nav2_costmap_2d observation sources.
PERCEPTION_KEYS = {'local_voxel_sources', 'sources'}
PERCEPTION_LAYER = ('local_costmap', 'local_costmap', 'ros__parameters', 'voxel_layer')
PERCEPTION_NODE = '/local_costmap/local_costmap'
LIDAR_SOURCE = 'scan'
SOURCE_DATA_TYPES = ('PointCloud2', 'LaserScan')
# nav2_costmap_2d ObstacleLayer::onInitialize's per-source parameters (Jazzy).
SOURCE_KEYS = {
    'topic': str, 'data_type': str, 'sensor_frame': str,
    'observation_persistence': float, 'expected_update_rate': float,
    'min_obstacle_height': float, 'max_obstacle_height': float,
    'inf_is_valid': bool, 'marking': bool, 'clearing': bool,
    'obstacle_max_range': float, 'obstacle_min_range': float,
    'raytrace_max_range': float, 'raytrace_min_range': float,
}
BENCH_KEYS = {'repeats', 'timeout', 'only', 'goals'}
NAV_BENCH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'nav_bench.py')
AMCL_DIAG_KEYS = {'enabled'}

# C2-NAV.41. WHICH COMMAND PATH the tour drives. This is not a tuning knob
# and not a nav2 parameter -- it is which processes own the wheels.
#
#   A  nav.launch.py alone. cmd_vel_relay publishes the controller topic
#      directly. Every C2-NAV.0 ... C2-NAV.40 tour ran this.
#   B  what mission.launch.py runs: nav.launch.py arbiter:=true, so the
#      relay's output goes to /cmd_vel_gated (C2-NAV.42; /cmd_vel_nav before
#      it), and cmd_vel_arbiter is the sole publisher of the controller topic.
#      This is the path the robot SHIPS in.
TOPOLOGIES = ('A', 'B')
WHEEL_TOPIC = '/diff_drive_controller/cmd_vel'
ARBITER_STATUS_TOPIC = '/cmd_vel_arbiter/status'
TOPOLOGY_PUBLISHER = {'A': 'cmd_vel_relay', 'B': 'cmd_vel_arbiter'}

# C2-NAV.42. The command chain between Nav2 and the wheels, checked link by
# link on the live graph. Before C2-NAV.42 topology B's relay published
# RAW_NAV_TOPIC and the arbiter read it, so the raw controller command
# reached the wheels past the smoother and the collision monitor.
RAW_NAV_TOPIC = '/cmd_vel_nav'
SMOOTHED_TOPIC = '/cmd_vel_smoothed'
MONITOR_OUT_TOPIC = '/cmd_vel'
GATED_TOPIC = '/cmd_vel_gated'
CHAIN_TOPICS = (RAW_NAV_TOPIC, SMOOTHED_TOPIC, MONITOR_OUT_TOPIC, GATED_TOPIC)
# nav2_bringup does not remap opennav_docking's cmd_vel, so docking_server is
# a publisher on the relay's input in BOTH topologies (C2-M5.0's
# c2m5_topology.txt recorded it). It publishes only while executing a dock or
# undock action, and nothing in this project sends one. Named here so it is
# listed, not silently tolerated; any OTHER extra publisher is a mismatch.
INERT_MONITOR_PEERS = ('docking_server',)
SAFETY_NODES = ('collision_monitor',)
MERGED_NAME = 'params_merged.yaml'
RESOLVED_NAME = 'experiment_resolved.json'
_NAME = re.compile(r'^[a-z0-9][a-z0-9_]*$')
_PARAM_GET = re.compile(r'^(Boolean|Integer|Double|String) value is: ?(.*)$')

# (live node, top-level yaml path, parameter) -- the accepted C2-NAV values
# and the safety gates every tour run must prove were actually loaded.
LIVE_CHECKS = (
    ('/local_costmap/local_costmap', 'local_costmap.local_costmap',
     'inflation_layer.cost_scaling_factor'),
    ('/local_costmap/local_costmap', 'local_costmap.local_costmap',
     'inflation_layer.inflation_radius'),
    ('/global_costmap/global_costmap', 'global_costmap.global_costmap',
     'inflation_layer.cost_scaling_factor'),
    ('/bt_navigator', 'bt_navigator', 'default_nav_through_poses_bt_xml'),
    ('/controller_server', 'controller_server', 'FollowPath.BaseObstacle.scale'),
    ('/controller_server', 'controller_server', 'FollowPath.xy_goal_tolerance'),
    ('/controller_server', 'controller_server', 'goal_checker.xy_goal_tolerance'),
    ('/controller_server', 'controller_server', 'goal_checker.yaw_goal_tolerance'),
    ('/collision_monitor', 'collision_monitor', 'PolygonStop.radius'),
    ('/collision_monitor', 'collision_monitor', 'PolygonStop.min_points'),
    ('/collision_monitor', 'collision_monitor', 'PolygonSlow.slowdown_ratio'),
    ('/amcl', 'amcl', 'resample_interval'),
)


class ExperimentError(ValueError):
    pass


def sha256_file(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def _same_kind(old, new):
    if isinstance(old, bool) or isinstance(new, bool):
        return isinstance(old, bool) and isinstance(new, bool)
    if isinstance(old, (int, float)) and isinstance(new, (int, float)):
        return True
    return type(old) is type(new)


def merge_overrides(base, overrides, allow_safety_change=False):
    """Return (merged copy of base, [(dotted path, old, new), ...])."""
    merged = copy.deepcopy(base)
    changes = []

    def walk(dst, src, path):
        for key, value in src.items():
            here = path + [str(key)]
            dotted = '.'.join(here)
            if here[0] in SAFETY_NODES and not allow_safety_change:
                raise ExperimentError(
                    f'override {dotted} touches a safety gate; set '
                    'allow_safety_change: true to do that deliberately')
            if not isinstance(dst, dict) or key not in dst:
                raise ExperimentError(f'override {dotted} does not exist in the base file')
            old = dst[key]
            if isinstance(value, dict):
                if not isinstance(old, dict):
                    raise ExperimentError(f'override {dotted} replaces a scalar with a block')
                walk(old, value, here)
                continue
            if isinstance(old, dict):
                raise ExperimentError(f'override {dotted} replaces a block with a scalar')
            if not _same_kind(old, value):
                raise ExperimentError(
                    f'override {dotted} changes type {type(old).__name__} -> '
                    f'{type(value).__name__}')
            if old != value:
                changes.append((dotted, old, value))
            dst[key] = value

    if not isinstance(overrides, dict):
        raise ExperimentError('nav2_overrides must be a mapping')
    walk(merged, overrides, [])
    return merged, changes


def _only_keys(block, allowed, where):
    if not isinstance(block, dict):
        raise ExperimentError(f'{where} must be a mapping')
    unknown = sorted(set(block) - allowed)
    if unknown:
        raise ExperimentError(f'unknown key(s) in {where}: {unknown}')


def load_experiment(path):
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    _only_keys(doc, TOP_KEYS, 'experiment')
    name = doc.get('name')
    if not isinstance(name, str) or not _NAME.match(name) or 'nav2_' in name:
        raise ExperimentError(
            f'name {name!r} must be lower-case [a-z0-9_] and must not contain '
            "'nav2_' (ros_clean.sh would kill any process carrying it)")
    bench = doc.get('bench') or {}
    _only_keys(bench, BENCH_KEYS, 'bench')
    repeats = bench.get('repeats', 1)
    timeout = bench.get('timeout', 75)
    only = bench.get('only')
    if not isinstance(repeats, int) or isinstance(repeats, bool) or repeats < 1:
        raise ExperimentError('bench.repeats must be an integer >= 1')
    if not isinstance(timeout, (int, float)) or isinstance(timeout, bool) or timeout <= 0:
        raise ExperimentError('bench.timeout must be a positive number of seconds')
    if only is not None and not isinstance(only, str):
        raise ExperimentError('bench.only must be a comma-separated string of scenario names')
    goals = bench.get('goals') or {}
    if not isinstance(goals, dict):
        raise ExperimentError('bench.goals must be a mapping of scenario -> [x, y]')
    for scenario, xy in goals.items():
        if (not isinstance(xy, list) or len(xy) != 2
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in xy)):
            raise ExperimentError(
                f'bench.goals.{scenario} must be [x, y] in world metres, got {xy!r}')
    goals = {str(s): [float(xy[0]), float(xy[1])] for s, xy in goals.items()}
    diag = doc.get('amcl_diag') or {}
    _only_keys(diag, AMCL_DIAG_KEYS, 'amcl_diag')
    enabled = diag.get('enabled', False)
    if not isinstance(enabled, bool):
        raise ExperimentError('amcl_diag.enabled must be true or false')
    allow = doc.get('allow_safety_change', False)
    if not isinstance(allow, bool):
        raise ExperimentError('allow_safety_change must be true or false')
    topology = doc.get('topology', 'A')
    if not isinstance(topology, str) or topology.strip().upper() not in TOPOLOGIES:
        raise ExperimentError(
            f'topology must be one of {list(TOPOLOGIES)} '
            f'(A = nav.launch.py alone, B = through cmd_vel_arbiter), got {topology!r}')
    return {
        'name': name,
        'description': doc.get('description', ''),
        'topology': topology.strip().upper(),
        'bench': {'repeats': repeats, 'timeout': timeout, 'only': only, 'goals': goals},
        'amcl_diag': {'enabled': enabled},
        'nav2_overrides': doc.get('nav2_overrides') or {},
        'allow_safety_change': allow,
        'perception': load_perception(doc.get('perception')),
    }


def load_perception(block):
    """Validate an experiment's `perception` block; None when absent."""
    if block is None:
        return None
    _only_keys(block, PERCEPTION_KEYS, 'perception')
    listed = block.get('local_voxel_sources')
    if (not isinstance(listed, list) or not listed
            or not all(isinstance(s, str) and _NAME.match(s) for s in listed)
            or len(set(listed)) != len(listed)):
        raise ExperimentError(
            'perception.local_voxel_sources must be a non-empty list of distinct '
            'lower-case source names')
    if LIDAR_SOURCE not in listed:
        raise ExperimentError(
            f"perception.local_voxel_sources must keep '{LIDAR_SOURCE}': an experiment "
            'may add to the 2D LiDAR, never remove it')
    sources = block.get('sources') or {}
    if not isinstance(sources, dict):
        raise ExperimentError('perception.sources must be a mapping of name -> source')
    for name, src in sources.items():
        where = f'perception.sources.{name}'
        if not isinstance(name, str) or not _NAME.match(name) or 'nav2_' in name:
            raise ExperimentError(f'{where}: bad source name')
        if name == LIDAR_SOURCE:
            raise ExperimentError(f"{where}: '{LIDAR_SOURCE}' is the shipped LiDAR source "
                                  'and cannot be redefined')
        _only_keys(src, set(SOURCE_KEYS), where)
        for key in ('topic', 'data_type'):
            if key not in src:
                raise ExperimentError(f'{where}.{key} is required')
        for key, value in src.items():
            want = SOURCE_KEYS[key]
            ok = (isinstance(value, bool) if want is bool else
                  isinstance(value, str) if want is str else
                  isinstance(value, (int, float)) and not isinstance(value, bool)
                  and math.isfinite(value))
            if not ok:
                raise ExperimentError(f'{where}.{key} must be a {want.__name__}, got {value!r}')
        if src['data_type'] not in SOURCE_DATA_TYPES:
            raise ExperimentError(f'{where}.data_type must be one of {list(SOURCE_DATA_TYPES)}')
        if not src['topic'].startswith('/'):
            raise ExperimentError(f'{where}.topic must be absolute')
    undefined = [s for s in listed if s != LIDAR_SOURCE and s not in sources]
    unlisted = [s for s in sources if s not in listed]
    if undefined:
        raise ExperimentError(f'perception lists undefined source(s) {undefined}')
    if unlisted:
        raise ExperimentError(f'perception defines unlisted source(s) {unlisted}')
    # nav2 declares the numeric source parameters as doubles; a YAML integer
    # would be rejected at load time, so they are written as floats.
    return {'local_voxel_sources': list(listed),
            'sources': {n: {k: (float(v) if SOURCE_KEYS[k] is float else v) for k, v in s.items()}
                        for n, s in sources.items()}}


def apply_perception(doc, perception):
    """Apply a validated perception block to a parameter document in place.

    Returns [(dotted path, old, new), ...] in merge_overrides' format.
    """
    if not perception:
        return []
    layer = doc
    for key in PERCEPTION_LAYER:
        if not isinstance(layer, dict) or key not in layer:
            raise ExperimentError(f"base file has no {'.'.join(PERCEPTION_LAYER)}")
        layer = layer[key]
    dotted = '.'.join(k for k in PERCEPTION_LAYER if k != 'ros__parameters')
    changes = []
    old = layer.get('observation_sources')
    if not isinstance(old, str) or LIDAR_SOURCE not in old.split():
        raise ExperimentError(f"base {dotted}.observation_sources does not carry '{LIDAR_SOURCE}'")
    new = ' '.join(perception['local_voxel_sources'])
    if old != new:
        changes.append((f'{dotted}.observation_sources', old, new))
        layer['observation_sources'] = new
    for name, src in perception['sources'].items():
        if name in layer:
            raise ExperimentError(f'perception source {name} already exists in {dotted}')
        layer[name] = dict(src)
        changes.append((f'{dotted}.{name}', None, dict(src)))
    return changes


def perception_live_checks(doc):
    """(node, yaml path, parameter) readbacks for the local voxel layer's sources.

    Always the source list, so a baseline proves it loaded LiDAR only; plus
    every value of every listed non-LiDAR source."""
    try:
        layer = doc
        for key in PERCEPTION_LAYER:
            layer = layer[key]
    except (KeyError, TypeError):
        return []
    yaml_path = '.'.join(PERCEPTION_LAYER[:2])
    checks = [(PERCEPTION_NODE, yaml_path, 'voxel_layer.observation_sources')]
    for name in str(layer.get('observation_sources', '')).split():
        if name == LIDAR_SOURCE or not isinstance(layer.get(name), dict):
            continue
        for key in sorted(layer[name]):
            checks.append((PERCEPTION_NODE, yaml_path, f'voxel_layer.{name}.{key}'))
    return checks


def tour_goals(nav_bench_path=NAV_BENCH):
    """{scenario: [x, y]} of nav_bench.py's committed TOUR, read statically --
    nav_bench imports rclpy, and resolving an experiment must not need ROS."""
    with open(nav_bench_path) as f:
        tree = ast.parse(f.read())
    for node in tree.body:
        if (isinstance(node, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == 'TOUR' for t in node.targets)):
            return {n: [float(x), float(y)] for n, x, y, _ in ast.literal_eval(node.value)}
    raise ExperimentError(f'no TOUR assignment in {nav_bench_path}')


def apply_goals(goals, committed):
    """(effective {scenario: [x, y]}, ['NAME:X,Y' for nav_bench --goal])."""
    unknown = sorted(set(goals) - set(committed))
    if unknown:
        raise ExperimentError(f'bench.goals names scenario(s) not in TOUR: {unknown}')
    effective = {n: list(xy) for n, xy in committed.items()}
    args = []
    for scenario, (x, y) in goals.items():
        effective[scenario] = [x, y]
        args.append(f'{scenario}:{x!r},{y!r}')
    return effective, args


def resolve(experiment_path, base_path, out_dir, nav_bench_path=NAV_BENCH):
    exp = load_experiment(experiment_path)
    committed = tour_goals(nav_bench_path)
    effective_goals, goal_args = apply_goals(exp['bench']['goals'], committed)
    with open(base_path) as f:
        base = yaml.safe_load(f)
    merged, changes = merge_overrides(base, exp['nav2_overrides'], exp['allow_safety_change'])
    if exp['perception'] and any(p.startswith('.'.join(k for k in PERCEPTION_LAYER
                                                       if k != 'ros__parameters'))
                                 for p, _, _ in changes):
        raise ExperimentError('perception and nav2_overrides both touch the local voxel layer')
    changes += apply_perception(merged, exp['perception'])
    os.makedirs(out_dir, exist_ok=True)
    if changes:
        params_path = os.path.join(os.path.abspath(out_dir), MERGED_NAME)
        with open(params_path, 'w') as f:
            yaml.safe_dump(merged, f, sort_keys=False)
    else:
        params_path = os.path.abspath(base_path)
    resolved = dict(
        exp,
        experiment_file=os.path.abspath(experiment_path),
        experiment_sha256=sha256_file(experiment_path),
        base_params_file=os.path.abspath(base_path),
        base_params_sha256=sha256_file(base_path),
        params_file=params_path,
        params_sha256=sha256_file(params_path),
        changes=[{'path': p, 'old': o, 'new': n} for p, o, n in changes],
        goal_changes=[{'scenario': s, 'old': committed[s], 'new': effective_goals[s]}
                      for s in exp['bench']['goals'] if committed[s] != effective_goals[s]],
        goal_args=goal_args,
        tour_goals=effective_goals,
    )
    with open(os.path.join(out_dir, RESOLVED_NAME), 'w') as f:
        json.dump(resolved, f, indent=1)
    return resolved


def lookup_param(block, dotted):
    """Resolve a ROS parameter name against a ros__parameters block whose keys
    may themselves contain dots ('FollowPath' -> 'BaseObstacle.scale')."""
    parts = dotted.split('.')
    cur, i = block, 0
    while i < len(parts):
        for j in range(len(parts), i, -1):
            key = '.'.join(parts[i:j])
            if isinstance(cur, dict) and key in cur:
                cur, i = cur[key], j
                break
        else:
            raise KeyError(dotted)
    return cur


def expected_value(doc, yaml_path, param):
    node = doc
    for part in yaml_path.split('.'):
        node = node[part]
    return lookup_param(node['ros__parameters'], param)


def parse_param_get(text):
    lines = [ln.strip() for ln in text.strip().splitlines() if ln.strip()]
    m = _PARAM_GET.match(lines[-1]) if lines else None
    if not m:
        return None
    kind, raw = m.groups()
    if kind == 'Boolean':
        return raw == 'True'
    if kind == 'Integer':
        return int(raw)
    if kind == 'Double':
        return float(raw)
    return raw


def values_match(expected, live):
    if live is None:
        return False
    if isinstance(expected, str) and '$(' in expected:
        # $(find-pkg-share X) is substituted at load time; the file-relative
        # tail is what the file actually chose.
        return isinstance(live, str) and live.endswith(expected.rsplit(')', 1)[-1])
    if isinstance(expected, bool) or isinstance(live, bool):
        return expected is live
    if isinstance(expected, (int, float)) and isinstance(live, (int, float)):
        return abs(expected - live) <= 1e-9
    return expected == live


def verify_live(params_path, out_path, timeout=20.0):
    with open(params_path) as f:
        doc = yaml.safe_load(f)
    mismatches = 0
    lines = [f'# live parameter readback against {params_path} '
             f'(sha256 {sha256_file(params_path)})']
    for node, yaml_path, param in tuple(LIVE_CHECKS) + tuple(perception_live_checks(doc)):
        want = expected_value(doc, yaml_path, param)
        try:
            res = subprocess.run(['ros2', 'param', 'get', node, param],
                                 capture_output=True, text=True, timeout=timeout)
            got = parse_param_get(res.stdout)
            raw = (res.stdout + res.stderr).strip().splitlines()[-1:] or ['']
        except subprocess.TimeoutExpired:
            got, raw = None, ['timeout']
        ok = values_match(want, got)
        mismatches += 0 if ok else 1
        lines.append(f'{"OK      " if ok else "MISMATCH"} {node} {param} '
                     f'file={want!r} live={got!r}' + ('' if ok else f' ({raw[0]})'))
    with open(out_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return mismatches, lines


def parse_topic_info(text):
    """Publisher node names from `ros2 topic info -v`, in the order printed.

    The `Endpoint type:` line is what decides, not the `Publisher count:`
    header: the same `Node name:` key introduces publisher AND subscription
    blocks, so anything that keys off the header alone counts subscribers as
    publishers the moment the output order changes.
    """
    names, name = [], None
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith('Node name:'):
            name = line.split(':', 1)[1].strip()
        elif line.startswith('Endpoint type:'):
            if line.split(':', 1)[1].strip() == 'PUBLISHER' and name:
                names.append(name)
            name = None
    return names


def parse_topic_endpoints(text):
    """(publishers, subscribers): node names from `ros2 topic info -v`.

    Deduplicated and sorted: behavior_server holds one publisher per behavior
    plugin, all on /cmd_vel_nav, and the question here is WHICH nodes.
    """
    kinds = {'PUBLISHER': set(), 'SUBSCRIPTION': set()}
    name = None
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith('Node name:'):
            name = line.split(':', 1)[1].strip()
        elif line.startswith('Endpoint type:'):
            kind = line.split(':', 1)[1].strip()
            if name and kind in kinds:
                kinds[kind].add(name)
            name = None
    return sorted(kinds['PUBLISHER']), sorted(kinds['SUBSCRIPTION'])


def check_command_path(topology, endpoints):
    """(mismatches, lines) -- every link from the controller to the relay's
    output, against the topology.

    `endpoints` maps each of CHAIN_TOPICS to (publishers, subscribers).

    Positive controls come first: controller_server must be seen publishing
    /cmd_vel_nav and velocity_smoother reading it, so a graph this function
    cannot read fails instead of passing the "nothing of ours is on the raw
    topic" checks that follow.
    """
    lines, bad = [], 0

    def check(ok, label, detail):
        nonlocal bad
        bad += 0 if ok else 1
        lines.append(f'{"OK      " if ok else "MISMATCH"} {label}: {detail}')

    def get(topic):
        pubs, subs = endpoints.get(topic, ([], []))
        return list(pubs), list(subs)

    pubs, subs = get(RAW_NAV_TOPIC)
    check('controller_server' in pubs, f'{RAW_NAV_TOPIC} controller publishes',
          f'want controller_server among {pubs}')
    check('velocity_smoother' in subs, f'{RAW_NAV_TOPIC} smoother reads',
          f'want velocity_smoother among {subs}')
    check('cmd_vel_relay' not in pubs, f'{RAW_NAV_TOPIC} relay does not publish',
          f'want no cmd_vel_relay among {pubs}')
    check('cmd_vel_arbiter' not in subs, f'{RAW_NAV_TOPIC} arbiter does not read',
          f'want no cmd_vel_arbiter among {subs}')

    pubs, subs = get(SMOOTHED_TOPIC)
    check(pubs == ['velocity_smoother'], f'{SMOOTHED_TOPIC} publisher',
          f"want ['velocity_smoother'], got {pubs}")
    check('collision_monitor' in subs, f'{SMOOTHED_TOPIC} monitor reads',
          f'want collision_monitor among {subs}')

    pubs, subs = get(MONITOR_OUT_TOPIC)
    extra = [p for p in pubs if p not in ('collision_monitor',) + INERT_MONITOR_PEERS]
    check('collision_monitor' in pubs and not extra, f'{MONITOR_OUT_TOPIC} publishers',
          f'want collision_monitor (+ inert {list(INERT_MONITOR_PEERS)}), got {pubs}')
    check('cmd_vel_relay' in subs, f'{MONITOR_OUT_TOPIC} relay reads',
          f'want cmd_vel_relay among {subs}')

    pubs, subs = get(GATED_TOPIC)
    if topology == 'B':
        check(pubs == ['cmd_vel_relay'], f'{GATED_TOPIC} publisher',
              f"want ['cmd_vel_relay'], got {pubs}")
        check('cmd_vel_arbiter' in subs, f'{GATED_TOPIC} arbiter reads',
              f'want cmd_vel_arbiter among {subs}')
    else:
        check(pubs == [], f'{GATED_TOPIC} unused in topology A',
              f'want no publisher, got {pubs}')
    return bad, lines


def parse_arbiter_status(text):
    """{key: value} from one /cmd_vel_arbiter/status line.

    cmd_vel_arbiter.format_status writes space-separated key=value; `ros2
    topic echo` wraps it in `data:` and may quote it. {} means no status was
    seen at all, which is how topology A proves no arbiter is running.
    """
    payload = None
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith('data:'):
            payload = line.split(':', 1)[1].strip()
            break
        if '=' in line and not line.startswith('-'):
            payload = line
            break
    if not payload:
        return {}
    out = {}
    for token in payload.strip().strip('"\'').split():
        key, sep, value = token.partition('=')
        if sep:
            out[key] = value
    return out


def check_topology(topology, publishers, status):
    """(mismatches, lines) -- that the command path the experiment asked for
    is the one actually wired up.

    Two things are asserted whatever the topology, because both have cost a
    run before:

    * the wheel topic has EXACTLY ONE publisher. Two and the robot tracks
      their average instead of obeying either (CLAUDE.md rule 5).
    * that publisher is the one this topology names. A silently absent
      arbiter leaves the relay driving the wheels and the tour is then
      topology A wearing topology B's label.

    Topology B also requires `mode=nav`. Nothing publishes /mission/mode in a
    tour, so an arbiter left in its safe `idle` default forwards nothing and
    the robot does not move at all -- C2-NAV.21 measured exactly 0.000 m.

    Topology A's arbiter check succeeds on seeing NOTHING, which is the
    failure mode CLAUDE.md warns about. Its positive control is in the same
    result: the publisher list is read from the live graph and must be
    non-empty and equal to ['cmd_vel_relay'], so a graph this function cannot
    read fails the first two checks rather than passing the third.
    """
    want = TOPOLOGY_PUBLISHER[topology]
    lines, bad = [], 0

    def check(ok, label, detail):
        nonlocal bad
        bad += 0 if ok else 1
        lines.append(f'{"OK      " if ok else "MISMATCH"} {label}: {detail}')

    check(len(publishers) == 1, f'{WHEEL_TOPIC} publisher count',
          f'want exactly 1, got {len(publishers)} {publishers}')
    check(publishers == [want], f'{WHEEL_TOPIC} owner',
          f'want [{want!r}] for topology {topology}, got {publishers}')
    if topology == 'B':
        check(status.get('mode') == 'nav', 'arbiter mode',
              f'want nav, got {status.get("mode")!r} '
              f'(status: {status or "no /cmd_vel_arbiter/status seen"})')
    else:
        check(not status, 'no arbiter running',
              f'want no {ARBITER_STATUS_TOPIC}, got {status or "none"}')
    return bad, lines


def verify_topology(topology, out_path, timeout=20.0):
    """Read the live command path back and check it against the experiment."""
    def run(cmd, secs):
        try:
            return subprocess.run(cmd, capture_output=True, text=True,
                                  timeout=secs).stdout
        except subprocess.TimeoutExpired as e:
            return e.stdout.decode() if isinstance(e.stdout, bytes) else (e.stdout or '')

    publishers = parse_topic_info(
        run(['ros2', 'topic', 'info', '-v', WHEEL_TOPIC], timeout))
    # --once blocks until a message arrives; in topology A none ever does, so
    # this must wait the timeout out to prove absence rather than assume it.
    status = parse_arbiter_status(
        run(['ros2', 'topic', 'echo', '--once', ARBITER_STATUS_TOPIC], timeout))
    mismatches, lines = check_topology(topology, publishers, status)
    endpoints = {topic: parse_topic_endpoints(
        run(['ros2', 'topic', 'info', '-v', topic], timeout)) for topic in CHAIN_TOPICS}
    chain_bad, chain_lines = check_command_path(topology, endpoints)
    mismatches += chain_bad
    lines = ([f'# live command path against topology {topology}'] + lines
             + ['# command chain, link by link (C2-NAV.42)'] + chain_lines
             + ['# endpoints read'] + [f'#   {t} pub={p} sub={s}'
                                        for t, (p, s) in endpoints.items()])
    with open(out_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return mismatches, lines


def verify_goals(resolved, bench_doc, tol=1e-9):
    """(mismatches, lines): every leg's recorded goal_world against the goal
    the resolved experiment asked for. A leg with no goal_world never started
    and cannot be checked; an overridden scenario that no leg checked is a
    mismatch, because then nothing shows the override was driven."""
    want = resolved['tour_goals']
    moved = {c['scenario'] for c in resolved.get('goal_changes', [])}
    mismatches, checked = 0, set()
    lines = []
    for leg in bench_doc['legs']:
        name, rep = leg['scenario'], leg.get('rep', 0)
        got = leg.get('goal_world')
        if got is None:
            lines.append(f'UNCHECKED {name} rep{rep}: no goal_world (leg never started)')
            continue
        exp = want.get(name)
        ok = exp is not None and all(abs(a - b) <= tol for a, b in zip(exp, got))
        if ok:
            checked.add(name)
        else:
            mismatches += 1
        lines.append(f'{"OK      " if ok else "MISMATCH"} {name} rep{rep} '
                     f'requested={exp} driven={got}'
                     + (' [overridden]' if name in moved else ''))
    for name in sorted(moved - checked):
        mismatches += 1
        lines.append(f'MISMATCH {name}: overridden but no leg recorded the requested goal')
    return mismatches, lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='command', required=True)
    p = sub.add_parser('resolve')
    p.add_argument('experiment')
    p.add_argument('--base', required=True)
    p.add_argument('--out-dir', required=True)
    v = sub.add_parser('verify-live')
    v.add_argument('--params', required=True)
    v.add_argument('--out', required=True)
    g = sub.add_parser('verify-goals')
    g.add_argument('--resolved', required=True)
    g.add_argument('--bench', required=True)
    g.add_argument('--out', required=True)
    t = sub.add_parser('verify-topology')
    t.add_argument('--resolved', required=True)
    t.add_argument('--out', required=True)
    args = ap.parse_args(argv)

    if args.command == 'verify-topology':
        with open(args.resolved) as f:
            topology = json.load(f).get('topology', 'A')
        mismatches, lines = verify_topology(topology, args.out)
        print('\n'.join(lines))
        return 3 if mismatches else 0
    if args.command == 'verify-goals':
        with open(args.resolved) as f:
            resolved = json.load(f)
        with open(args.bench) as f:
            bench_doc = json.load(f)
        mismatches, lines = verify_goals(resolved, bench_doc)
        with open(args.out, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        print('\n'.join(lines))
        return 3 if mismatches else 0
    if args.command == 'verify-live':
        mismatches, lines = verify_live(args.params, args.out)
        print('\n'.join(lines))
        return 3 if mismatches else 0
    try:
        resolved = resolve(args.experiment, args.base, args.out_dir)
    except (ExperimentError, OSError, yaml.YAMLError) as e:
        print(f'nav_params_overlay: REFUSED: {e}', file=sys.stderr)
        return 2
    print(f"nav_params_overlay: {resolved['name']}: {len(resolved['changes'])} parameter "
          f"change(s); params {resolved['params_file']} sha256 {resolved['params_sha256']}")
    for change in resolved['changes']:
        print(f"  {change['path']}: {change['old']!r} -> {change['new']!r}")
    print(f"nav_params_overlay: {len(resolved['goal_changes'])} goal change(s)")
    for change in resolved['goal_changes']:
        print(f"  goal {change['scenario']}: {change['old']} -> {change['new']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
