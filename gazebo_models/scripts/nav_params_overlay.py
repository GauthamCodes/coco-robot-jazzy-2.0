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

`resolve` writes RUN_DIR/experiment_resolved.json and, only when something
changes, RUN_DIR/params_merged.yaml (neither name contains ros_clean.sh's
'nav2_'). `verify-live` reads the accepted and safety-critical parameters
back off the RUNNING nodes and compares them with the file: a file that was
edited and a parameter that was loaded are different claims.
"""
import argparse
import copy
import hashlib
import json
import os
import re
import subprocess
import sys

import yaml

TOP_KEYS = {'name', 'description', 'bench', 'amcl_diag', 'nav2_overrides',
            'allow_safety_change'}
BENCH_KEYS = {'repeats', 'timeout', 'only'}
AMCL_DIAG_KEYS = {'enabled'}
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
    diag = doc.get('amcl_diag') or {}
    _only_keys(diag, AMCL_DIAG_KEYS, 'amcl_diag')
    enabled = diag.get('enabled', False)
    if not isinstance(enabled, bool):
        raise ExperimentError('amcl_diag.enabled must be true or false')
    allow = doc.get('allow_safety_change', False)
    if not isinstance(allow, bool):
        raise ExperimentError('allow_safety_change must be true or false')
    return {
        'name': name,
        'description': doc.get('description', ''),
        'bench': {'repeats': repeats, 'timeout': timeout, 'only': only},
        'amcl_diag': {'enabled': enabled},
        'nav2_overrides': doc.get('nav2_overrides') or {},
        'allow_safety_change': allow,
    }


def resolve(experiment_path, base_path, out_dir):
    exp = load_experiment(experiment_path)
    with open(base_path) as f:
        base = yaml.safe_load(f)
    merged, changes = merge_overrides(base, exp['nav2_overrides'], exp['allow_safety_change'])
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
    for node, yaml_path, param in LIVE_CHECKS:
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
    args = ap.parse_args(argv)

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
    return 0


if __name__ == '__main__':
    sys.exit(main())
