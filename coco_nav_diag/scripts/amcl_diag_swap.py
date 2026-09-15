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
Swap the live /amcl component for coco_nav_diag::AmclNode.

nav.launch.py composes every Nav2 node into one component_container_isolated
(/nav2_container), so there is no AMCL process to replace. C2-NAV.37 did
this swap by hand; this tool makes it reproducible:

  1. list the container and find /amcl
  2. unload only /amcl, then verify every other component is still loaded
  3. load coco_nav_diag::AmclNode with the params file's amcl: block
     (flattened the way a params file is) plus the diag_* overrides
  4. bring it to active, issuing only the lifecycle transitions its CURRENT
     state needs -- nav2's lifecycle manager may activate the reloaded node
     itself, and racing it fails
  5. read diag_* and two AMCL parameters back off the RUNNING node -- an
     edited file and a loaded parameter are different claims

Usage:
  amcl_diag_swap.py load --params FILE --diag-output PATH [--diag-enabled false]
  amcl_diag_swap.py unload      # the node's destructor writes the capture footer
  Either subcommand accepts --container NAME and --dry-run.
"""
import argparse
import json
import re
import subprocess
import sys
import time

import yaml

DEFAULT_CONTAINER = '/nav2_container'
PLUGIN_PACKAGE = 'coco_nav_diag'
PLUGIN_CLASS = 'coco_nav_diag::AmclNode'
AMCL_NAME = '/amcl'
READBACK_PARAMS = ('resample_interval', 'robot_model_type')

_COMPONENT_LINE = re.compile(r'^\s*(\d+)\s+(/\S+)\s*$')
_LOADED_LINE = re.compile(
    r"Loaded component (\d+) into '([^']+)' container node as '([^']+)'")
_PARAM_GET = re.compile(r'^(Boolean|Integer|Double|String) value is: ?(.*)$')
_LIFECYCLE_STATE = re.compile(r'^\s*([a-z]+) \[\d+\]')


class SwapError(RuntimeError):
    pass


def flatten(value, prefix='', out=None):
    out = {} if out is None else out
    if isinstance(value, dict):
        for key, sub in value.items():
            flatten(sub, f'{prefix}.{key}' if prefix else str(key), out)
    else:
        out[prefix] = value
    return out


def param_arg(value):
    """Encode a value so ros2cli's yaml.safe_load parsing returns the same type."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        text = repr(value)
        if text in ('inf', '-inf', 'nan'):
            return {'inf': '.inf', '-inf': '-.inf', 'nan': '.nan'}[text]
        # PyYAML only reads a float when the mantissa has a dot: 1e-05 is a str.
        if 'e' in text and '.' not in text.split('e')[0]:
            mantissa, exponent = text.split('e')
            text = f'{mantissa}.0e{exponent}'
        return text
    return json.dumps(value)


def parse_components(text):
    comps = []
    for line in text.splitlines():
        m = _COMPONENT_LINE.match(line)
        if m:
            comps.append((int(m.group(1)), m.group(2)))
    return comps


def parse_param_get(text):
    lines = [ln for ln in text.strip().splitlines() if ln.strip()]
    m = _PARAM_GET.match(lines[-1].strip()) if lines else None
    if not m:
        raise SwapError(f'unparseable `ros2 param get` output: {text!r}')
    kind, raw = m.groups()
    if kind == 'Boolean':
        return raw == 'True'
    if kind == 'Integer':
        return int(raw)
    if kind == 'Double':
        return float(raw)
    return raw


def parse_lifecycle_state(text):
    for line in text.splitlines():
        m = _LIFECYCLE_STATE.match(line)
        if m:
            return m.group(1)
    return None


def amcl_params(path):
    with open(path) as f:
        doc = yaml.safe_load(f)
    try:
        block = doc['amcl']['ros__parameters']
    except (KeyError, TypeError):
        raise SwapError(f'{path} has no amcl.ros__parameters block')
    return flatten(block)


def load_command(container, params):
    cmd = ['ros2', 'component', 'load', container, PLUGIN_PACKAGE, PLUGIN_CLASS]
    for key in sorted(params):
        cmd += ['-p', f'{key}:={param_arg(params[key])}']
    return cmd


def run(cmd, dry_run, timeout=60.0, check=True):
    print('$ ' + ' '.join(cmd), flush=True)
    if dry_run:
        return ''
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if check and res.returncode != 0:
        raise SwapError(f'rc={res.returncode}: {" ".join(cmd[:5])}\n'
                        f'{res.stdout}{res.stderr}')
    return res.stdout if check else res.stdout + res.stderr


def unload_amcl(container, dry_run):
    before = parse_components(run(['ros2', 'component', 'list', container], dry_run))
    if dry_run:
        run(['ros2', 'component', 'unload', container, '<amcl uid>'], True)
        return
    amcl = [c for c in before if c[1] == AMCL_NAME]
    if len(amcl) != 1:
        raise SwapError(f'expected exactly one {AMCL_NAME} in {container}, found {before}')
    run(['ros2', 'component', 'unload', container, str(amcl[0][0])], False)
    after = parse_components(run(['ros2', 'component', 'list', container], False))
    others = [c for c in before if c[1] != AMCL_NAME]
    if after != others:
        raise SwapError(f'unloading {AMCL_NAME} disturbed other components: '
                        f'expected {others}, found {after}')
    print(f'amcl_diag_swap: unloaded {AMCL_NAME} (uid {amcl[0][0]}); '
          f'{len(others)} other components intact', flush=True)


def bring_to_active(dry_run, timeout=45.0):
    """
    Return the transitions this tool issued to reach active.

    Measured, C2-NAV.39: after a reload nav2's localization lifecycle
    manager sometimes configures and activates the new node itself, and an
    unconditional `configure` then fails with "Unknown transition requested".
    """
    if dry_run:
        for transition in ('configure', 'activate'):
            run(['ros2', 'lifecycle', 'set', AMCL_NAME, transition], True)
        return []
    issued = []
    state = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = parse_lifecycle_state(
            run(['ros2', 'lifecycle', 'get', AMCL_NAME], False, check=False))
        if state == 'active':
            return issued
        transition = {'unconfigured': 'configure', 'inactive': 'activate'}.get(state)
        if transition:
            out = run(['ros2', 'lifecycle', 'set', AMCL_NAME, transition], False, check=False)
            if 'successful' in out:
                issued.append(transition)
            continue
        time.sleep(0.5)
    raise SwapError(f'{AMCL_NAME} did not reach active within {timeout:g} s '
                    f'(last state {state!r})')


def do_load(args):
    params = amcl_params(args.params)
    n_file = len(params)
    overrides = {
        'use_sim_time': True,
        'diag_enabled': args.diag_enabled,
        'diag_output_path': args.diag_output,
        'diag_max_events': args.diag_max_events,
    }
    if args.diag_enabled and not args.diag_output:
        raise SwapError('--diag-output is required when --diag-enabled is true')
    params.update(overrides)

    unload_amcl(args.container, args.dry_run)
    out = run(load_command(args.container, params), args.dry_run)
    if not args.dry_run:
        m = _LOADED_LINE.search(out)
        if not m or m.group(3) != AMCL_NAME:
            raise SwapError(f'load did not report {AMCL_NAME}: {out!r}')
    issued = bring_to_active(args.dry_run)

    checks = {'diag_enabled': args.diag_enabled, 'diag_output_path': args.diag_output}
    for name in READBACK_PARAMS:
        if name in params:
            checks[name] = params[name]
    for name, want in checks.items():
        got = run(['ros2', 'param', 'get', AMCL_NAME, name], args.dry_run)
        if args.dry_run:
            continue
        value = parse_param_get(got)
        if value != want:
            raise SwapError(f'live {AMCL_NAME}.{name} = {value!r}, expected {want!r}')
    how = ', '.join(issued) if issued else 'none (the lifecycle manager activated it)'
    print(f'amcl_diag_swap: {PLUGIN_CLASS} active with {n_file} parameters from '
          f'{args.params} + {len(overrides)} overrides; transitions issued here: {how}; '
          f'readback {"skipped (dry run)" if args.dry_run else "OK"}: {checks}', flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='command', required=True)
    for name in ('load', 'unload'):
        p = sub.add_parser(name)
        p.add_argument('--container', default=DEFAULT_CONTAINER)
        p.add_argument('--dry-run', action='store_true')
        if name == 'load':
            p.add_argument('--params', required=True)
            p.add_argument('--diag-output', default='')
            p.add_argument('--diag-enabled', default='true', choices=('true', 'false'))
            p.add_argument('--diag-max-events', type=int, default=200000)
    args = ap.parse_args(argv)
    try:
        if args.command == 'load':
            args.diag_enabled = args.diag_enabled == 'true'
            do_load(args)
        else:
            unload_amcl(args.container, args.dry_run)
    except (SwapError, subprocess.TimeoutExpired, OSError) as e:
        print(f'amcl_diag_swap: FAILED: {e}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
