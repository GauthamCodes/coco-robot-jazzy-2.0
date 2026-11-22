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

"""Run one owned Isaac session with observable startup and safe teardown."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

from coco_sim.isaac.session import (
    classify_nav,
    nav_parameters,
    process_identity,
    same_process,
    session_members,
)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('output', type=Path)
    p.add_argument('--domain', type=int, default=78)
    p.add_argument('--gui', action='store_true')
    p.add_argument('--rviz', action='store_true')
    p.add_argument('--rviz-software', action='store_true')
    p.add_argument('--rviz-native', action='store_true', help='diagnose native GPU teardown')
    p.add_argument('--rviz-stock', action='store_true', help='diagnose upstream RViz teardown')
    p.add_argument('--ideal-odom-amcl', action='store_true')
    p.add_argument('--nav', action='store_true')
    p.add_argument('--nav-composed', action='store_true')
    p.add_argument('--raw-evidence', type=Path)
    p.add_argument(
        '--ack-ms',
        type=int,
        default=200,
        help='BT acknowledgement budget; 0 retains the installed default (diagnosis)',
    )
    p.add_argument('--planner-probes', type=int, default=0)
    p.add_argument('--goals', type=int, default=1)
    p.add_argument('--startup-only', action='store_true')
    p.add_argument('--inject-failure', choices=('after-ready', 'during-goal'))
    args = p.parse_args()
    if args.nav and (
        not args.raw_evidence or not json.loads(args.raw_evidence.read_text()).get('passed')
    ):
        p.error('--nav requires a passing --raw-evidence result')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[1]
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain))
    env['ROS_LOG_DIR'] = str(out / 'ros_log')
    start = time.monotonic()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    records, handles, processes, rc, gpu = {}, [], {}, {}, []
    result = {
        'passed': False,
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'options': vars(args) | {'output': str(out), 'raw_evidence': str(args.raw_evidence)},
    }
    events = []
    stage = 'startup failure'
    owned_members = {}

    def snapshot_owned():
        for record in records.values():
            if same_process(record):
                for member in session_members(record['pid']):
                    owned_members[(member['pid'], member['start_ticks'])] = member

    def event(kind, **fields):
        row = {
            'event': kind,
            'elapsed_wall_s': time.monotonic() - start,
            'wall_monotonic_s': time.monotonic(),
            **fields,
        }
        events.append(row)
        (out / 'session.json').write_text(
            json.dumps({'processes': records, 'events': events}, indent=2)
        )

    def spawn(name, command, extra_env=None):
        handle = (out / (name + '.log')).open('w')
        handles.append(handle)
        process = subprocess.Popen(
            command,
            cwd=repo,
            env=env | (extra_env or {}),
            stdout=handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        processes[name] = process
        records[name] = process_identity(process.pid)
        records[name]['command'] = command
        records[name]['environment_overrides'] = extra_env or {}
        event('spawn', name=name, pid=process.pid)
        return process

    def wait_exit(name, timeout):
        process = processes[name]
        try:
            process.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            return False

    def close_rviz(process):
        if not args.rviz_stock:
            (out / 'rviz.stop').write_text('stop')
            event('rviz_cooperative_stop', pid=process.pid)
            return wait_exit('rviz', 15)
        # Request normal Qt/window-manager close, before destroying ROS/GL peers.
        try:
            listing = subprocess.check_output(['wmctrl', '-lp'], text=True)
            for row in listing.splitlines():
                fields = row.split(None, 4)
                if len(fields) >= 3 and fields[2] == str(process.pid):
                    subprocess.run(['wmctrl', '-ic', fields[0]], check=True)
                    event('window_close', pid=process.pid, window=fields[0])
            return wait_exit('rviz', 15)
        except (OSError, subprocess.CalledProcessError) as exc:
            event('window_close_error', error=repr(exc))
            return False

    def stop(name):
        process = processes[name]
        record = records[name]
        snapshot_owned()
        if process.poll() is None and same_process(record):
            closed = name == 'rviz' and close_rviz(process)
            if not closed:
                event('signal', name=name, signal='SIGINT')
                # Signal leader only; ros2 launch forwards to its own children.
                # /usr/bin/time exec wrapper needs the owned process group.
                if name == 'launch':
                    os.killpg(process.pid, signal.SIGINT)
                else:
                    process.send_signal(signal.SIGINT)
                if not wait_exit(name, 30):
                    for sig in (signal.SIGTERM, signal.SIGKILL):
                        event('recovery_signal', name=name, signal=sig.name)
                        for member in session_members(process.pid):
                            if same_process(member):
                                os.kill(member['pid'], sig)
                        if wait_exit(name, 10):
                            break
        rc[name] = process.poll()
        event('exit', name=name, code=rc[name])

    def run_probe(name, phase, timeout=180):
        process = spawn(
            name,
            [
                'python3',
                str(repo / 'scripts/isaac_session_probe.py'),
                str(out / (name + '.json')),
                '--phase',
                phase,
                '--planner-probes',
                str(args.planner_probes if phase == 'nav' else 0),
            ],
        )
        if not wait_exit(name, timeout):
            raise RuntimeError(name + ' exceeded wall deadline')
        return json.loads((out / (name + '.json')).read_text())

    try:
        isaac = spawn(
            'launch',
            [
                '/usr/bin/time',
                '-v',
                '-o',
                str(out / 'resources.txt'),
                'bash',
                str(repo / 'scripts/isaac_sim_coco.sh'),
                '--output',
                str(out / 'sim'),
                *(['--gui'] if args.gui else []),
            ],
        )
        deadline = time.monotonic() + 1200
        while not (out / 'sim/ready.marker').exists():
            if isaac.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Isaac initialization exited or timed out')
            time.sleep(0.2)  # file predicate polling, not a startup delay
        event('simulator_ready_marker')
        stage = 'ROS discovery failure'
        state = run_probe('readiness', 'state')
        if not state['passed']:
            stage = state['failure_class']
            raise RuntimeError(state.get('error'))
        event('ros_state_ready')
        spawn(
            'ros_peer',
            [
                'python3',
                str(repo / 'scripts/isaac_ros.py'),
                '--ros-args',
                '-p',
                'use_sim_time:=true',
                '-p',
                'initial_mode:=idle',
            ],
        )
        if args.rviz:
            # RViz may save view/window state on close; never write the source config.
            import yaml

            rviz_config = yaml.safe_load((repo / 'coco_sim/rviz/isaac.rviz').read_text())
            for display in rviz_config['Visualization Manager']['Displays']:
                display['Enabled'] = True
                display['Value'] = True
            (out / 'isaac.rviz').write_text(yaml.safe_dump(rviz_config))
            renderer_env = env | (
                {
                    'LIBGL_ALWAYS_SOFTWARE': '1',
                    '__GLX_VENDOR_LIBRARY_NAME': 'mesa',
                    'LP_NUM_THREADS': '2',
                }
                if not args.rviz_native
                else {}
            )
            renderer = subprocess.run(
                ['glxinfo', '-B'], env=renderer_env, capture_output=True, text=True
            )
            (out / 'renderer.txt').write_text(renderer.stdout + renderer.stderr)
            spawn(
                'rviz',
                [
                    (
                        '/opt/ros/jazzy/lib/rviz2/rviz2'
                        if args.rviz_stock
                        else str(
                            Path(os.environ.get('COCO_WS', str(Path.home() / 'coco_isaac_ws')))
                            / 'isaac_rviz/managed_rviz'
                        )
                    ),
                    '-d',
                    str(out / 'isaac.rviz'),
                    '--ros-args',
                    '-p',
                    'use_sim_time:=true',
                ],
                (
                    {
                        'LIBGL_ALWAYS_SOFTWARE': '1',
                        '__GLX_VENDOR_LIBRARY_NAME': 'mesa',
                        'LP_NUM_THREADS': '2',
                        'COCO_RVIZ_STOP_FILE': str(out / 'rviz.stop'),
                    }
                    if not args.rviz_native
                    else {'COCO_RVIZ_STOP_FILE': str(out / 'rviz.stop')}
                ),
            )
        if args.inject_failure == 'after-ready':
            raise RuntimeError('injected failure after readiness')
        if args.nav:
            import yaml
            from isaac_fixture_map import write_map

            write_map(out / 'map')
            params = nav_parameters(
                yaml.safe_load((repo / 'gazebo_models/config/nav2_params.yaml').read_text()),
                out / 'map/fixture.yaml',
                args.ack_ms or None,
            )
            if args.ideal_odom_amcl:
                # Isaac odometry is measured ground truth, not noisy wheel integration.
                for key in ('alpha1', 'alpha2', 'alpha3', 'alpha4', 'alpha5'):
                    params['amcl']['ros__parameters'][key] = 0.0
            (out / 'nav2_params.yaml').write_text(yaml.safe_dump(params))
            spawn(
                'nav2',
                [
                    'ros2',
                    'launch',
                    'gazebo_models',
                    'nav.launch.py',
                    'arbiter:=true',
                    'use_composition:=' + ('True' if args.nav_composed else 'False'),
                    'params_file:=' + str(out / 'nav2_params.yaml'),
                    'map:=' + str(out / 'map/fixture.yaml'),
                ],
            )
        if args.startup_only:
            result.update(behavior_passed=True)
        else:
            command = (
                [
                    'python3',
                    str(repo / 'scripts/isaac_session_probe.py'),
                    str(out / 'navigation.json'),
                    '--planner-probes',
                    str(args.planner_probes),
                    '--goals',
                    str(args.goals),
                ]
                if args.nav
                else [
                    'python3',
                    str(repo / 'scripts/isaac_smoke_observer.py'),
                    str(out / 'raw.json'),
                ]
            )
            observer = spawn('observer', command)
            deadline = time.monotonic() + 360
            while observer.poll() is None:
                snapshot_owned()
                if isaac.poll() is not None:
                    stage = 'simulator execution failure'
                    raise RuntimeError('Isaac exited during observation')
                if time.monotonic() > deadline:
                    raise RuntimeError('observer wall deadline exceeded')
                if args.inject_failure == 'during-goal':
                    tracefile = out / 'navigation.jsonl'
                    if (
                        tracefile.exists()
                        and '"event": "goal_acknowledged"' in tracefile.read_text()
                    ):
                        raise RuntimeError('injected failure after goal accepted')
                sample = subprocess.run(
                    ['nvidia-smi', '--query-gpu=memory.used', '--format=csv,noheader,nounits'],
                    capture_output=True,
                    text=True,
                )
                gpu.append(
                    {
                        'elapsed_wall_s': time.monotonic() - start,
                        'total_gpu_used_mib': sample.stdout.strip(),
                    }
                )
                time.sleep(0.2)
            observation = json.loads(
                (out / ('navigation.json' if args.nav else 'raw.json')).read_text()
            )
            result.update(
                behavior_passed=observation['passed'],
                failure_class=observation.get('failure_class'),
                observation_file='navigation.json' if args.nav else 'raw.json',
            )
    except BaseException as exc:
        result.update(error=repr(exc), failure_class=stage, behavior_passed=False)
        event('failure', error=repr(exc), failure_class=stage)
    finally:
        # First inhibit autonomous commands through the existing arbiter and prove
        # stationary state. This also cancels any accepted navigation goal.
        if 'ros_peer' in processes and processes['ros_peer'].poll() is None:
            try:
                stop_result = run_probe('safe_stop', 'stop', 30)
                result['zero_motion_verified'] = stop_result['passed']
            except BaseException as exc:
                result['zero_motion_verified'] = False
                event('safe_stop_failed', error=repr(exc))
        for name in ('observer', 'nav2', 'rviz', 'ros_peer', 'safe_stop', 'readiness', 'launch'):
            if name in processes:
                stop(name)
        leftovers = [
            member
            for member in owned_members.values()
            if same_process(member) and process_identity(member['pid'])['state'] != 'Z'
        ]
        result['leftover_processes'] = leftovers
        # Bounded recovery of verified owned descendants, even if launch leader died.
        for member in leftovers:
            if same_process(member):
                event('recovery_signal', pid=member['pid'], signal='SIGTERM')
                os.kill(member['pid'], signal.SIGTERM)
        recovery_deadline = time.monotonic() + 5
        while (
            any(same_process(m) and process_identity(m['pid'])['state'] != 'Z' for m in leftovers)
            and time.monotonic() < recovery_deadline
        ):
            time.sleep(0.05)
        remaining = [
            m for m in leftovers if same_process(m) and process_identity(m['pid'])['state'] != 'Z'
        ]
        for member in remaining:
            event('recovery_signal', pid=member['pid'], signal='SIGKILL')
            os.kill(member['pid'], signal.SIGKILL)
        result['recovery_killed'] = remaining
        for handle in handles:
            handle.close()
        (out / 'process_exit.json').write_text(json.dumps(rc, indent=2))
        (out / 'gpu.json').write_text(json.dumps(gpu, indent=2))
        clean = (out / 'sim/shutdown_requested.json').exists() and not (
            out / 'sim/failure.txt'
        ).exists()
        result['clean_shutdown'] = clean and rc.get('launch') == 0
        navlog = (out / 'nav2.log').read_text() if (out / 'nav2.log').exists() else ''
        result['child_exit_failures'] = re.findall(r'process has died.*exit code (-?\d+)', navlog)
        result['failure_class'] = classify_nav(navlog, result.get('failure_class'))
        recovery = any(e['event'] == 'recovery_signal' for e in events)
        result['auxiliary_clean_shutdown'] = (
            not recovery
            and not leftovers
            and not result['child_exit_failures']
            and all(value == 0 for value in rc.values())
        )
        if rc.get('rviz', 0) != 0:
            result['rviz_failure'] = True
        result['passed'] = bool(
            result.get('behavior_passed')
            and result['clean_shutdown']
            and result['auxiliary_clean_shutdown']
            and result.get('zero_motion_verified')
        )
        if not result['passed'] and not result.get('failure_class'):
            result['failure_class'] = (
                'RViz failure' if result.get('rviz_failure') else 'shutdown failure'
            )
        result['elapsed_wall_s'] = time.monotonic() - start
        (out / 'result.json').write_text(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
