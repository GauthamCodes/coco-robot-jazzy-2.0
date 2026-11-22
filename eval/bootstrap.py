"""NON_QUALIFYING engineering execution; never calls the production batch runner.

Production host_policy.json is read-only. Capacity reserves below are emergency
abort budgets, not calibrated workload requirements or M1 qualification.
"""
import copy
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from evidence import atomic_json, bag_hash, bytes_used, disk_check, outside_git
from qualification import cpu_percent, identity, inventory, load_policy, qualify

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'scripts'))
from process_groups import stop


def safety_policy(snapshot, production, min_free_kbytes):
    """Reserve half physical RAM, at least 16 kernel low-memory reserves;
    reserve one third GPU memory. Deliberately conservative engineering budgets.
    Neither is a prediction of simulator requirements. Existing stricter committed
    resource limits take precedence. No swap is counted as available RAM.
    """
    gpu = snapshot.get('gpu') or {}
    if not snapshot.get('ram_total_bytes') or not gpu.get('total_bytes'):
        raise ValueError('Machine capacity unavailable; cannot derive safety reserve')
    ram = max(snapshot['ram_total_bytes'] // 2, min_free_kbytes * 1024 * 16)
    vram = (gpu['total_bytes'] + 2) // 3
    policy = copy.deepcopy(production)
    for backend in policy['backends']:
        for phase in ('preflight', 'runtime'):
            limits = policy['backends'][backend][phase]
            limits['min_ram_available_bytes'] = max(ram, limits.get('min_ram_available_bytes') or 0)
            limits['min_vram_free_bytes'] = max(vram, limits.get('min_vram_free_bytes') or 0)
            limits['max_load1'] = min(snapshot['cpu_count'], limits['max_load1'])
    return policy, dict(
        kind='BOOTSTRAP_EMERGENCY_RESERVE_NOT_PRODUCTION_THRESHOLD',
        ram_total_bytes=snapshot['ram_total_bytes'], vram_total_bytes=gpu['total_bytes'],
        kernel_min_free_kbytes=min_free_kbytes,
        derivation='Reserve 50% physical RAM and >=16x kernel min_free_kbytes; '
                   'reserve 1/3 physical VRAM; load ceiling <= logical CPUs and '
                   'committed load limit. Stricter calibrated limits take precedence. '
                   'Allocation bursts between 1Hz samples are not a hard kernel quota.',
        limits=policy['backends'])


def environment_reasons(env):
    reasons = []
    required = {'ROS_DISTRO': 'jazzy', 'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                'ROS2CLI_NO_DAEMON': '1', 'GZ_IP': '127.0.0.1', 'GZ_VERSION': 'harmonic'}
    for key, expected in required.items():
        if env.get(key) != expected:
            reasons.append(f'ENVIRONMENT: {key} must be {expected}')
    if 'name="lo"' not in env.get('CYCLONEDDS_URI', ''):
        reasons.append('ENVIRONMENT: explicit CycloneDDS loopback configuration required')
    if not env.get('AMENT_PREFIX_PATH'):
        reasons.append('ENVIRONMENT: ROS overlay not sourced')
    return reasons


def verdict(snapshot, policy, backend, domain, owned=(), phase='preflight'):
    check = qualify(snapshot, policy, backend, phase, owned=owned, domain=domain)
    return dict(check, qualified=False, bootstrap_safe=check['qualified'],
                execution_class='NON_QUALIFYING', m1_denominator_eligible=False,
                production_qualification='NOT_EVALUATED_BY_BOOTSTRAP')


def owned_descendants(snapshot, owned):
    """Parentage plus PID/start identity, never domain/argv as ownership proof."""
    current = {p['pid']: p for p in snapshot['processes']}
    pids = {p['pid'] for p in owned if p['pid'] in current and
            current[p['pid']]['start_ticks'] == p['start_ticks']}
    changed = True
    while changed:
        changed = False
        for p in current.values():
            if p['ppid'] in pids and p['pid'] not in pids:
                pids.add(p['pid']); changed = True
    known = {(p['pid'], p['start_ticks']) for p in owned}
    return list(owned) + [p for p in current.values() if p['pid'] in pids and
                         (p['pid'], p['start_ticks']) not in known]


def component_samples(snapshot, owned, previous):
    """Observed owned-process RSS and CPU; plugins share simulator accounting."""
    result=[]; current={}; now=time.monotonic(); hz=os.sysconf('SC_CLK_TCK')
    identities={(p['pid'],p['start_ticks']) for p in owned}
    for p in snapshot['processes']:
        key=(p['pid'],p['start_ticks'])
        if key not in identities:continue
        path=Path('/proc')/str(p['pid'])
        try:
            fields=(path/'stat').read_text().rsplit(')',1)[1].split()
            ticks=int(fields[11])+int(fields[12])
            rss=int(next(line.split()[1] for line in (path/'status').read_text().splitlines() if line.startswith('VmRSS:')))*1024
        except (OSError,StopIteration,ValueError):continue
        current[key]=(ticks,now)
        old=previous.get(key);cpu=None if old is None else 100*(ticks-old[0])/hz/(now-old[1])
        argv=' '.join(p.get('argv',[]));component='launch_or_runtime_overhead'
        for label, signatures in (
            ('gazebo_including_sensor_plugins',('gz sim','gzserver')),
            ('ros_bridge_middleware',('parameter_bridge',)),
            ('robot_tf',('robot_state_publisher',)),
            ('recorder',('bootstrap_recorder.py','eval/recorder.py')),
            ('observer',('object_height_node',)),
            ('nav2',('planner_server','controller_server','bt_navigator','amcl','map_server','nav2_container')),
            ('moveit_manipulation',('move_group','arm_control','grasp_server')),
            ('mission',('mission_node','mission_sequencer'))):
            if any(signature in argv for signature in signatures):component=label;break
        from resource_attribution import process_memory
        result.append(dict(pid=p['pid'],start_ticks=p['start_ticks'],component=component,
                           rss_bytes=rss,cpu_percent_one_core=cpu, **process_memory(p['pid'],p['start_ticks'])))
    return result,current


def run_bootstrap(output, backend, domain=82, seconds=120, preflight_only=False, profile="full", fixture="acquisition"):
    if not 1 <= seconds <= 600 or not 0 <= domain <= 232:
        raise ValueError('Bootstrap requires 1..600 wall seconds and domain 0..232')
    if profile not in ('full', 'rgbd_observation', 'rgbd_capture', 'rgbd_manipulation') or (profile != 'full' and backend != 'gazebo'):
        raise ValueError('Sensor profile currently supports Gazebo only')
    from rgbd_fixture import FIXTURES
    if fixture not in FIXTURES:
        raise ValueError('Unknown engineering fixture')
    root = outside_git(output)
    root.mkdir(parents=True, exist_ok=False)
    (root / 'logs').mkdir()
    manifest = dict(schema_version=1, execution_class='NON_QUALIFYING', qualified=False,
                    m1_denominator_eligible=False, satisfies_m1_8=False, test_a=False,
                    state='CREATED', profile=profile, fixture=fixture, backend=backend, seed=5, seed_group='dev',
                    requested_colour='red', home_xy_m=[-2., 0.],
                    source_sha=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip(),
                    domain=domain, owned_processes=[],
                    purpose='Engineering RGB-D visibility / fail-closed control diagnostic',
                    ground_truth_use='Evaluator-only post-run validation; never autonomy input')
    atomic_json(root / 'manifest.json', manifest)
    # Global lock shared with all bootstrap backends; production still sees owned
    # robotics and refuses concurrent launch. Lock is never used as ownership proof.
    with (root.parent / '.coco-bootstrap.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        snap = inventory()
        production = load_policy()
        try:
            policy, envelope = safety_policy(snap, production,
                int(Path('/proc/sys/vm/min_free_kbytes').read_text()))
        except ValueError as exc:
            manifest.update(state='BLOCKED_BOOTSTRAP', reasons=[str(exc)])
            atomic_json(root / 'manifest.json', manifest)
            return 3
        atomic_json(root / 'safety_envelope.json', envelope)
        atomic_json(root / 'production_policy.json', production)
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS2CLI_NO_DAEMON='1')
        atomic_json(root / 'bootstrap_environment.json', {key:env.get(key) for key in (
            'ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION','CYCLONEDDS_URI',
            'ROS2CLI_NO_DAEMON','GZ_IP','GZ_VERSION','AMENT_PREFIX_PATH','COCO_WS')})
        check = verdict(snap, policy, backend, domain)
        reasons = check['reasons'] + environment_reasons(env)
        disk = disk_check(root, json.loads((REPO / 'eval/storage_policy.json').read_text()), starting=True)
        reasons += disk['reasons']
        if subprocess.check_output(['git', 'status', '--porcelain=v1'], cwd=REPO, text=True).strip():
            reasons.append('WORKTREE_DIRTY: commit before simulation')
        atomic_json(root / 'logs/preflight.json', dict(inventory=snap, verdict=check,
                                                      environment_reasons=environment_reasons(env), disk=disk))
        if reasons or preflight_only:
            manifest.update(state='BLOCKED_BOOTSTRAP' if reasons else 'SAFE_BOOTSTRAP_PREFLIGHT_ONLY', reasons=reasons)
            atomic_json(root / 'manifest.json', manifest)
            return 3 if reasons else 0
        from environment import capture
        atomic_json(root / 'env.json', capture(Path.home() / 'isaac-sim/venv/bin/python'))
        children, handles, owned, samples = [], [], [], []
        storage = json.loads((REPO / 'eval/storage_policy.json').read_text())
        started = time.monotonic(); previous = snap; last_clock = None; process_previous = {}
        manifest['state'] = 'RUNNING_BOOTSTRAP'
        def save():
            manifest['owned_processes'] = owned
            atomic_json(root / 'manifest.json', manifest)
        def spawn(name, argv):
            handle = (root / 'logs' / (name + '.log')).open('x'); handles.append(handle)
            child = subprocess.Popen(argv, cwd=REPO, env=env, stdout=handle,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            children.append((name, child))
            owned.append(dict(identity(child.pid),spawn_observed_wall_epoch_ns=time.time_ns(),spawn_observed_monotonic_s=time.monotonic()))
            save()
            return child
        recorder = runner = truth = None
        try:
            save()
            recorder = spawn('recorder', [sys.executable, str(REPO / 'eval/bootstrap_recorder.py'), str(root)])
            ready_deadline = time.monotonic() + 30
            while time.monotonic() - started < seconds:
                tick = time.monotonic()
                snap = inventory(); owned = owned_descendants(snap, owned)
                check = verdict(snap, policy, backend, domain, owned, 'runtime')
                reasons = list(check['reasons'])
                disk = disk_check(root, storage)
                reasons += disk['reasons']
                if bytes_used(root) > storage['max_run_bytes']:
                    reasons.append('RUN_DISK_BUDGET_EXCEEDED')
                clock_path = root / 'logs/clock_latest.json'
                clock = json.loads(clock_path.read_text()) if clock_path.exists() else {}
                pair = (clock.get('sim_s'), clock.get('clock_receive_monotonic_s'))
                rtf = None
                if last_clock and all(v is not None for v in (*pair, *last_clock)) and pair[1] > last_clock[1]:
                    rtf = (pair[0] - last_clock[0]) / (pair[1] - last_clock[1])
                row = dict(wall_epoch_ns=snap['wall_epoch_ns'], wall_elapsed_s=tick-started,
                           cpu_percent=cpu_percent(previous, snap), load1=snap['load1'],
                           ram_available_bytes=snap['ram_available_bytes'],
                           ram_used_bytes=snap['ram_total_bytes']-snap['ram_available_bytes'],
                           vram_free_bytes=(snap.get('gpu') or {}).get('free_bytes'),
                           vram_used_bytes=(snap['gpu']['total_bytes']-snap['gpu']['free_bytes']) if snap.get('gpu') else None,
                           disk_free_bytes=disk['free_bytes'], evidence_bytes=disk['used_bytes'],
                           sim_s=pair[0], rtf=rtf, reasons=reasons)
                components, process_previous = component_samples(snap, owned, process_previous)
                row['owned_components'] = components
                from resource_attribution import pressure
                row['memory_pressure'] = pressure()
                row['queue_usage'] = 'DDS occupancy unavailable; configured engineering image/observation depth=1'
                row['clock_receive_monotonic_s'] = pair[1]
                samples.append(row)
                with (root / 'host_metrics.jsonl').open('a') as stream:
                    stream.write(json.dumps(row, allow_nan=False) + '\n'); stream.flush()
                atomic_json(root / 'logs/inventory_latest.json', snap)
                previous = snap; last_clock = pair; save()
                if reasons:
                    resource = any(r.startswith(('min_', 'max_', 'RUN_DISK', 'FILESYSTEM', 'BATCH_DISK')) for r in reasons)
                    manifest.update(state='ABORTED_BOOTSTRAP_RESOURCE_LIMIT' if resource else 'ABORTED_BOOTSTRAP_SAFETY', reasons=reasons)
                    break
                if recorder.poll() is not None:
                    raise RuntimeError('MCAP recorder exited')
                if runner is None:
                    if (root / 'logs/recorder_ready.json').exists():
                        if profile in ('rgbd_observation', 'rgbd_capture', 'rgbd_manipulation'):
                            runner = spawn('sensor_fixture', [sys.executable, str(REPO / 'eval/rgbd_fixture.py'), str(root / 'logs/sensor_fixture'), fixture, profile])
                            continue
                        runner = spawn('episode', [sys.executable, str(REPO / 'scripts/isaac_fetch.py'),
                            str(root / 'logs/episode'), '--backend', backend, '--level', 'positions',
                            '--seed', '5', '--colour', 'red', '--domain', str(domain),
                            '--timeout', str(seconds), '--eval-logging'])
                    elif tick > ready_deadline:
                        raise RuntimeError('MCAP recorder startup deadline')
                episode = root / 'logs/episode/manifest.json'
                if backend == 'gazebo' and runner is not None and truth is None and episode.exists():
                    truth = spawn('truth', [sys.executable, str(REPO / 'eval/gazebo_truth.py'), str(episode)])
                if truth is not None and truth.poll() is not None:
                    raise RuntimeError('Evaluator truth adapter exited')
                if runner is not None and runner.poll() is not None:
                    manifest.update(state='BOOTSTRAP_EPISODE_ENDED', episode_returncode=runner.returncode)
                    break
                time.sleep(max(0, 1-(time.monotonic()-tick)))
            else:
                manifest['state'] = 'BOOTSTRAP_WALL_BUDGET_REACHED'
        except BaseException as exc:
            manifest.update(state='ABORTED_BOOTSTRAP_ERROR', reasons=[repr(exc)])
        finally:
            errors = []
            for name, child in sorted(children, key=lambda item: item[0] == 'recorder'):
                try: stop(name, child)
                except Exception as exc: errors.append(repr(exc))
            for handle in handles: handle.close()
            after = inventory(); survivors = []
            for saved in owned:
                try: current = identity(saved['pid'])
                except OSError: continue
                if current['start_ticks'] == saved['start_ticks'] and current['state'] != 'Z':
                    survivors.append(current)
            atomic_json(root / 'logs/teardown.json', dict(inventory=after, survivors=survivors, errors=errors))
            if survivors or errors:
                manifest.update(state='ABORTED_BOOTSTRAP_TEARDOWN', teardown_errors=errors)
            if (root / 'bag').exists(): manifest['bag_hash'] = bag_hash(root / 'bag')
            metrics = ('cpu_percent', 'load1', 'ram_used_bytes', 'ram_available_bytes',
                       'vram_used_bytes', 'vram_free_bytes', 'rtf', 'evidence_bytes', 'disk_free_bytes')
            atomic_json(root / 'resource_summary.json', dict(qualified=False, execution_class='NON_QUALIFYING',
                sampling='1 Hz host-wide; GPU/RAM include background usage; RTF clock receipt deltas',
                sample_count=len(samples), extrema={k:dict(min=min(v), max=max(v))
                    for k in metrics if (v := [s[k] for s in samples if s[k] is not None])},
                full_stack_envelope_verified=False))
            # Post-run evaluator only; engineering fixture has a separate truth manifest.
            fixture_manifest = root / 'logs/sensor_fixture/truth_manifest.json'
            if fixture_manifest.exists():
                manifest['targets'] = json.loads(fixture_manifest.read_text())['targets']
                # Generated fixture starts at origin, unlike the benchmark home.
                manifest['home_xy_m'] = [0., 0.]
            # Post-run evaluator only; missing evidence remains unknown.
            from capture_status import summarize
            manifest['capture'] = summarize(root,manifest['state'],manifest.get('reasons',[]))
            atomic_json(root/'capture_status.json',manifest['capture'])
            try:
                from batch import read_events
                if (root / 'logs/episode/manifest.json').exists():
                    episode = json.loads((root / 'logs/episode/manifest.json').read_text())
                    manifest['targets'] = episode['targets']
                from evaluation import evaluate_recording
                thresholds = json.loads((REPO/'eval/thresholds.json').read_text())['thresholds']
                recorded=manifest['capture']['has_trace']
                pipeline = evaluate_recording(read_events(root) if recorded else [],manifest,thresholds,
                    integrity_errors=[] if recorded else ['No events.jsonl: recorder did not reach capture'])
                result = pipeline['oracle']
                atomic_json(root/'logs/oracle_facts.json',pipeline['facts'])
                for name in ('oracle','analysis','manipulation'):
                    atomic_json(root/(name+'.json'),pipeline[name])
                with (root/'logs/normalized.jsonl').open('x') as stream:
                    for event in pipeline['trace']:stream.write(json.dumps(event,allow_nan=False)+'\n')
                if result.get('mission_oracle_disagreement') is True:
                    manifest['state'] = 'STOP_ORACLE_DISAGREEMENT'
            except Exception as exc:
                atomic_json(root / 'logs/oracle_error.json', dict(error=repr(exc)))
            manifest['wall_time_s'] = time.monotonic()-started
            save()
        return 0 if manifest['state'] == 'BOOTSTRAP_EPISODE_ENDED' else 3
