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
Summarise a p03c episode matrix: one row per run, every number from a file.

    python3 docs/data/p03c_matrix_report.py ROOT [--json OUT.json] [--md OUT.md]

Reads what p03c_episode_run.sh left in each ROOT/<run>/ and nothing else.
A missing file is reported as missing, never filled in.
"""

import argparse
import json
import math
import os
import re

from coco_config.robot import FIXED_REGION_MAP, region_by_id, SPAWN_XY

ORDER = ('fixed_red', 'fixed_green', 'fixed_blue', 'fixed_yellow',
         'colours_s1', 'colours_s2', 'colours_s4',
         'positions_s1', 'positions_s2', 'positions_s4')


def _json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _text(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return ''


def _field(line, key):
    match = re.search(rf'(?:^| ){key}=(\S+)', line)
    return match.group(1) if match else None


def row(run_dir):
    """Build one report row from a run directory."""
    name = os.path.basename(run_dir.rstrip('/'))
    out = {'run': name}
    manifest = _json(os.path.join(run_dir, 'manifest.json'))
    if manifest is None:
        out['error'] = 'no manifest.json'
        return out
    colour = manifest['requested_colour']
    target = next(t for t in manifest['targets'] if t['colour'] == colour)
    out.update({
        'level': manifest['level'], 'seed': manifest['seed'],
        'episode_id': manifest['episode_id'], 'requested': colour,
        'region': target['region_id'],
        'frozen_region': FIXED_REGION_MAP[colour],
        'target_xy': [round(target['x'], 4), round(target['y'], 4)],
        'region_map': {t['colour']: t['region_id']
                       for t in manifest['targets']},
    })

    spawn = _json(os.path.join(run_dir, 'readback_spawn.json'))
    if spawn:
        errs = spawn['models'].values()
        out['spawn_ok'] = spawn['ok']
        out['spawn_max_xy_m'] = max(math.hypot(e['dx'], e['dy'])
                                    for e in errs)
        out['spawn_max_abs_dz_m'] = max(abs(e['dz']) for e in errs)
        out['spawn_max_tilt_rad'] = max(e['tilt'] for e in errs)
    else:
        out['spawn_ok'] = None

    out['region_params'] = _text(os.path.join(run_dir,
                                              'region_params.txt')).strip()
    final = _text(os.path.join(run_dir, 'final_state.txt')).splitlines()
    final = final[0] if final else ''
    out['state'] = _field(final, 'state')
    out['reason'] = _field(final, 'reason')
    out['result'] = _field(final, 'result')

    result = _json(os.path.join(run_dir, 'result.json'))
    if result:
        out['outcome'] = result['outcome']
        out['failure_reason'] = result['failure_reason']
        out['mission_sim_s'] = result['timings'].get('mission_sim_s')
        out['mission_wall_s'] = result['timings'].get('mission_wall_s')

    mission_log = _text(os.path.join(run_dir, 'mission.log'))
    arrived = re.search(r'arrived: target axis at base-x ([0-9.]+) '
                        r'\(window centre ([0-9.]+)\), y ([+-][0-9.]+)',
                        mission_log)
    if arrived:
        out['approach_x'] = float(arrived.group(1))
        out['approach_window_centre'] = float(arrived.group(2))
        out['approach_y'] = float(arrived.group(3))
    travel = re.search(r'approach finished: arrived after ([0-9.]+) m',
                       mission_log)
    out['approach_travel_m'] = float(travel.group(1)) if travel else None
    exec_up = re.search(r'mission_executive up: colour=\S+ lane=([+-][0-9.]+)'
                        r'( \(region (\S+), episode map\))?', mission_log)
    if exec_up:
        out['executive_lane'] = float(exec_up.group(1))
        out['executive_region'] = exec_up.group(3)

    end = _json(os.path.join(run_dir, 'final_gz.json')) or {}
    coco = end.get('coco')
    if coco:
        out['final_robot_xy'] = [round(coco['x'], 3), round(coco['y'], 3)]
        out['final_home_error_m'] = round(math.hypot(
            coco['x'] - SPAWN_XY[0], coco['y'] - SPAWN_XY[1]), 3)
    back = _json(os.path.join(run_dir, 'readback_end.json'))
    if back:
        obs = back.get('observed', {}).get(target['model'])
        if obs:
            out['final_target_xyz'] = [round(obs['x'], 3), round(obs['y'], 3),
                                       round(obs['z'], 3)]

    # Ground truth where the robot stood as each leg began: the recorder's
    # trace carries x, y from gz and the mission state per row.
    trace = os.path.join(run_dir, 'cmdpath', 'trace.csv')
    first = {}
    try:
        with open(trace) as f:
            header = f.readline().strip().split(',')
            ix, iy, ist = (header.index('x'), header.index('y'),
                           header.index('mission_state'))
            for line in f:
                cols = line.rstrip('\n').split(',')
                state = cols[ist] if ist < len(cols) else ''
                if state and state not in first and cols[ix] and cols[iy]:
                    first[state] = (float(cols[ix]), float(cols[iy]))
    except (OSError, ValueError):
        pass
    # ALIGN_FOR_CLIMB can last less than one trace row; the first CLIMB
    # row is then the nearest recorded pose to the end of the Nav2 leg.
    arrival = next((s for s in ('ALIGN_FOR_CLIMB', 'CLIMB') if s in first),
                   None)
    if arrival:
        out['pre_ramp_state_sampled'] = arrival
        x, y = first[arrival]
        out['pre_ramp_gt_xy'] = [round(x, 3), round(y, 3)]
        lane = region_by_id(target['region_id']).lane_y
        out['pre_ramp_lane_error_m'] = round(y - lane, 3)
        out['pre_ramp_frozen_lane_error_m'] = round(
            y - region_by_id(FIXED_REGION_MAP[colour]).lane_y, 3)
    if 'SEARCH_TARGET' in first:
        out['climb_end_gt_xy'] = [round(v, 3) for v in first['SEARCH_TARGET']]

    summary = _json(os.path.join(run_dir, 'cmdpath', 'summary.json')) or {}
    bypass = summary.get('bypass_source', {})
    out['bypass_raw_controller'] = bypass.get('wheel_matches_raw_controller')
    breach = summary.get('stop_breach', {})
    out['stop_rows_wheels_driven'] = breach.get('stop_rows_wheels_driven')
    runner = _text(os.path.join(run_dir, 'runner.log'))
    out['runner_checks'] = ('PASS' if 'runner checks PASS' in runner
                            else 'FAIL' if 'runner checks FAIL' in runner
                            else 'INCOMPLETE')
    out['failed_checks'] = re.findall(r'p03c_run: FAIL (.*) \(\d', runner)
    out['wheel_publishers'] = (
        1 if 'PASS exactly one publisher on /diff_drive_controller/cmd_vel'
        in runner else None)
    return out


def markdown(rows):
    """Render the rows as the report table."""
    head = ('| run | seed | episode_id | requested | region (frozen) | '
            'target x, y | spawn xy / dz err | result | reason | '
            'sim s | wall s | approach x / y | home err m | bypass | '
            'checks |')
    lines = [head, '|' + '|'.join(['---'] * (head.count('|') - 1)) + '|']
    for r in rows:
        if 'error' in r:
            lines.append(f"| {r['run']} | {r['error']} |")
            continue
        spawn = ('--' if r.get('spawn_ok') is None else
                 f"{r['spawn_max_xy_m'] * 1000:.3f} mm / "
                 f"{r['spawn_max_abs_dz_m'] * 1000:.3f} mm")
        approach = ('--' if 'approach_x' not in r else
                    f"{r['approach_x']:.4f} / {r['approach_y']:+.4f}")
        lines.append(
            f"| {r['run']} | {r['seed']} | {r['episode_id']} | "
            f"{r['requested']} | {r['region']} ({r['frozen_region']}) | "
            f"{r['target_xy'][0]:.4f}, {r['target_xy'][1]:+.4f} | {spawn} | "
            f"{r.get('outcome', '--')} | {r.get('failure_reason') or '--'} | "
            f"{r.get('mission_sim_s', '--')} | "
            f"{r.get('mission_wall_s', '--')} | {approach} | "
            f"{r.get('final_home_error_m', '--')} | "
            f"{r.get('bypass_raw_controller', '--')} | "
            f"{r['runner_checks']} |")
    return '\n'.join(lines)


def main():
    """Report every run under ROOT in ORDER, then any others."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root')
    parser.add_argument('--json', default='')
    parser.add_argument('--md', default='')
    args = parser.parse_args()
    names = [n for n in ORDER if os.path.isdir(os.path.join(args.root, n))]
    names += sorted(n for n in os.listdir(args.root)
                    if n not in names
                    and os.path.isdir(os.path.join(args.root, n)))
    rows = [row(os.path.join(args.root, n)) for n in names]
    text = markdown(rows)
    print(text)
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(rows, f, indent=2, sort_keys=True)
            f.write('\n')
    if args.md:
        with open(args.md, 'w') as f:
            f.write(text + '\n')


if __name__ == '__main__':
    main()
