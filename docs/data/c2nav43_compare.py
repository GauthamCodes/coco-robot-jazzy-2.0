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
"""C2-NAV.43 Part K -- baseline_lidar_only vs depth_fusion, offline.

No metric is defined here. Per leg and per arm it applies, unchanged:
  c2nav39_tour_report.leg_detail   status, duration, goal / yaw error, true
                                   geometric clearance, PolygonStop holds,
                                   deadlocks (C2-NAV.40)
  c2nav41_topology.arm             arm tables, monitor authority, STOP breach,
                                   raw-controller bypass rows (C2-NAV.41)
and reads, per run, what the runner and the recorder wrote:
  params_live.txt / topology_live.txt / perception_live.txt   OK / MISMATCH
  manifest.json                    git sha, params sha, nav.launch.py args, exit
  <run>.json legs                  n_stale_cmd_drops, n_loop_rate_misses
  nav.log                          controller / costmap missed-rate warnings
  perception_record/record_summary.json   phantom local-costmap marks, ramp
                                   marks, Nav2 container and depth-cloud CPU

  python3 -P docs/data/c2nav43_compare.py \
      --baseline ~/coco_nav_runs/baseline_lidar_only/baseline_lidar_only_r0[1-3] \
      --fusion ~/coco_nav_runs/depth_fusion/depth_fusion_r0[1-3] [--json out.json]

Three tours per arm, one route set, legs that are not independent: counts and
medians are printed; no significance is computed.
"""

import argparse
import importlib.util
import json
import os
import re
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def _load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, f'{name}.py'))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


REPORT = _load('c2nav39_tour_report')
TOPO = _load('c2nav41_topology')

LOG_PATTERNS = {
    'controller_missed_rate': re.compile(r'Control loop missed its desired rate'),
    'costmap_missed_rate': re.compile(r'Map update loop missed its desired rate'),
    'sensor_not_current': re.compile(r'observation buffer has not been updated|not current|out of date',
                                     re.IGNORECASE),
    'failed_to_make_progress': re.compile(r'Failed to make progress'),
}


def _count_lines(path, patterns):
    counts = {k: 0 for k in patterns}
    if not os.path.exists(path):
        return None
    with open(path, errors='replace') as f:
        for line in f:
            for k, pat in patterns.items():
                if pat.search(line):
                    counts[k] += 1
    return counts


def _verdicts(path):
    if not os.path.exists(path):
        return None
    ok = bad = 0
    with open(path) as f:
        for line in f:
            if line.startswith('OK'):
                ok += 1
            elif line.startswith('MISMATCH'):
                bad += 1
    return {'ok': ok, 'mismatch': bad}


def run_record(run_dir):
    run_dir = os.path.expanduser(run_dir)
    tag = os.path.basename(os.path.normpath(run_dir))
    with open(os.path.join(run_dir, f'{tag}.json')) as f:
        legs = json.load(f)['legs']
    details = []
    for leg in legs:
        trace = os.path.join(run_dir, f'{tag}_traces', f"{leg['scenario']}_rep{leg.get('rep', 0)}.csv")
        rows = REPORT.load_trace_rows(trace) if os.path.exists(trace) else None
        details.append(REPORT.leg_detail(leg, rows, details[-1] if details else None))
    manifest = json.load(open(os.path.join(run_dir, 'manifest.json')))
    rec_path = os.path.join(run_dir, 'perception_record', 'record_summary.json')
    single = TOPO.arm([run_dir], 'B', expect='B')
    return {
        'run': tag,
        'git_sha': manifest.get('git_sha', '')[:7],
        'params_sha256': manifest.get('params_sha256', '')[:8],
        'nav_launch_args': manifest.get('nav_launch_args'),
        'exit_code': manifest.get('exit_code'),
        'summary': single['summary'],
        'authority': single['authority'],
        'stop_breach': single['stop_breach'],
        'bypass_source': single['bypass_source'],
        'legs': [{k: d[k] for k in ('scenario', 'status', 'duration_sim_s', 'final_goal_err_m',
                                    'final_yaw_err_rad', 'true_min_clearance_m', 'polygon_stop_s',
                                    'stop_activations', 'stop_deadlock')} for d in details],
        'timeouts': sum(1 for d in details if d['status'] == 'TIMEOUT'),
        'aborted': sum(1 for d in details if d['status'] not in ('SUCCEEDED', 'TIMEOUT')),
        'stale_cmd_drops': sum(leg.get('n_stale_cmd_drops') or 0 for leg in legs),
        'loop_rate_misses': sum(leg.get('n_loop_rate_misses') or 0 for leg in legs),
        'nav_log': _count_lines(os.path.join(run_dir, 'nav.log'), LOG_PATTERNS),
        'params_live': _verdicts(os.path.join(run_dir, 'params_live.txt')),
        'topology_live': _verdicts(os.path.join(run_dir, 'topology_live.txt')),
        'perception_live': _verdicts(os.path.join(run_dir, 'perception_live.txt')),
        'record': json.load(open(rec_path)) if os.path.exists(rec_path) else None,
    }


def _med(values):
    values = [v for v in values if v is not None]
    return round(statistics.median(values), 3) if values else None


def arm_record(label, run_dirs):
    runs = [run_record(r) for r in run_dirs]
    arm = TOPO.arm(run_dirs, label, expect='B')
    recs = [r['record'] for r in runs if r['record']]
    return {
        'label': label,
        'runs': runs,
        'table': arm['table'],
        'summary': arm['summary'],
        'authority': arm['authority'],
        'stop_breach': arm['stop_breach'],
        'bypass_source': arm['bypass_source'],
        'timeouts': sum(r['timeouts'] for r in runs),
        'aborted': sum(r['aborted'] for r in runs),
        'stale_cmd_drops': sum(r['stale_cmd_drops'] for r in runs),
        'loop_rate_misses': sum(r['loop_rate_misses'] for r in runs),
        'nav_log': {k: sum((r['nav_log'] or {}).get(k, 0) for r in runs) for k in LOG_PATTERNS},
        'readback_mismatches': sum(((r['params_live'] or {}).get('mismatch', 0)
                                    + (r['topology_live'] or {}).get('mismatch', 0)
                                    + (r['perception_live'] or {}).get('mismatch', 0)) for r in runs),
        'phantom_cells_total': sum(x.get('phantom_cells_total', 0) for x in recs),
        'grids_with_phantom': sum(x.get('grids_with_phantom', 0) for x in recs),
        'grids': sum(x.get('grids', 0) for x in recs),
        'phantom_max_distance_m': max((x.get('phantom_max_distance_m', 0.0) for x in recs), default=None),
        # Only runs recorded after the field-of-view split (02cd725) carry it.
        'phantom_fov_split': {
            'runs': sum(1 for x in recs if 'phantom_in_camera_fov_cells_total' in x),
            **{k: sum(x.get(k, 0) for x in recs if 'phantom_in_camera_fov_cells_total' in x)
               for k in ('phantom_in_camera_fov_cells_total', 'phantom_in_camera_fov_grids',
                         'phantom_in_lidar_fov_cells_total', 'phantom_in_lidar_fov_grids',
                         'phantom_in_neither_fov_cells_total', 'phantom_in_neither_fov_grids')},
            'grids': sum(x.get('grids', 0) for x in recs if 'phantom_in_camera_fov_cells_total' in x),
        },
        'ramp_cells_max': max((x.get('ramp_cells_max', 0) for x in recs), default=None),
        'grids_with_ramp_marks': sum(x.get('grids_with_ramp_marks', 0) for x in recs),
        'nav2_cpu_cores_median': _med([(x.get('cpu_nav2_container') or {}).get('mean_cores') for x in recs]),
        'depth_cloud_cpu_cores_median': _med([(x.get('cpu_depth_cloud') or {}).get('mean_cores')
                                              for x in recs]),
        'entry_abs_yaw_err_rad': [abs(leg['final_yaw_err_rad']) for r in runs for leg in r['legs']
                                  if leg['scenario'] == 'enclosure_entry'
                                  and leg['final_yaw_err_rad'] is not None],
    }


def _frac(pair):
    return f'{pair[0]}/{pair[1]}'


def render(arms):
    out = ['### Per run', '',
           '| arm | run | legs | ordinary | entry | exit | tour sim s | min true clear m | STOP n / s | '
           'deadlocks | timeouts | bypass rows | exceeded | stale drops | loop misses | '
           'phantom cells (grids) | ramp cells max | Nav2 cores | depth cores | readback |',
           '|' + '---|' * 20]
    for a in arms:
        for r in a['runs']:
            s = r['summary']
            rec = r['record'] or {}
            rb = [r['params_live'], r['topology_live'], r['perception_live']]
            rb_txt = '/'.join(f"{x['ok']}ok{('+' + str(x['mismatch']) + 'MM') if x['mismatch'] else ''}"
                              if x else '--' for x in rb)
            out.append(
                f"| {a['label']} | `{r['run']}` | {_frac(s['legs'])} | {_frac(s['ordinary'])} | "
                f"{_frac(s['enclosure_entry'])} | {_frac(s['enclosure_exit'])} | {s['tour_sim_s'][0]} | "
                f"{s['true_min_clearance_m']} | {s['stop_activations']} / {s['polygon_stop_s']} | "
                f"{s['deadlocks'] or '--'} | {r['timeouts']} | {r['bypass_source']['rows']} | "
                f"{r['authority']['exceeded']}/{r['authority']['samples']} | {r['stale_cmd_drops']} | "
                f"{r['loop_rate_misses']} | {rec.get('phantom_cells_total', '--')} "
                f"({rec.get('grids_with_phantom', '--')}/{rec.get('grids', '--')}) | "
                f"{rec.get('ramp_cells_max', '--')} | "
                f"{(rec.get('cpu_nav2_container') or {}).get('mean_cores', '--')} | "
                f"{(rec.get('cpu_depth_cloud') or {}).get('mean_cores', '--')} | {rb_txt} |")
    out += ['', '### Per arm', '',
            '| arm | legs | ordinary | entry | exit | median tour sim s | min true clear m | STOP n / s | '
            'deadlocks | timeouts | goal err med (succ) m | abs yaw err med (succ) rad | entry abs yaw err rad | '
            'bypass rows | exceeded | STOP rows driven | phantom cells (grids) | Nav2 cores med | '
            'depth cores med | controller / costmap missed-rate lines |',
            '|' + '---|' * 20]
    for a in arms:
        s = a['summary']
        out.append(
            f"| {a['label']} | {_frac(s['legs'])} | {_frac(s['ordinary'])} | {_frac(s['enclosure_entry'])} | "
            f"{_frac(s['enclosure_exit'])} | {s['median_tour_sim_s']} | {s['true_min_clearance_m']} | "
            f"{s['stop_activations']} / {s['polygon_stop_s']} | {s['deadlocks'] or '--'} | {a['timeouts']} | "
            f"{s['median_goal_err_succeeded_m']} | {s['median_abs_yaw_err_succeeded_rad']} | "
            f"{[round(v, 3) for v in a['entry_abs_yaw_err_rad']]} | {a['bypass_source']['rows']} | "
            f"{a['authority']['exceeded']}/{a['authority']['samples']} | "
            f"{a['stop_breach']['stop_rows_wheels_driven']}/{a['stop_breach']['stop_rows']} | "
            f"{a['phantom_cells_total']} ({a['grids_with_phantom']}/{a['grids']}) | "
            f"{a['nav2_cpu_cores_median']} | {a['depth_cloud_cpu_cores_median']} | "
            f"{a['nav_log']['controller_missed_rate']} / {a['nav_log']['costmap_missed_rate']} |")
    out += ['', '### Phantom local-costmap cells by sensor view (runs recorded after 02cd725)', '',
            '| arm | runs | grids | in camera view: cells (grids) | in LiDAR view: cells (grids) | '
            'in neither: cells (grids) |', '|---|---|---|---|---|---|']
    for a in arms:
        p = a['phantom_fov_split']
        out.append(f"| {a['label']} | {p['runs']} | {p['grids']} | "
                   f"{p['phantom_in_camera_fov_cells_total']} ({p['phantom_in_camera_fov_grids']}) | "
                   f"{p['phantom_in_lidar_fov_cells_total']} ({p['phantom_in_lidar_fov_grids']}) | "
                   f"{p['phantom_in_neither_fov_cells_total']} ({p['phantom_in_neither_fov_grids']}) |")
    out += ['', '### Per leg', '',
            '| leg | arm | succeeded | median s | goal err m | abs yaw err rad | true clear m | '
            'PolygonStop s (legs) | DWB zero-vx | cmd<0.05 |',
            '|---|---|---|---|---|---|---|---|---|---|']
    for name in REPORT.TOUR_ORDER:
        for a in arms:
            r = a['table'].get(name)
            if not r:
                continue
            stop = '--' if r['polygon_stop_s'] is None else f"{r['polygon_stop_s']:.2f} ({r['polygon_stop_legs']})"
            out.append(f"| `{name}` | {a['label']} | {r['succeeded']}/{r['attempts']} | {r['median_s']} | "
                       f"{r['median_goal_err_m']} | {r['median_abs_yaw_err_rad']} | "
                       f"{REPORT._fmt(r['true_min_clearance_m'], '.4f')} | {stop} | {r['median_dwb_zero_vx']} | "
                       f"{r['median_cmd_below_0_05']} |")
    return '\n'.join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(prog='c2nav43_compare.py')
    ap.add_argument('--baseline', nargs='+', required=True)
    ap.add_argument('--fusion', nargs='+', required=True)
    ap.add_argument('--json')
    args = ap.parse_args(argv)
    arms = [arm_record('baseline_lidar_only', args.baseline), arm_record('depth_fusion', args.fusion)]
    print(render(arms))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(arms, f, indent=1)
        print(f'\nwrote {args.json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
