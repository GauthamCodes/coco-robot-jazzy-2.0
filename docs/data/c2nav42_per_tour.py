"""Per-tour Task 7 table from c2nav41_topology.arm() (unchanged definitions) + nav_bench's worst_crawl."""
import importlib.util
import io
import json
import os
import sys
from contextlib import redirect_stdout

WT = "/home/gautham/ros2_ws(personal)/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis"
spec = importlib.util.spec_from_file_location('top', os.path.join(WT, 'docs/data/c2nav41_topology.py'))
top = importlib.util.module_from_spec(spec)
spec.loader.exec_module(top)

R = '/home/gautham/coco_nav_runs'
runs = [(f'{R}/baseline_topology_b/baseline_topology_b_r{n:02d}', 'B') for n in (1, 2, 4, 5, 6, 7, 8)]
runs += [(f'{R}/baseline/baseline_r{n:02d}', 'A') for n in (1, 2, 3, 4)]
rows = []
for run, topo in runs:
    tag = os.path.basename(run)
    legacy = topo == 'A' and tag != 'baseline_r04'
    with redirect_stdout(io.StringIO()):
        a = top.arm([run], topo, expect=topo, allow_legacy=legacy)
    s = a['summary']
    legs = json.load(open(os.path.join(run, f'{tag}.json')))['legs']
    crawl = max((leg.get('worst_crawl') or {}).get('crawl_len_s') or 0.0 for leg in legs)
    stale = sum(leg.get('n_stale_cmd_drops') or 0 for leg in legs)
    man = json.load(open(os.path.join(run, 'manifest.json')))
    rows.append({
        'run': tag, 'git': man.get('git_sha', '')[:7], 'dirty': man.get('git_dirty_paths'),
        'legs': f"{s['legs'][0]}/{s['legs'][1]}", 'ordinary': f"{s['ordinary'][0]}/{s['ordinary'][1]}",
        'entry': f"{s['enclosure_entry'][0]}/{s['enclosure_entry'][1]}",
        'exit': f"{s['enclosure_exit'][0]}/{s['enclosure_exit'][1]}",
        'tour_sim_s': s['tour_sim_s'][0], 'true_clear_m': s['true_min_clearance_m'],
        'stop_n': s['stop_activations'], 'stop_s': s['polygon_stop_s'], 'deadlocks': s['deadlocks'],
        'goal_err_med_m': s['median_goal_err_succeeded_m'],
        'yaw_err_med_rad': s['median_abs_yaw_err_succeeded_rad'],
        'longest_crawl_s': round(crawl, 2),
        'exceeded': f"{a['authority']['exceeded']}/{a['authority']['samples']}",
        'bypass_rows': a['bypass_source']['rows'],
        'bypass_eq_raw': a['bypass_source']['wheel_matches_raw_controller'],
        'stop_rows_driven': f"{a['stop_breach']['stop_rows_wheels_driven']}/{a['stop_breach']['stop_rows']}",
        'stale_drops': stale, 'live': a['runs'][0]['live'],
    })
hdr = list(rows[0].keys())
print('| ' + ' | '.join(hdr) + ' |')
print('|' + '---|' * len(hdr))
for r in rows:
    print('| ' + ' | '.join(str(r[h]) for h in hdr) + ' |')
json.dump(rows, open(sys.argv[1], 'w'), indent=1)
