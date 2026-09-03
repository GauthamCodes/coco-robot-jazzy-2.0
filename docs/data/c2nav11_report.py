"""C2-NAV.11 report: closest-approach and continuity metrics, computed
from committed trace CSVs and result JSONs. No ROS needed.

Usage: python3 -P docs/data/c2nav11_report.py .navbench/results <tag1> [<tag2> ...]
"""
import csv
import json
import math
import sys

SW_CORNER = (-3.25, 2.15)          # box_obstacle_1 SW corner, c2nav9_corridor.py
DEADLOCK_POSE = (-3.3001, 1.9095)  # C2-NAV.8 r1's frozen GT pose, same source
WAYPOINT = (-3.40, 1.35)
FINAL_GOAL = (-3.575, 2.95)


def closest(rows, target):
    best = None
    for r in rows:
        if r['x'] == '':
            continue
        d = math.dist((float(r['x']), float(r['y'])), target)
        if best is None or d < best:
            best = d
    return best


def report(outdir, tag):
    with open(f'{outdir}/{tag}.json') as f:
        run = json.load(f)
    with open(f'{outdir}/{tag}_stop.json') as f:
        stop = json.load(f)

    leg = next(leg for leg in run['legs'] if leg['scenario'] == 'enclosure_entry')
    with open(f'{outdir}/{tag}_traces/enclosure_entry_rep0.csv') as f:
        rows = list(csv.DictReader(f))

    d_sw = closest(rows, SW_CORNER)
    d_deadlock = closest(rows, DEADLOCK_POSE)
    stop_leg = stop['legs'].get('enclosure_entry', {})

    print(f'=== {tag}: enclosure_entry (NavigateThroughPoses, '
          f'via {leg.get("through_poses_world")}) ===')
    print(f'  status                          : {leg["status"]}')
    print(f'  duration_sim_s                   : {leg["duration_sim_s"]}')
    print(f'  final_goal_err_m                 : {leg.get("final_goal_err_m")}')
    print(f'  early_plan_n_poses                : {leg.get("early_plan_n_poses")}')
    print(f'  early_plan_ts_offset_from_t0_s     : '
          f'{leg.get("early_plan_ts_offset_from_t0_s")}')
    print(f'  early_plan_endpoint_to_final_goal_m: '
          f'{leg.get("early_plan_endpoint_to_final_goal_m")}')
    print(f'  closest approach to SW corner {SW_CORNER}: '
          f'{d_sw:.3f} m' if d_sw is not None else '  closest to SW corner: n/a')
    print(f'  closest approach to r1 deadlock pose {DEADLOCK_POSE}: '
          f'{d_deadlock:.3f} m' if d_deadlock is not None else
          '  closest to deadlock pose: n/a')
    print(f'  whole-run PolygonStop rows       : '
          f'{stop_leg.get("n_stop_rows")} / {stop_leg.get("n_rows")} '
          f'({100 * stop_leg.get("stop_frac", 0):.2f}%)')
    print(f'  whole-run true min base clearance : '
          f'{stop_leg.get("d_min_base_m_min")} m '
          f'(live lidar-derived, base_footprint origin -- the collision '
          f'monitor\'s own metric, NOT nav_bench\'s quantized min_clearance_m)')
    print(f'  nav_bench min_clearance_m (quantized, unreliable per C2-NAV.7): '
          f'{leg.get("min_clearance_m")} m')
    print(f'  dwb_best_critic_mean.BaseObstacle : '
          f'{leg["dwb_best_critic_mean"].get("BaseObstacle")}')
    print(f'  dwb_illegal_frac_transit          : {leg.get("dwb_illegal_frac_transit")}')
    print(f'  dwb_best_vx_zero_frac             : {leg.get("dwb_best_vx_zero_frac")}')
    print(f'  terminal_yaw_travel_rad           : {leg.get("terminal_yaw_travel_rad")}')
    print(f'  terminal_frac_of_leg              : {leg.get("terminal_frac_of_leg")}')
    print(f'  t_terminal_s                      : {leg.get("t_terminal_s")}')
    print()
    return {
        'tag': tag, 'status': leg['status'],
        'd_sw_corner_m': d_sw, 'd_deadlock_pose_m': d_deadlock,
        'stop_frac': stop_leg.get('stop_frac'),
        'n_stop_rows': stop_leg.get('n_stop_rows'),
        'n_rows': stop_leg.get('n_rows'),
        'true_min_clearance_m': stop_leg.get('d_min_base_m_min'),
    }


if __name__ == '__main__':
    outdir = sys.argv[1]
    tags = sys.argv[2:]
    rows = [report(outdir, t) for t in tags]
    print('=== summary table ===')
    print(f'{"run":<16}{"status":<12}{"SW corner m":<14}'
          f'{"deadlock m":<13}{"STOP frac":<12}{"true min clr m"}')
    for r in rows:
        print(f'{r["tag"]:<16}{r["status"]:<12}'
              f'{r["d_sw_corner_m"]:<14.3f}{r["d_deadlock_pose_m"]:<13.3f}'
              f'{100*r["stop_frac"]:<11.2f}%{r["true_min_clearance_m"]}')
