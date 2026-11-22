#!/usr/bin/env python3
"""Pretty-print one nav_bench result file."""
import json
import sys

KEYS = ['scenario', 'status', 'duration_sim_s', 'duration_wall_s', 'rtf',
        'start_world', 'end_world', 'goal_world', 'final_goal_err_m',
        'path_len_m', 'straight_line_m', 'path_efficiency',
        'min_clearance_m', 'med_clearance_m', 'frac_track_inside_inflation',
        'min_scan_range_m',
        'v_cmd_mean', 'v_cmd_med', 'v_cmd_p95', 'v_cmd_max',
        'v_actual_mean', 'v_actual_med', 'v_actual_max',
        'v_wheel_mean', 'v_wheel_max',
        'w_cmd_mean', 'w_cmd_max', 'w_actual_mean',
        'frac_cmd_below_0.05', 'frac_actual_below_0.05',
        'n_stops', 'n_reversals_cmd', 'n_osc_cmd', 'osc_per_sec',
        't_transit_s', 't_terminal_s', 'terminal_frac_of_leg',
        'transit_len_m', 'transit_speed_mean', 'transit_v_med',
        'terminal_yaw_travel_rad', 'terminal_v_med',
        'dwb_illegal_frac_transit', 'dwb_illegal_by_critic_transit',
        'dwb_illegal_frac_terminal', 'dwb_illegal_by_critic_terminal',
        'n_progress_failures', 'n_loop_rate_misses', 'n_stale_cmd_drops',
        'loop_rate_min_hz',
        'hz_cmd_vel_nav', 'hz_cmd_vel_smoothed', 'hz_cmd_vel',
        'hz_diff_drive_controller/cmd_vel',
        'n_plans', 'replans_per_sec', 'plan_len_m_first', 'plan_len_m_last',
        'n_local_plans', 'xtrack_med_m', 'xtrack_max_m',
        'cm_gated_frac', 'cm_action_frac', 'cm_polygon_secs', 'cm_msgs',
        'dwb_cycles', 'dwb_hz', 'dwb_traj_per_cycle', 'dwb_illegal_frac',
        'dwb_worst_cycle_illegal_frac', 'dwb_illegal_by_critic',
        'dwb_best_critic_mean', 'dwb_best_vx_mean', 'dwb_best_vx_zero_frac']


def main(path):
    d = json.load(open(path))
    print(f"=== {d['tag']}  robot_radius={d['robot_radius']} "
          f"inflation={d['inflation_radius']} "
          f"controller_hz={d['controller_frequency']}")
    for leg in d['legs']:
        print(f"\n---------- {leg.get('scenario')} rep{leg.get('rep')} "
              f"[{leg.get('status')}]")
        print(f"    probes: {leg.get('probes')}")
        for k in KEYS[1:]:
            if k in leg and leg[k] not in (None, {}, []):
                print(f"    {k:34} {leg[k]}")
        if leg.get('warnings'):
            print('    warnings:')
            for k, v in leg['warnings'].items():
                print(f'      {v:4}x {k}')
        w = leg.get('worst_crawl')
        if w:
            print('    WORST CRAWL:')
            for k, v in w.items():
                if k in ('costmap_offsets', 'costmap_costs'):
                    continue
                print(f'      {k:32} {v}')
            if 'costmap_costs' in w:
                offs, costs = w['costmap_offsets'], w['costmap_costs']
                line = ' '.join(
                    f"{o:+.2f}:{c}" for o, c in zip(offs, costs)
                    if abs(o * 100 % 10) < 1e-6)
                print(f'      costmap across robot (offset:cost)')
                print(f'        {line}')


if __name__ == '__main__':
    main(sys.argv[1])
