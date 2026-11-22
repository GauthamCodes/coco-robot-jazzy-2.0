"""Where did each failing leg actually end up, and what did the robot do?"""
import json, os, math
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')
for tag in ('c2n26_slow_r2', 'c2n26_slow_r3', 'c2n26_bslow_r3',
            'c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4',
            'c2n21_bbase_r2', 'c2n21_bbase_r3'):
    p = os.path.join(R, tag + '.json')
    if not os.path.exists(p):
        continue
    d = json.load(open(p))
    print(f'--- {tag} ---')
    for L in d['legs']:
        if L['status'] == 'SUCCEEDED':
            continue
        print(f"  {L['scenario']:<17}{L['status']:<10}"
              f"start={L['start_world']} end={L['end_world']} "
              f"goal={L['goal_world']} err={L['final_goal_err_m']} "
              f"t={L['duration_wall_s']}s plans={L.get('n_plans')} "
              f"prog_fail={L.get('n_progress_failures')} "
              f"vmax={L.get('v_actual_max')}")
