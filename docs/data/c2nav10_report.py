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
"""C2-NAV.10 report: three fresh-simulator corridor-aligned-waypoint runs.

Reads the committed .navbench/results/c2n10_appr_r{1,2,3}.json and
c2n10_appr_r{1,2,3}_stop.csv artifacts and prints the numbers this
experiment's verdict rests on. No simulator, no ROS -- pure offline
report generation over already-committed data, exactly like
c2nav8_report.py before it.

Usage:
  python3 c2nav10_report.py [results_dir]   # default .navbench/results
"""
import csv
import json
import math
import os
import sys

SW_CORNER = (-3.25, 2.15)
DEADLOCK_POSE = (-3.3009, 1.9100)   # C2-NAV.8 r1's frozen pose
WAYPOINT = (-3.40, 1.35)
GOAL = (-3.575, 2.95)


def load(results_dir, tag):
    with open(os.path.join(results_dir, f'{tag}.json')) as f:
        bench = json.load(f)
    rows = list(csv.DictReader(open(os.path.join(results_dir, f'{tag}_stop.csv'))))
    gt = [(float(r['gt_x']), float(r['gt_y']), float(r['t_s']), r['monitor_action'])
          for r in rows if r['gt_x']]
    return bench, gt


def leg(bench, name):
    for leg in bench['legs']:
        if leg['scenario'] == name:
            return leg
    return None


def main():
    results_dir = sys.argv[1] if len(sys.argv) > 1 else '.navbench/results'
    tags = ['c2n10_appr_r1', 'c2n10_appr_r2', 'c2n10_appr_r3']

    print('=' * 78)
    print('C2-NAV.10: corridor-aligned waypoint, 3 fresh-simulator runs')
    print('=' * 78)
    print(f'waypoint {WAYPOINT}  final goal {GOAL}  SW corner {SW_CORNER}  '
          f"C2-NAV.8 r1 deadlock {DEADLOCK_POSE}")

    for tag in tags:
        bench, gt = load(results_dir, tag)
        print()
        print(f'--- {tag} ---')
        cg = leg(bench, 'corridor_gate')
        wp = leg(bench, 'enclosure_entry_waypoint')
        en = leg(bench, 'enclosure_entry')
        print(f"  corridor_gate            : {cg['status']:<10} "
              f"illegal_frac_transit={cg['dwb_illegal_frac_transit']}")
        print(f"  enclosure_entry_waypoint : {wp['status']:<10} end={wp['end_world']} "
              f"err={wp['final_goal_err_m']}m illegal_frac_transit="
              f"{wp['dwb_illegal_frac_transit']} cm={wp.get('cm_action_frac')}")
        print(f"  enclosure_entry          : {en['status']:<10} end={en['end_world']} "
              f"err={en['final_goal_err_m']}m cm={en.get('cm_action_frac')}")
        if en.get('terminal_yaw_travel_rad') is not None:
            print(f"    terminal_yaw_travel_rad={en['terminal_yaw_travel_rad']:.3f}  "
                  f"t_terminal_s={en['t_terminal_s']:.1f}  "
                  f"terminal_frac_of_leg={en['terminal_frac_of_leg']:.3f}")
        else:
            print("    (never reached terminal phase -- froze/aborted in transit)")

        # STOP frame count and SW-corner proximity, whole run (all 3 legs)
        from collections import Counter
        c = Counter(a for (_, _, _, a) in gt)
        n_stop = c.get('STOP', 0)
        d_sw = min(math.hypot(x - SW_CORNER[0], y - SW_CORNER[1]) for (x, y, _, _) in gt)
        d_dl = min(math.hypot(x - DEADLOCK_POSE[0], y - DEADLOCK_POSE[1])
                   for (x, y, _, _) in gt)
        print(f"    whole-run STOP frames: {n_stop} of {len(gt)} "
              f"({100*n_stop/len(gt):.2f}%)")
        print(f"    closest approach to SW corner        : {d_sw*1000:.0f} mm")
        print(f"    closest approach to r1 deadlock pose : {d_dl*1000:.0f} mm")

    print()
    print('=' * 78)
    print('SUMMARY')
    print('=' * 78)
    print(f'{"run":<6}{"waypoint leg":<14}{"final leg":<12}{"whole-run STOP%":>16}'
          f'{"min dist to SW corner":>24}')
    for tag in tags:
        bench, gt = load(results_dir, tag)
        wp = leg(bench, 'enclosure_entry_waypoint')
        en = leg(bench, 'enclosure_entry')
        from collections import Counter
        c = Counter(a for (_, _, _, a) in gt)
        n_stop = c.get('STOP', 0)
        d_sw = min(math.hypot(x - SW_CORNER[0], y - SW_CORNER[1]) for (x, y, _, _) in gt)
        print(f'{tag.split("_")[-1]:<6}{wp["status"]:<14}{en["status"]:<12}'
              f'{100*n_stop/len(gt):>15.2f}%{d_sw*1000:>21.0f} mm')


if __name__ == '__main__':
    sys.exit(main())
