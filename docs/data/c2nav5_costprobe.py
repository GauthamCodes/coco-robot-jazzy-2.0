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
"""C2-NAV.5 cost-field probe: confirm the C2-NAV.4 mechanism on a run
that does NOT stall.

C2-NAV.3's `c2nav3_capture.py` snapshots the cost field when it detects a
STALL -- ten seconds of zero command between 0.5 m and 2.0 m from the
goal. That is exactly the right trigger for diagnosing a stall and
exactly the wrong one for confirming a fix, because a configuration that
works never fires it. Run unmodified against CSF 65 it returns "no
snapshots", which is indistinguishable in the artifact from an
instrument that never subscribed.

So this triggers on GEOMETRY instead: the first `/evaluation` cycle after
the robot crosses each of a fixed ladder of distances to the goal. The
1.20-1.40 m rung is the one that matters -- all three C2-NAV.3/.4
baseline stalls sit inside it, at 1.279, 1.299 and 1.312 m -- so the
candidate's cost field is read at the same place the baseline's was.

Everything downstream is C2-NAV.3's and C2-NAV.4's own code, imported
rather than reimplemented: `Capture` and `snapshot` from
c2nav3_capture.py, `Costmap` from c2nav3_mapgrid.py, `plan_costs` and
`describe` from c2nav4_costfield.py. A number here is comparable to a
number there because it was produced by the same lines.

Changes nothing. Subscribe-only, plus the NavigateToPose goal that makes
the controller run at all.

Usage:
  python3 c2nav5_costprobe.py <out_prefix>
writes  <out_prefix>_cost.json      one snapshot per distance rung
        <out_prefix>_timeline.csv   the whole approach, one row per cycle
"""
import csv
import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy                                               # noqa: E402
from rclpy.executors import MultiThreadedExecutor          # noqa: E402

from c2nav3_capture import (                               # noqa: E402
    GOAL_MAP, GOAL_WORLD, Capture, snapshot)
from c2nav3_mapgrid import Costmap                         # noqa: E402
from c2nav4_costfield import describe, plan_costs          # noqa: E402
from nav2_msgs.action import NavigateToPose                # noqa: E402

# Distance-to-goal rungs, metres, descending. A snapshot is taken on the
# first evaluation cycle at or below each. 1.30 is the rung that carries
# the argument: the baseline's three recorded stalls are 1.279, 1.299 and
# 1.312 m out, so this reads the candidate's field where the baseline's
# was read. The rest bracket it so a single unlucky cycle cannot be the
# whole result.
RUNGS = [2.00, 1.60, 1.40, 1.30, 1.20, 1.00, 0.80, 0.60, 0.40]

# Total wall budget for the approach, seconds. Matches the C2-NAV.3
# capture's, so a run that never gets there is bounded the same way.
BUDGET_S = 150.0


def measure(s):
    """The cost field at one snapshot, in C2-NAV.3's own terms.

    Returns None when the snapshot is missing either grid, which is a
    real outcome and not an error: DWB publishes the transformed plan
    and the costmap on their own timers, and a cycle can land between.
    """
    if 'costmap' not in s or 'transformed_plan' not in s:
        return None
    cm = Costmap(s['costmap'])
    plan = [(p[0], p[1]) for p in s['transformed_plan']['poses']]
    pc = plan_costs(cm, plan)

    out = {'plan': describe(pc), 'plan_costs': pc,
           'n_plan_poses': len(plan),
           'costmap_age_s_vs_eval': s['costmap']['age_s_vs_eval'],
           'tplan_age_s_vs_eval': s['transformed_plan']['age_s_vs_eval'],
           'tplan_frame': s['transformed_plan']['frame_id']}

    # The cell the robot is standing in. C2-NAV.3's finding was that the
    # robot sits in the last cost-0 cell and every plan pose ahead of it
    # is 60-164; both halves of that need the same grid.
    tf = s.get('tf', {}).get('odom_from_base', {})
    if 'x' in tf:
        c = cm.world_to_map(tf['x'], tf['y'])
        out['robot_cell'] = list(c) if c else None
        out['cost_at_robot'] = cm.cost(*c) if c else None

    # "Cost at pinch centre": the plan pose of MAXIMUM cost, which is the
    # tightest point of the corridor the plan threads. Reported with its
    # position so it can be checked against the map rather than trusted.
    if pc:
        idx = max(range(len(pc)), key=lambda i: pc[i])
        out['pinch'] = {'plan_index': idx, 'cost': pc[idx],
                        'xy': [round(plan[idx][0], 4),
                               round(plan[idx][1], 4)]}

    # DWB's own decision at this cycle, unreplayed: what it chose, what
    # BaseObstacle charged it, and what standing still would have cost.
    ch = s['chosen']
    raw = {c[0]: c[1] for c in ch['critics']}
    scaled = {c[0]: c[1] * c[2] for c in ch['critics']}
    out['chosen'] = {'vx': round(ch['vx'], 4), 'wz': round(ch['wz'], 4),
                     'total': round(ch['total'], 3),
                     'n_critics': ch['n_critics'],
                     'BaseObstacle_raw': raw.get('BaseObstacle'),
                     'BaseObstacle_scaled': (
                         None if 'BaseObstacle' not in scaled
                         else round(scaled['BaseObstacle'], 3)),
                     'critics_scaled': {k: round(v, 3)
                                        for k, v in scaled.items()}}
    # The zero-velocity reference, C2-NAV.4's definition exactly: the
    # cheapest LEGAL trajectory that does not translate, over every wz the
    # sampler produced -- not the single (0, 0) sample, which the sampler
    # need not contain. Restricted to trajectories scored to COMPLETION,
    # because a short-circuited partial total is not a score.
    ncrit = max((t['n_critics'] for t in s['all']), default=0)
    zeros = [t for t in s['all']
             if abs(t['vx']) < 1e-9 and t['total'] >= 0.0
             and t['n_critics'] == ncrit]
    z = min(zeros, key=lambda t: t['total'], default=None)
    out['zero'] = (None if z is None
                   else {'vx': z['vx'], 'wz': z['wz'],
                         'total': z['total'], 'n_critics': z['n_critics']})
    # The best FORWARD trajectory scored to completion, and its margin
    # against standing still. A negative margin is the stall.
    fwd = [t for t in s['all']
           if t['vx'] > 1e-9 and t['total'] >= 0.0
           and t['n_critics'] == ncrit]
    f = min(fwd, key=lambda t: t['total'], default=None)
    out['best_forward'] = (None if f is None
                           else {'vx': f['vx'], 'wz': f['wz'],
                                 'total': f['total'],
                                 'n_critics': f['n_critics']})
    if f is not None and z is not None:
        out['forward_minus_zero'] = round(f['total'] - z['total'], 3)
    out['n_traj'] = s['n_traj']
    out['n_legal'] = s['n_legal']
    out['full_score_n_critics'] = ncrit
    out['dist_to_goal'] = round(s.get('dist_to_goal_world', float('nan')), 4)
    out['gt'] = s.get('gt')
    return out


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else '/tmp/c2nav5'
    rclpy.init()
    node = Capture()
    node.t0 = time.time()
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    print('[costprobe] waiting for navigate_to_pose')
    if not node.ac.wait_for_server(timeout_sec=40.0):
        print('[costprobe] NO ACTION SERVER')
        return 1
    g = NavigateToPose.Goal()
    g.pose.header.frame_id = 'map'
    g.pose.header.stamp = node.get_clock().now().to_msg()
    g.pose.pose.position.x = GOAL_MAP[0]
    g.pose.pose.position.y = GOAL_MAP[1]
    g.pose.pose.orientation.w = 1.0
    print(f'[costprobe] goal world {GOAL_WORLD} = map {GOAL_MAP}')
    node.t0 = time.time()
    fut = node.ac.send_goal_async(g)
    dl = time.time() + 20.0
    while not fut.done() and time.time() < dl:
        time.sleep(0.05)
    if not fut.done():
        print('[costprobe] NO ACK')
        return 1
    handle = fut.result()
    print(f'[costprobe] accepted={handle.accepted}')
    if not handle.accepted:
        return 1

    # Prove the instrument can SEE something before believing a quiet
    # result. This probe's whole job is to report cost fields on a run
    # that does not stall; "no snapshots" and "never subscribed" have to
    # be distinguishable, and only a positive control does that.
    t_end = time.time() + 40.0
    while time.time() < t_end:
        with node.lock:
            n = len(node.eval_msgs)
            has_cm = node.costmap is not None
            has_tp = node.tplan is not None
        if n and has_cm and has_tp:
            print(f'[costprobe] alive: {n} /evaluation msgs, costmap yes, '
                  'transformed plan yes')
            break
        time.sleep(0.5)
    else:
        with node.lock:
            print('[costprobe] INSTRUMENT NOT ALIVE: '
                  f'eval={len(node.eval_msgs)} '
                  f'costmap={node.costmap is not None} '
                  f'tplan={node.tplan is not None}')
        return 1

    taken = {}
    d_min = float('inf')
    deadline = time.time() + BUDGET_S
    while time.time() < deadline and len(taken) < len(RUNGS):
        time.sleep(0.2)
        with node.lock:
            gt = node.gt
            recent = node.eval_msgs[-1] if node.eval_msgs else None
        if gt is None or recent is None:
            continue
        dist = math.dist(gt[:2], GOAL_WORLD)
        d_min = min(d_min, dist)
        for r in RUNGS:
            if r in taken or dist > r:
                continue
            t_eval, m = recent
            if not (0 <= m.best_index < len(m.twists)):
                continue
            s = snapshot(node, m, t_eval)
            mm = measure(s)
            if mm is None:
                continue                    # try again on the next cycle
            mm['rung_m'] = r
            mm['t_s'] = round(time.time() - node.t0, 2)
            taken[r] = mm
            pl = mm['plan']
            print(f'[costprobe] rung {r:.2f} m at t={mm["t_s"]:.1f}s '
                  f'd={mm["dist_to_goal"]:.3f} plan cost '
                  f'min={pl.get("min")} med={pl.get("median")} '
                  f'max={pl.get("max")} n_zero={pl.get("n_zero")} '
                  f'| chosen vx={mm["chosen"]["vx"]:.4f} '
                  f'BaseOb={mm["chosen"]["BaseObstacle_scaled"]}',
                  flush=True)

    with node.lock:
        rows = list(node.rows)
    print(f'[costprobe] {len(taken)} rungs, {len(rows)} timeline rows, '
          f'closest approach {d_min:.3f} m')

    with open(f'{prefix}_cost.json', 'w') as f:
        json.dump({'rungs': RUNGS,
                   'budget_s': BUDGET_S,
                   'goal_world': list(GOAL_WORLD),
                   'closest_approach_m': round(d_min, 4),
                   'snapshots': [taken[r] for r in RUNGS if r in taken]},
                  f, indent=1)
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(f'{prefix}_timeline.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print(f'[costprobe] wrote {prefix}_cost.json and {prefix}_timeline.csv')

    try:
        handle.cancel_goal_async()
    except Exception:                                      # noqa: BLE001
        pass
    time.sleep(2.0)
    ex.shutdown()
    return 0 if taken else 2


if __name__ == '__main__':
    sys.exit(main())
