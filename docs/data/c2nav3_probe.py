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
"""C2-NAV.3 controlled micro-probe: one pose, one plan, one costmap, and
a swept command.

/evaluation gives 819 trajectories per cycle, but the (vx, wz) pairs it
samples are whatever the iterator produced, so a table read straight off
it confounds vx with wz -- the best trajectory at each vx has a different
wz. This regenerates trajectories at the SAME captured stall pose, from
the SAME captured costmap and transformed plan, with wz held fixed while
vx is swept, and scores them through the rebuilt critics.

The generator is StandardTrajectoryGenerator::generateTrajectory and
projectVelocity, transcribed from the Nav2 Jazzy source. It is validated
first against every trajectory the capture kept poses for: if the
regenerated poses do not land on DWB's own poses, nothing below it is
worth reading.

It also reproduces DWBLocalPlanner::scoreTrajectory's short-circuit, so a
regenerated trajectory is reported the way DWB would have reported it --
complete, or aborted at critic N with a partial sum.

Usage:
  python3 c2nav3_probe.py <capture>_stall.json [snapshot_index]
"""
import json
import math
import sys

from c2nav3_mapgrid import (Costmap, MapGrid, score_base_obstacle,
                            score_mapgrid_traj, seeds_goal_dist,
                            seeds_path_dist)

# FollowPath, from the C2-NAV.0 baseline parameter file.
SIM_TIME = 1.5
LINEAR_GRANULARITY = 0.05
ANGULAR_GRANULARITY = 0.025
ACC_X, DECEL_X = 3.0, -2.5
ACC_TH, DECEL_TH = 3.2, -3.2
INCLUDE_LAST_POINT = True
MAX_VEL_X, MIN_VEL_X = 0.3, 0.0
MAX_VEL_TH, VX_SAMPLES, VTH_SAMPLES = 1.0, 20, 40

# The critics, in the order the parameter file lists them -- which is the
# order DWB scores them in, and therefore the order the short-circuit
# aborts in.
SCALES = [('RotateToGoal', 32.0), ('Oscillation', 1.0),
          ('BaseObstacle', 8.0), ('GoalAlign', 24.0),
          ('PathAlign', 32.0), ('PathDist', 32.0), ('GoalDist', 24.0)]
FPD = 0.1          # forward_point_distance for both Align critics
RES = 0.05         # local costmap resolution


def mapgrid_scale(scale):
    """MapGridCritic::getScale -- resolution * 0.5 * scale."""
    return RES * 0.5 * scale


def project_velocity(v0, accel, decel, dt, target):
    if v0 < target:
        return min(target, v0 + accel * dt)
    return max(target, v0 + decel * dt)


def time_steps(vx, wz):
    """StandardTrajectoryGenerator::getTimeSteps, discretize_by_time false."""
    proj_lin = abs(vx) * SIM_TIME
    proj_ang = abs(wz) * SIM_TIME
    n = math.ceil(max(proj_lin / LINEAR_GRANULARITY,
                      proj_ang / ANGULAR_GRANULARITY))
    n = int(n)
    if n == 0:
        n = 1
    return [SIM_TIME / n] * n


def generate(start, start_vel, cmd):
    """StandardTrajectoryGenerator::generateTrajectory."""
    x, y, th = start
    vx, vth = start_vel
    cvx, cvth = cmd
    poses = [(x, y, th)]
    for dt in time_steps(cvx, cvth):
        vx = project_velocity(vx, ACC_X, DECEL_X, dt, cvx)
        vth = project_velocity(vth, ACC_TH, DECEL_TH, dt, cvth)
        x = x + vx * math.cos(th) * dt
        y = y + vx * math.sin(th) * dt
        th = th + vth * dt
        poses.append((x, y, th))
    if INCLUDE_LAST_POINT:
        poses.append((x, y, th))
    return poses


def score(cm, pdg, gdg, gag, poses):
    """DWBLocalPlanner::scoreTrajectory, including the short-circuit.

    Returns (status, total, per_critic_raw, aborted_at). RotateToGoal and
    Oscillation are 0.0 at this pose -- RotateToGoal because in_window_ is
    false 1.3 m from a 0.05 m xy_goal_tolerance, Oscillation because
    nothing has been latched; both are read back from /evaluation and
    asserted by the caller rather than assumed here.
    """
    raw = {'RotateToGoal': 0.0, 'Oscillation': 0.0}
    st, v = score_base_obstacle(cm, poses)
    if st != 'OK':
        return ('ILLEGAL', None, raw, f'BaseObstacle: {v}')
    raw['BaseObstacle'] = v
    st, v = score_mapgrid_traj(gag, poses, FPD, False)
    if st != 'OK':
        return ('ILLEGAL', None, raw, f'GoalAlign: {v}')
    raw['GoalAlign'] = v
    st, v = score_mapgrid_traj(pdg, poses, FPD, False)
    if st != 'OK':
        return ('ILLEGAL', None, raw, f'PathAlign: {v}')
    raw['PathAlign'] = v
    st, v = score_mapgrid_traj(pdg, poses, 0.0, True)
    if st != 'OK':
        return ('ILLEGAL', None, raw, f'PathDist: {v}')
    raw['PathDist'] = v
    st, v = score_mapgrid_traj(gdg, poses, 0.0, True)
    if st != 'OK':
        return ('ILLEGAL', None, raw, f'GoalDist: {v}')
    raw['GoalDist'] = v
    total = 0.0
    for name, sc in SCALES:
        eff = mapgrid_scale(sc) if name in ('GoalAlign', 'PathAlign',
                                            'PathDist', 'GoalDist') else sc
        total += raw[name] * eff
    return ('OK', total, raw, None)


def short_circuited_total(raw, best_total):
    """Where DWBLocalPlanner::scoreTrajectory would have stopped, given a
    running best. Returns (n_critics_scored, partial_total, aborted)."""
    total = 0.0
    for i, (name, sc) in enumerate(SCALES, start=1):
        if name not in raw:
            return (i - 1, total, True)
        eff = mapgrid_scale(sc) if name in ('GoalAlign', 'PathAlign',
                                            'PathDist', 'GoalDist') else sc
        total += raw[name] * eff
        if best_total > 0 and total > best_total:
            return (i, total, True)
    return (len(SCALES), total, False)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else \
        '.navbench/results/c2n3_stall.json'
    which = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    d = json.load(open(path))
    s = d['snapshots'][which]

    cm = Costmap(s['costmap'])
    plan = [(p[0], p[1]) for p in s['transformed_plan']['poses']]
    # The start pose must be DWB's, not a TF lookup: the TF lookup happens
    # a fraction of a second later, and at this stall the robot is turning
    # on the spot at ~0.44 rad/s, so a 0.2 s lag is ~0.09 rad of yaw --
    # enough to move the trajectory endpoints by tens of centimetres. Every
    # trajectory in /evaluation starts at the pose DWB actually used, and
    # they all agree, so poses[0] IS that pose. Checked, not assumed.
    starts = {tuple(round(v, 6) for v in pr['poses'][0])
              for pr in s['probes'].values()}
    if len(starts) != 1:
        print(f'start poses disagree across probes: {starts}')
        return 1
    rx, ry, rth = next(iter(starts))

    pd_seeds, _ = seeds_path_dist(cm, plan)
    gd_seeds, _ = seeds_goal_dist(cm, plan)
    pdg = MapGrid(cm)
    pdg.seeds = pd_seeds
    pdg.propagate()
    gdg = MapGrid(cm)
    gdg.seeds = gd_seeds
    gdg.propagate()
    ang = math.atan2(plan[-1][1] - ry, plan[-1][0] - rx)
    ga_plan = list(plan)
    ga_plan[-1] = (plan[-1][0] + FPD * math.cos(ang),
                   plan[-1][1] + FPD * math.sin(ang))
    ga_seeds, _ = seeds_goal_dist(cm, ga_plan)
    gag = MapGrid(cm)
    gag.seeds = ga_seeds
    gag.propagate()

    print(f'=== {path} snapshot {which} ===')
    print(f'robot in costmap frame ({rx:.4f}, {ry:.4f}) yaw {rth:.4f}, '
          f'cell {cm.world_to_map(rx, ry)}, cost '
          f'{cm.cost(*cm.world_to_map(rx, ry))}')
    print(f"distance to goal {s['dist_to_goal_world']:.4f} m, heading error "
          f"{s['heading_error_to_goal_deg']:+.2f} deg")
    print(f"DWB chose vx {s['chosen']['vx']:.4f} wz {s['chosen']['wz']:.4f} "
          f"total {s['chosen']['total']:.2f}")
    best = float(s['chosen']['total'])

    # -- 1. is the generator right? --------------------------------------
    # computeNewVelocity ramps from the CURRENT velocity toward the
    # commanded one under the acceleration limits, so the current velocity
    # is part of the trajectory shape. It is not in /evaluation, so it is
    # recovered here by fitting it to the captured trajectories and the fit
    # residual is reported. A residual near zero means the generator, the
    # start pose and the start velocity are all right; anything else and
    # the sweep below is not evidence.
    def worst_err(sv):
        w = 0.0
        for pr in s['probes'].values():
            vx, wz = pr['evaluated']
            regen = generate((rx, ry, rth), sv, (vx, wz))
            dwbp = pr['poses']
            n = min(len(regen), len(dwbp))
            for i in range(n):
                w = max(w, math.dist(regen[i][:2], dwbp[i][:2]))
        return w

    best_sv, best_err = (0.0, 0.0), worst_err((0.0, 0.0))
    lo_x, hi_x, lo_t, hi_t = -0.31, 0.31, -1.05, 1.05
    for _ in range(6):
        nx = [lo_x + i * (hi_x - lo_x) / 20.0 for i in range(21)]
        nt = [lo_t + i * (hi_t - lo_t) / 20.0 for i in range(21)]
        for a in nx:
            for b in nt:
                e = worst_err((a, b))
                if e < best_err:
                    best_err, best_sv = e, (a, b)
        sx = (hi_x - lo_x) / 20.0
        st_ = (hi_t - lo_t) / 20.0
        lo_x, hi_x = best_sv[0] - sx, best_sv[0] + sx
        lo_t, hi_t = best_sv[1] - st_, best_sv[1] + st_

    print()
    print('=== generator validation: regenerated poses vs DWB\'s own ===')
    print(f'  start pose (DWB\'s own, from /evaluation): '
          f'({rx:.5f}, {ry:.5f}, {rth:.5f})')
    print(f'  start velocity fitted to the captured trajectories: '
          f'vx {best_sv[0]:+.4f}  wz {best_sv[1]:+.4f}')
    print('  {:<14} {:>8} {:>8} {:>7} {:>7} {:>12}'.format(
        'probe', 'vx', 'wz', 'n_dwb', 'n_regen', 'max_err_m'))
    worst = 0.0
    for label, pr in s['probes'].items():
        vx, wz = pr['evaluated']
        regen = generate((rx, ry, rth), best_sv, (vx, wz))
        dwbp = pr['poses']
        n = min(len(regen), len(dwbp))
        err = max((math.dist(regen[i][:2], dwbp[i][:2]) for i in range(n)),
                  default=0.0)
        worst = max(worst, err)
        print('  {:<14} {:>8.4f} {:>8.4f} {:>7} {:>7} {:>12.6f}'.format(
            label, vx, wz, len(dwbp), len(regen), err))
    print(f'  worst pose error over all probes: {worst:.6f} m')
    if worst > 5e-3:
        print('  GENERATOR DOES NOT MATCH -- the sweep below is not valid')
        return 1
    print('  generator reproduces DWB\'s trajectories; the sweep is valid')
    start_vel = best_sv

    # -- 2. the controlled sweep -----------------------------------------
    vx_samples = [MIN_VEL_X + i * (MAX_VEL_X - MIN_VEL_X) / (VX_SAMPLES - 1)
                  for i in range(VX_SAMPLES)]
    for wz_fixed in (0.0, 0.2564, -0.2564, 0.7692, -0.7692):
        print()
        print('=== vx swept over the sampler\'s own 20 values, '
              f'wz held at {wz_fixed:+.4f} ===')
        print('  raw scores; "abort" is the critic DWB short-circuits or '
              'throws on')
        print('  {:>7} {:>7} {:>9} {:>7} {:>7} {:>7} {:>7} {:>7} {:>8} {:>22}'
              .format('vx', 'end_m', 'total', 'BaseOb', 'GoalAl', 'PathAl',
                      'PathDs', 'GoalDs', 'ncrit', 'verdict'))
        for vx in vx_samples:
            poses = generate((rx, ry, rth), start_vel, (vx, wz_fixed))
            end_d = math.dist(poses[-1][:2], (rx, ry))
            st, total, raw, why = score(cm, pdg, gdg, gag, poses)
            n, partial, aborted = short_circuited_total(raw, best)
            if st == 'ILLEGAL':
                verdict = 'ILLEGAL ' + why.split(':')[0]
            elif aborted:
                verdict = f'short-circuit @{n}'
            elif total < best:
                verdict = 'WINS'
            else:
                verdict = 'loses'
            print('  {:>7.4f} {:>7.3f} {:>9} {:>7} {:>7} {:>7} {:>7} {:>7} '
                  '{:>8} {:>22}'.format(
                      vx, end_d,
                      f'{total:.2f}' if total is not None else 'n/a',
                      f"{raw.get('BaseObstacle', float('nan')):.0f}",
                      f"{raw.get('GoalAlign', float('nan')):.0f}",
                      f"{raw.get('PathAlign', float('nan')):.0f}",
                      f"{raw.get('PathDist', float('nan')):.0f}",
                      f"{raw.get('GoalDist', float('nan')):.0f}",
                      n, verdict))

    # -- 3. the arithmetic of the trap -----------------------------------
    print()
    print('=== how much can the MapGrid critics ever be worth here? ===')
    reach = MAX_VEL_X * SIM_TIME
    cells = reach / RES
    print(f'  sim_time {SIM_TIME} s x max_vel_x {MAX_VEL_X} m/s = {reach:.2f} '
          f'm = {cells:.0f} cells: the furthest the LAST pose can be, and')
    print('  aggregation_type is "last", so that bounds every MapGrid critic.')
    for name, sc in SCALES:
        if name in ('GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'):
            print(f'    {name:<13} best case {cells:.0f} cells x '
                  f'{mapgrid_scale(sc):.3f} = {cells * mapgrid_scale(sc):.2f}')
    tot = sum(cells * mapgrid_scale(sc) for name, sc in SCALES
              if name in ('GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'))
    print(f'    upper bound on the total MapGrid reward for moving: {tot:.2f}')
    print('  BaseObstacle costs scale 8.0 x cell cost. That reward is spent')
    print(f'  by a cell cost of {tot / 8.0:.2f}.')
    rcell = cm.world_to_map(rx, ry)
    ring = []
    for dx in range(-3, 4):
        for dy in range(-3, 4):
            a, b = rcell[0] + dx, rcell[1] + dy
            if 0 <= a < cm.nx and 0 <= b < cm.ny and (dx or dy):
                ring.append(cm.cost(a, b))
    print(f'  cheapest non-zero cost within 3 cells (0.15 m) of the robot: '
          f'{min([c for c in ring if c > 0], default=None)}')
    plan_costs = [cm.cost(*cm.world_to_map(px, py)) for px, py in plan
                  if cm.world_to_map(px, py)]
    print(f'  cost along the transformed plan: min {min(plan_costs)} '
          f'max {max(plan_costs)} over {len(plan_costs)} poses; '
          f'{sum(1 for c in plan_costs if c == 0)} of them are cost 0')
    print(f'  so following the plan costs at least '
          f'{min(plan_costs)} x 8.0 = {min(plan_costs) * 8.0:.0f} in '
          f'BaseObstacle, against a standing-still total of {best:.2f}.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
