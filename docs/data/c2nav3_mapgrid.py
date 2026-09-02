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
"""C2-NAV.3 analysis: rebuild the four MapGrid critics and check the
rebuild against DWB's own numbers.

A claim about what GoalDist "means" is worth nothing unless the rebuilt
value reproduces the raw_score DWB published for the same trajectory. This
reimplements, from the Nav2 Jazzy source (dwb_critics 1.3.11, verified
byte-identical to the jazzy branch tip), exactly:

  MapGridCritic::reset / propogateManhattanDistances   map_grid.cpp
  GoalDistCritic::prepare / getLastPoseOnCostmap       goal_dist.cpp
  PathDistCritic::prepare                              path_dist.cpp
  PathAlignCritic::prepare / scorePose / getScale      path_align.cpp
  GoalAlignCritic::prepare / scorePose                 goal_align.cpp
  BaseObstacleCritic::scoreTrajectory / isValidCost    base_obstacle.cpp
  nav_2d_utils::adjustPlanResolution                   path_ops.cpp
  costmap_queue::CostmapQueue                          costmap_queue.cpp

and scores every trajectory the capture kept poses for, then prints the
rebuilt raw score beside the published one.

Three facts from that source that the rebuild depends on, and that are
easy to get wrong:

1. The propagation is NOT a path length and does NOT avoid obstacles.
   MapGridQueue::validCellToQueue returns true unconditionally, and
   getNextCell enqueues all four neighbours whatever their cost. The
   value written is CellData::absolute_difference(src_x, x) +
   absolute_difference(src_y, y) -- the MANHATTAN (L1) distance in CELLS
   from the seed that claimed the cell. With 4-connectivity and no
   blocking, that is exactly min-over-seeds L1, so the rebuild uses a
   multi-source BFS and asserts the two agree.

2. aggregation_type defaults to "last". The score of a trajectory is the
   value at its LAST pose, not a sum along it. sim_time 1.5 s at
   max_vel_x 0.3 caps the last pose at 0.45 m = 9 cells from the robot,
   which bounds how much any of these critics can ever differ between
   standing still and driving.

3. PathAlign and GoalAlign are the same two grids read at a point
   forward_point_distance (0.1 m) ahead of the last pose's heading, and
   they set stop_on_failure_ = false, so unlike PathDist/GoalDist they
   never throw on an obstacle cell.

Usage:
  python3 c2nav3_mapgrid.py <capture>_stall.json [snapshot_index]
"""
import json
import math
import sys
from collections import deque

# nav2_costmap_2d/cost_values.hpp
NO_INFORMATION = 255
LETHAL_OBSTACLE = 254
INSCRIBED_INFLATED_OBSTACLE = 253

CRITIC_ORDER = ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign',
                'PathAlign', 'PathDist', 'GoalDist']


class Costmap:
    """The subset of nav2_costmap_2d::Costmap2D the critics use."""

    def __init__(self, meta):
        self.res = meta['resolution']
        self.nx = meta['size_x']
        self.ny = meta['size_y']
        self.ox, self.oy = meta['origin']
        self.data = meta['data']

    def world_to_map(self, wx, wy):
        """Costmap2D::worldToMap -- truncation, and a hard reject below
        the origin. Returns None when the point is off the grid."""
        if wx < self.ox or wy < self.oy:
            return None
        mx = int((wx - self.ox) / self.res)
        my = int((wy - self.oy) / self.res)
        if mx < self.nx and my < self.ny:
            return (mx, my)
        return None

    def index(self, mx, my):
        return my * self.nx + mx

    def cost(self, mx, my):
        return self.data[self.index(mx, my)]


def adjust_plan_resolution(poses, resolution):
    """nav_2d_utils::adjustPlanResolution, faithfully, including its
    integer step arithmetic."""
    if not poses:
        return []
    out = [poses[0]]
    last = poses[0]
    min_sq = resolution * resolution * 4.0
    for i in range(1, len(poses)):
        loop = poses[i]
        sq = (loop[0] - last[0]) ** 2 + (loop[1] - last[1]) ** 2
        if sq > min_sq:
            diff = math.sqrt(sq) - math.sqrt(min_sq)
            steps = int(diff / resolution) - 1
            if steps != 0:
                dx = (loop[0] - last[0]) / float(steps)
                dy = (loop[1] - last[1]) / float(steps)
                for j in range(1, steps):
                    out.append((last[0] + j * dx, last[1] + j * dy))
        out.append(loop)
        last = (loop[0], loop[1])
    return out


class MapGrid:
    """MapGridCritic: reset(), the seed set, and the L1 propagation."""

    def __init__(self, cm):
        self.cm = cm
        n = cm.nx * cm.ny
        self.obstacle_score = float(n)          # == cell_values_.size()
        self.unreachable_score = float(n) + 1.0
        self.vals = [self.unreachable_score] * n
        self.seeds = []

    def propagate(self):
        """propogateManhattanDistances via CostmapQueue.

        The queue is ordered by L1 distance and never blocks on cost, so a
        plain multi-source BFS over 4-neighbours gives the same field.
        Verified against the direct min-over-seeds L1 in check_l1().
        """
        cm = self.cm
        dq = deque()
        for (mx, my) in self.seeds:
            i = cm.index(mx, my)
            self.vals[i] = 0.0
            dq.append((mx, my, 0))
        seen = [False] * (cm.nx * cm.ny)
        for (mx, my) in self.seeds:
            seen[cm.index(mx, my)] = True
        while dq:
            mx, my, d = dq.popleft()
            for ax, ay in ((mx - 1, my), (mx + 1, my), (mx, my - 1),
                           (mx, my + 1)):
                if 0 <= ax < cm.nx and 0 <= ay < cm.ny:
                    i = cm.index(ax, ay)
                    if not seen[i]:
                        seen[i] = True
                        self.vals[i] = float(d + 1)
                        dq.append((ax, ay, d + 1))

    def check_l1(self):
        """min-over-seeds L1, computed directly, must equal the flood."""
        cm = self.cm
        bad = 0
        step = max(1, (cm.nx * cm.ny) // 400)
        for i in range(0, cm.nx * cm.ny, step):
            cx, cy = i % cm.nx, i // cm.nx
            m = min(abs(sx - cx) + abs(sy - cy) for sx, sy in self.seeds)
            if abs(self.vals[i] - m) > 1e-9:
                bad += 1
        return bad

    def score_pose(self, x, y):
        """MapGridCritic::scorePose. None means 'Goes Off Grid'."""
        c = self.cm.world_to_map(x, y)
        if c is None:
            return None
        return self.vals[self.cm.index(*c)]


def seeds_path_dist(cm, plan):
    """PathDistCritic::prepare -- every plan cell until the plan leaves
    the costmap, all seeded at 0."""
    adj = adjust_plan_resolution(plan, cm.res)
    seeds, started = [], False
    for (gx, gy) in adj:
        c = cm.world_to_map(gx, gy)
        if c is not None and cm.cost(*c) != NO_INFORMATION:
            seeds.append(c)
            started = True
        elif started:
            break
    return seeds, adj


def seeds_goal_dist(cm, plan):
    """GoalDistCritic::getLastPoseOnCostmap -- the LAST plan cell still on
    the costmap. One seed, not the goal per se: whatever pose of the
    global plan is last inside the local window."""
    adj = adjust_plan_resolution(plan, cm.res)
    last, started = None, False
    for (gx, gy) in adj:
        c = cm.world_to_map(gx, gy)
        if c is not None and cm.cost(*c) != NO_INFORMATION:
            last = c
            started = True
        elif started:
            break
    return ([last] if last else []), adj


def forward_pose(x, y, th, distance):
    """dwb_critics::getForwardPose."""
    return (x + distance * math.cos(th), y + distance * math.sin(th))


def score_mapgrid_traj(grid, poses, fpd, stop_on_failure):
    """MapGridCritic::scoreTrajectory, aggregation_type 'last'.

    stop_on_failure True  (PathDist, GoalDist): every pose is checked for
                          the obstacle/unreachable sentinels and the
                          trajectory is illegal if any hits one; the score
                          is the last pose's value.
    stop_on_failure False (PathAlign, GoalAlign): start_index jumps to the
                          last pose, so only that one is read -- but
                          scorePose still throws if it is off the grid.
    fpd                   forward_point_distance; 0.0 for the Dist critics.
    """
    idxs = range(len(poses)) if stop_on_failure else [len(poses) - 1]
    score = None
    for i in idxs:
        x, y, th = poses[i]
        if fpd:
            x, y = forward_pose(x, y, th, fpd)
        v = grid.score_pose(x, y)
        if v is None:
            return ('ILLEGAL', 'Trajectory Goes Off Grid.')
        if stop_on_failure:
            if v == grid.obstacle_score:
                return ('ILLEGAL', 'Trajectory Hits Obstacle.')
            if v == grid.unreachable_score:
                return ('ILLEGAL', 'Trajectory Hits Unreachable Area.')
        score = v
    return ('OK', score)


def score_base_obstacle(cm, poses):
    """BaseObstacleCritic with sum_scores false: the LAST pose's cost, and
    illegal if ANY pose is lethal / inscribed / unknown."""
    score = 0.0
    for (x, y, _th) in poses:
        c = cm.world_to_map(x, y)
        if c is None:
            return ('ILLEGAL', 'Trajectory Goes Off Grid.')
        cost = cm.cost(*c)
        if cost in (LETHAL_OBSTACLE, INSCRIBED_INFLATED_OBSTACLE,
                    NO_INFORMATION):
            return ('ILLEGAL', 'Trajectory Hits Obstacle.')
        score = float(cost)
    return ('OK', score)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else \
        '.navbench/results/c2n3_stall.json'
    which = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    d = json.load(open(path))
    s = d['snapshots'][which]

    cm = Costmap(s['costmap'])
    plan = [(p[0], p[1]) for p in s['transformed_plan']['poses']]
    print(f'=== {path} snapshot {which} ===')
    print(f"robot (world) {s['gt']['x']:.4f}, {s['gt']['y']:.4f} "
          f"yaw {s['gt']['yaw']:.4f}")
    print(f"distance to goal {s['dist_to_goal_world']:.4f} m   "
          f"heading error to goal {s['heading_error_to_goal_deg']:.2f} deg")
    print(f"stalled for {s['stalled_for_s']} s")
    print(f"costmap {cm.nx}x{cm.ny} @ {cm.res:.4f} m, origin "
          f"({cm.ox:.3f}, {cm.oy:.3f}) frame {s['costmap']['frame_id']}, "
          f"costmap age vs /evaluation {s['costmap']['age_s_vs_eval']:+.3f} s")
    print(f"transformed plan {len(plan)} poses, frame "
          f"{s['transformed_plan']['frame_id']}, age "
          f"{s['transformed_plan']['age_s_vs_eval']:+.3f} s")

    rx, ry = s['tf']['odom_from_base']['x'], s['tf']['odom_from_base']['y']
    rcell = cm.world_to_map(rx, ry)
    print(f"robot in costmap frame ({rx:.4f}, {ry:.4f}) -> cell {rcell}, "
          f"cost {cm.cost(*rcell) if rcell else 'OFF'}")

    # -- the two grids ---------------------------------------------------
    pd_seeds, adj = seeds_path_dist(cm, plan)
    gd_seeds, _ = seeds_goal_dist(cm, plan)
    print(f"\nadjustPlanResolution: {len(plan)} -> {len(adj)} poses "
          f"({'no-op' if len(adj) == len(plan) else 'interpolated'})")
    print(f"PathDist seeds: {len(pd_seeds)} cells  (first {pd_seeds[:2]} "
          f"last {pd_seeds[-2:] if pd_seeds else None})")
    print(f"GoalDist seed : {gd_seeds}  <- the LAST plan pose on the "
          f"costmap, which is what GoalDist measures distance to")
    if gd_seeds:
        gx = cm.ox + (gd_seeds[0][0] + 0.5) * cm.res
        gy = cm.oy + (gd_seeds[0][1] + 0.5) * cm.res
        print(f"  that cell is ({gx:.3f}, {gy:.3f}) in "
              f"{s['costmap']['frame_id']}; plan's own last pose is "
              f"({plan[-1][0]:.3f}, {plan[-1][1]:.3f})")
        if rcell:
            l1 = (abs(gd_seeds[0][0] - rcell[0])
                  + abs(gd_seeds[0][1] - rcell[1]))
            l2 = math.dist((rx, ry), (plan[-1][0], plan[-1][1]))
            print(f"  L1(robot cell, goal cell) = {l1} cells = "
                  f"{l1 * cm.res:.3f} m ;  Euclidean = {l2:.3f} m")

    pdg = MapGrid(cm)
    pdg.seeds = pd_seeds
    pdg.propagate()
    gdg = MapGrid(cm)
    gdg.seeds = gd_seeds
    gdg.propagate()
    print(f"\nflood vs direct min-L1 mismatches: PathDist {pdg.check_l1()}, "
          f"GoalDist {gdg.check_l1()}  (must be 0)")

    # GoalAlign uses GoalDist's grid built on a plan whose LAST pose is
    # nudged forward_point_distance along the robot->goal bearing.
    fpd = 0.1
    goal_x, goal_y = plan[-1][0], plan[-1][1]
    ang = math.atan2(goal_y - ry, goal_x - rx)
    ga_plan = list(plan)
    ga_plan[-1] = (goal_x + fpd * math.cos(ang), goal_y + fpd * math.sin(ang))
    ga_seeds, _ = seeds_goal_dist(cm, ga_plan)
    gag = MapGrid(cm)
    gag.seeds = ga_seeds
    gag.propagate()
    print(f"GoalAlign seed: {ga_seeds} (GoalDist seed nudged {fpd} m along "
          f"the robot->goal bearing {math.degrees(ang):.1f} deg)")

    # PathAlign reads PathDist's grid at the forward point.
    print()
    print('=== rebuilt vs published, per probe trajectory ===')
    print('Every value below is a RAW critic score (cells for the MapGrid')
    print('critics, costmap cost for BaseObstacle). "pub" is what DWB')
    print('published on /evaluation; "reb" is this rebuild.')
    hdr = ('  {:<13} {:>8} {:>8} {:>5} | {:>16} {:>16} {:>16} {:>16} {:>16}')
    print(hdr.format('probe', 'vx', 'wz', 'nposes', 'BaseObstacle',
                     'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'))
    ok_n = bad_n = 0
    for label, p in s['probes'].items():
        poses = p['poses']
        pub = {n: raw for n, raw, sc in p['critics']}
        reb = {}
        st, v = score_base_obstacle(cm, poses)
        reb['BaseObstacle'] = v if st == 'OK' else f'ILL:{v[:14]}'
        st, v = score_mapgrid_traj(gag, poses, fpd, False)
        reb['GoalAlign'] = v if st == 'OK' else f'ILL:{v[:14]}'
        st, v = score_mapgrid_traj(pdg, poses, fpd, False)
        reb['PathAlign'] = v if st == 'OK' else f'ILL:{v[:14]}'
        st, v = score_mapgrid_traj(pdg, poses, 0.0, True)
        reb['PathDist'] = v if st == 'OK' else f'ILL:{v[:14]}'
        st, v = score_mapgrid_traj(gdg, poses, 0.0, True)
        reb['GoalDist'] = v if st == 'OK' else f'ILL:{v[:14]}'
        cells = []
        for k in ['BaseObstacle', 'GoalAlign', 'PathAlign', 'PathDist',
                  'GoalDist']:
            pv = pub.get(k)
            rv = reb[k]
            if pv is None:
                cells.append(f'--/{rv}')            # short-circuited: not
                continue                            # published, rebuilt only
            match = isinstance(rv, float) and abs(rv - pv) < 1e-6
            ok_n += int(match)
            bad_n += int(not match)
            cells.append(f'{pv:g}/{rv if not isinstance(rv, float) else rv:g}'
                         + ('' if match else ' X'))
        print(hdr.format(label, f"{p['evaluated'][0]:.4f}",
                         f"{p['evaluated'][1]:.4f}", len(poses), *cells))
    print(f'\npublished critics reproduced: {ok_n} matched, {bad_n} did not')
    print('("--/x" = DWB short-circuited before reaching that critic and')
    print(' published no value for it; the rebuild says what it would be.)')

    # -- what the cost field around the robot looks like ------------------
    print()
    print('=== BaseObstacle cost along the straight-ahead ray ===')
    th = s['tf']['odom_from_base']['yaw']
    print('  {:>7} {:>9} {:>9} {:>7} {:>8} {:>9} {:>9}'.format(
        'd_m', 'x', 'y', 'cell', 'cost', 'GoalDist', 'PathDist'))
    dd = 0.0
    while dd <= 1.0001:
        x = rx + dd * math.cos(th)
        y = ry + dd * math.sin(th)
        c = cm.world_to_map(x, y)
        if c is None:
            print(f'  {dd:>7.2f}  OFF GRID')
        else:
            print('  {:>7.2f} {:>9.3f} {:>9.3f} {:>7} {:>8} {:>9.0f} '
                  '{:>9.0f}'.format(
                      dd, x, y, f'{c[0]},{c[1]}', cm.cost(*c),
                      gdg.vals[cm.index(*c)], pdg.vals[cm.index(*c)]))
        dd += 0.05
    return 0


if __name__ == '__main__':
    sys.exit(main())
