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
"""C2-NAV.17: is the GOOD/BAD route split a genuine near-tie in
SmacPlanner2D's own objective, flipped by the ~148 mm / 10 deg start-pose
difference C2-NAV.16 measured at the WAYPOINT RemovePassedGoals tick?

C2-NAV.16 (docs/data/c2nav16_bench.json, `first_divergence`) established,
from COMMITTED /plan captures, that:
  - GOOD (c2n15_tour_r1, SUCCEEDED) and BAD (c2n16_tour_r1, TIMEOUT) are
    near-identical up to the WAYPOINT removal tick (t0+9.0s), diverging by
    147.6 mm / ~10 deg in robot pose at that instant.
  - The FIRST /plan snapshot at/after that tick already carries the full
    difference: GOOD's replan clears box_obstacle_1 by 292.4 mm (never
    enters PolygonStop or the SW column); BAD's replan clears it by only
    230.6 mm (inside PolygonStop, inside the SW column) -- CASE A, the
    global planner choosing the bad route, not DWB drifting from a good
    one.
That leaves one question no prior C2-NAV session answered: is this a
GENUINE near-tie in SmacPlanner2D's own cost objective (so that the
measured 148 mm/10 deg state difference is a PLAUSIBLE cause), or are the
two routes far apart in cost (so the state difference alone cannot
explain the flip, and something else -- e.g. costmap staleness, a
different obstacle_layer scan -- must be invoked)?

This module changes NOTHING navigational. It is pure offline geometry:
  1. `build_costmap()`   -- reconstructs the ACTUAL Nav2 global costmap
                             cost field (resolution 0.05 m, robot_radius
                             0.20 m, inflation_radius 0.5 m, GLOBAL
                             cost_scaling_factor 5.0 -- all read verbatim
                             from docs/data/c2nav11_ntp_params.yaml, the
                             byte-identical params file both C2-NAV.15 and
                             C2-NAV.16 ran), using the documented Nav2
                             InflationLayer formula, over the SAME box/
                             cylinder geometry c2nav9_corridor.py already
                             verified against coco_world.world.
  2. `astar_2d()`        -- an 8-connected grid search that reproduces,
                             as closely as this environment lets it be
                             verified, nav2_smac_planner::Node2D's own
                             search: straight-line (Euclidean) heuristic,
                             step cost length*(1 + cost_travel_multiplier
                             * cell_cost/252), cost_travel_multiplier=2.0
                             (the installed GridBased param). Node2D has
                             NO heading state (see SOURCE FINDINGS in the
                             module docstring below) -- this searcher
                             takes no heading input, structurally, not by
                             omission.
  3. `classify_route()`  -- SAFE vs SW-column, the EXACT test
                             c2nav15_planwindow.py's `plan_enters_sw_column`
                             already uses on the real /plan captures, so a
                             synthetic path and a captured one are
                             comparable by construction.
  4. Reproduces the two REAL captured plans (GOOD/BAD start -> goal) from
     this reconstruction and checks whether it recovers the REAL
     GOOD-safe / BAD-SW split -- the tool's own validation gate, not a
     side check (section 16 requires this).
  5. Sweeps start position (+/-0.15 m, both axes) and, separately,
     confirms heading has no effect (by construction), to find whether a
     route-selection BOUNDARY exists between the two real start poses.

SOURCE FINDINGS (nav2_smac_planner 1.3.11, installed at
/opt/ros/jazzy/include/nav2_smac_planner/*.hpp -- headers only; the
package ships no .cpp, apt has no deb-src entry, and this environment has
no outbound network access (`curl` to github.com timed out), so the
*.cpp bodies (node_2d.cpp, a_star.cpp) could NOT be inspected directly.
Everything below is either (a) read verbatim from the installed header
files -- cited by file:line, OBSERVED -- or (b) the publicly documented
Nav2 cost-map/planner behaviour (navigation.ros.org), cited as
DOCUMENTED, not verified against this machine's compiled source:

  OBSERVED (node_2d.hpp):
    - `static float cost_travel_multiplier;` (Node2D, line ~271) -- 2D
      search has its OWN weight parameter, separate from the generic
      `SearchInfo.cost_penalty` (types.hpp) that Hybrid-A*/Lattice use.
      The installed GridBased config sets `cost_travel_multiplier: 2.0`
      and never sets `cost_penalty` -- consistent with this being the 2D
      planner, not Hybrid.
    - `getCoords(index, width, angles)` THROWS
      "Node type Node2D does not have a valid angle quantization" unless
      `angles == 1` (node_2d.hpp ~200-210). Node2D's graph has exactly
      one angle bin. There is no heading state in the search AT ALL for
      SmacPlanner2D -- this is a hard structural fact, not a tuning
      default.
    - `getTraversalCost(child)` and `getHeuristicCost(coords, goal)` are
      DECLARED (float-returning, take a child node / two coordinate
      pairs) but their bodies are compiled into libnav2_smac_planner_2d.so
      and are NOT available to read in this environment.
  OBSERVED (a_star.hpp):
    - `NodeComparator::operator()` is `return a.first > b.first;` -- a
      plain `std::priority_queue` min-heap on total cost ONLY. There is
      NO secondary tie-break key (no index, no insertion order, no
      preference for straighter paths). Ties in the open list resolve by
      whatever order the underlying container happens to hold them in,
      which depends on expansion order -- itself a function of the START
      cell. This is the mechanism by which a GENUINE cost tie between two
      routes can resolve differently for two different (but nearby) start
      cells, without any randomness or any change to the costmap.
  DOCUMENTED, not verified against local source (Nav2 project docs,
  navigation.ros.org "Costmap2D" and "SmacPlanner" pages, and the
  well-established public formula used across the Nav2 docs and this
  repo's OWN prior sessions -- e.g. C2-NAV.4/.5's inflation-cost
  derivations already use it):
    - InflationLayer cost: 254 at distance<=0 (LETHAL_OBSTACLE); 253 for
      0 < distance <= robot_radius (INSCRIBED_INFLATED_OBSTACLE, footprint
      here is a circle so inscribed==circumscribed==robot_radius); for
      robot_radius < distance <= inflation_radius,
      cost = round(252*exp(-cost_scaling_factor*(distance-robot_radius))+1);
      0 beyond inflation_radius.
    - Costmap cost >= INSCRIBED_INFLATED_OBSTACLE (253) is treated as
      not-traversable by planning (the robot's centre cannot legally
      occupy that cell).
    - Node2D::getTraversalCost's normalising denominator (252 vs 253) and
      exact algebraic form could not be confirmed against source in this
      environment; 252.0 is used below and flagged everywhere it matters.
    - Default 2D search neighbourhood is 8-connected (Moore); no
      `motion_model_for_search` override is present in
      c2nav11_ntp_params.yaml's GridBased block.
    - `getHeuristicCost` for Node2D is the straight-line (Euclidean)
      distance to the goal cell -- the standard admissible A* heuristic
      for an 8-connected grid with unit/root-2 step costs.

Usage:
  python3 c2nav17_routeselect.py selftest
  python3 c2nav17_routeselect.py replay      # reproduce the 2 real plans
  python3 c2nav17_routeselect.py sweep       # position-sensitivity grid
  python3 c2nav17_routeselect.py heading     # heading sensitivity
  python3 c2nav17_routeselect.py boundary    # interpolated GOOD->BAD line
  python3 c2nav17_routeselect.py dump <out.json>
  python3 c2nav17_routeselect.py viz
  python3 c2nav17_routeselect.py all
"""
import heapq
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c2nav9_corridor import (                                # noqa: E402
    BOXES, EXTRA_BOXES, CIRCLES, rect, dist_to_box, GRID_X, GRID_Y,
    grid_index, build_clearance_grid, bottleneck, CORRIDOR_GATE_GOAL,
    FOOT_CIRC_R,
)
from c2nav8_report import nearest_full                       # noqa: E402
from c2nav12_report import (                                 # noqa: E402
    WAYPOINT, GOAL_SHIFTED, SW_CORNER, DEADLOCK_POSE,
)

REPO_ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
C2NAV16_JSON = os.path.join(HERE, 'c2nav16_bench.json')
BOX1 = [b for b in BOXES if b[0] == 'box_obstacle_1'][0]
BOX1_X0, BOX1_X1, BOX1_Y0, BOX1_Y1 = rect(BOX1)

# ---------------------------------------------------------------------
# 0. CANONICAL CONFIG, read verbatim from docs/data/c2nav11_ntp_params.yaml
#    -- the byte-identical params file BOTH C2-NAV.15 (GOOD) and
#    C2-NAV.16 (BAD) ran (params_sha256 in c2nav12_bench.json /
#    c2nav16_compare.py header). NOT the live gazebo_models/config/
#    nav2_params.yaml, which is the frozen baseline this experiment does
#    not touch and which still carries C2-NAV.2's rejected values.
# ---------------------------------------------------------------------
RESOLUTION = 0.05             # global_costmap.resolution
ROBOT_RADIUS = 0.20           # global_costmap.robot_radius
INFLATION_RADIUS = 0.5        # global_costmap.inflation_layer.inflation_radius
CSF_GLOBAL = 5.0              # global_costmap.inflation_layer.cost_scaling_factor
LETHAL_OBSTACLE = 254.0
INSCRIBED_INFLATED_OBSTACLE = 253.0
COST_NORM = 252.0             # DOCUMENTED, not source-verified -- see module docstring
COST_TRAVEL_MULTIPLIER = 2.0  # planner_server.GridBased.cost_travel_multiplier

# GOOD/BAD critical-replan states, quoted verbatim from the C2-NAV.16
# committed record (first_divergence.gt_good_tick / gt_bad_tick), the
# robot's actual ground-truth pose at t0+9.0s -- the sample nearest the
# WAYPOINT-removal RateController tick in BOTH runs.
GOOD_START = (-2.5604, 1.645)
GOOD_START_YAW = 0.9792
BAD_START = (-2.6766, 1.554)
BAD_START_YAW = 0.8049
POSE_DELTA_MM = 147.6   # c2nav16_bench.json first_divergence.pose_delta_at_tick_mm

# The two REAL captured plans at/after the tick (c2nav16_bench.json
# first_divergence.good_tick / bad_tick), for direct comparison against
# this module's synthetic replan.
REAL_GOOD_MIN_CLEARANCE_M = 0.2924
REAL_GOOD_SW_COLUMN = False
REAL_BAD_MIN_CLEARANCE_M = 0.2306
REAL_BAD_SW_COLUMN = True
REAL_GOAL_LAST_POSE = (-3.519, 2.94)   # both ticks' last_pose, identical


def hdr(t):
    print()
    print('=' * 78)
    print(t)
    print('=' * 78)


# ---------------------------------------------------------------------
# 1. NAV2 GLOBAL COSTMAP RECONSTRUCTION (documented InflationLayer formula
#    over c2nav9_corridor's own verified geometry).
# ---------------------------------------------------------------------

def build_costmap(res=RESOLUTION):
    xs = np.arange(GRID_X[0], GRID_X[1] + res, res)
    ys = np.arange(GRID_Y[0], GRID_Y[1] + res, res)
    X, Y = np.meshgrid(xs, ys, indexing='xy')
    dist = np.full(X.shape, np.inf)
    for b in list(BOXES) + list(EXTRA_BOXES):
        x0, x1, y0, y1 = rect(b)
        qx = np.clip(X, x0, x1)
        qy = np.clip(Y, y0, y1)
        dist = np.minimum(dist, np.hypot(X - qx, Y - qy))
    for (_name, cx, cy, r) in CIRCLES:
        dist = np.minimum(dist, np.maximum(np.hypot(X - cx, Y - cy) - r, 0.0))

    cost = np.zeros_like(dist)
    cost[dist <= 0.0] = LETHAL_OBSTACLE
    m_insc = (dist > 0.0) & (dist <= ROBOT_RADIUS)
    cost[m_insc] = INSCRIBED_INFLATED_OBSTACLE
    m_infl = (dist > ROBOT_RADIUS) & (dist <= INFLATION_RADIUS)
    cost[m_infl] = np.round(
        (INSCRIBED_INFLATED_OBSTACLE - 1) *
        np.exp(-CSF_GLOBAL * (dist[m_infl] - ROBOT_RADIUS)) + 1.0)
    return xs, ys, cost, dist


# ---------------------------------------------------------------------
# 2. Node2D-style 8-connected A* (see module docstring SOURCE FINDINGS
#    for exactly which parts are OBSERVED vs DOCUMENTED-not-verified).
# ---------------------------------------------------------------------

NEI8 = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]


def sw_column_mask(xs, ys):
    """Boolean grid, True where a cell satisfies the EXACT SW-column test
    `classify_route`/c2nav15_planwindow use (dist_to_box < 0.60 m, south
    of the box, within 0.15 m of the box's west face). Used to BLOCK that
    region and force A* to find the best available route that never
    enters it -- the only way to price "the SAFE route" when no real
    captured polyline for it is committed (only summary stats are)."""
    X, Y = np.meshgrid(xs, ys, indexing='xy')
    mask = np.zeros(X.shape, dtype=bool)
    ny, nx = X.shape
    for iy in range(ny):
        for ix in range(nx):
            x, y = X[iy, ix], Y[iy, ix]
            d_box, _q = dist_to_box(x, y, BOX1)
            if d_box < 0.60 and y < BOX1_Y0 and x < BOX1_X0 + 0.15:
                mask[iy, ix] = True
    return mask


def astar_2d(cost, xs, ys, start_xy, goal_xy,
             cost_travel_multiplier=COST_TRAVEL_MULTIPLIER, block_mask=None):
    """8-connected grid A*. Heading is NOT a parameter -- Node2D has no
    heading state (OBSERVED, node_2d.hpp getCoords()); this function
    takes none, structurally, to make that fact impossible to violate by
    accident. `block_mask`, if given, is an ADDITIONAL exclusion (e.g.
    sw_column_mask) -- not part of the real costmap, used only to price
    the best route that avoids a named region (see sw_column_mask)."""
    res = xs[1] - xs[0]
    ny, nx = cost.shape
    si = grid_index(xs, ys, *start_xy)
    gi = grid_index(xs, ys, *goal_xy)

    def valid(iy, ix):
        if not (0 <= iy < ny and 0 <= ix < nx):
            return False
        if cost[iy, ix] >= INSCRIBED_INFLATED_OBSTACLE:
            return False
        if block_mask is not None and block_mask[iy, ix]:
            return False
        return True

    if not valid(*si):
        return dict(found=False, reason=f'start cell {si} is in collision '
                    f'(cost={cost[si]})')
    if not valid(*gi):
        return dict(found=False, reason=f'goal cell {gi} is in collision '
                    f'(cost={cost[gi]})')

    counter = 0
    openq = [(0.0, counter, si)]
    gscore = {si: 0.0}
    parent = {}
    closed = set()

    def h(iy, ix):
        return math.hypot((ix - gi[1]) * res, (iy - gi[0]) * res)

    found = False
    while openq:
        _f, _c, node = heapq.heappop(openq)
        if node in closed:
            continue
        closed.add(node)
        if node == gi:
            found = True
            break
        iy, ix = node
        for dx, dy, base in NEI8:
            nb = (iy + dy, ix + dx)
            if nb in closed or not valid(*nb):
                continue
            child_cost = cost[nb]
            normalized = child_cost / COST_NORM
            step = base * res * (1.0 + cost_travel_multiplier * normalized)
            ng = gscore[node] + step
            if nb not in gscore or ng < gscore[nb] - 1e-12:
                gscore[nb] = ng
                parent[nb] = node
                counter += 1
                heapq.heappush(openq, (ng + h(*nb), counter, nb))

    if not found:
        return dict(found=False, reason='no path found (search exhausted)')

    path_idx = [gi]
    n = gi
    while n != si:
        n = parent[n]
        path_idx.append(n)
    path_idx.reverse()
    path_xy = [(float(xs[ix]), float(ys[iy])) for (iy, ix) in path_idx]
    length = sum(math.dist(path_xy[i], path_xy[i + 1])
                 for i in range(len(path_xy) - 1))
    cells = [cost[iy, ix] for (iy, ix) in path_idx]
    return dict(found=True, path=path_xy, path_cells=path_idx,
                length_m=round(length, 4),
                integrated_cost=round(gscore[gi], 4),
                n_cells=len(path_xy),
                max_cost=float(max(cells)), mean_cost=float(np.mean(cells)),
                n_cells_cost_gt0=int(sum(1 for c in cells if c > 0)),
                n_iterations=counter)


def classify_route(path_xy):
    """EXACT test c2nav15_planwindow.analyze_snapshot uses for
    `plan_enters_sw_column`, applied here to a synthetic path so a
    synthetic and a captured route are comparable by construction."""
    sw = False
    min_clear = math.inf
    min_clear_who = None
    for (x, y) in path_xy:
        d_box, _q = dist_to_box(x, y, BOX1)
        if d_box < 0.60 and y < BOX1_Y0 and x < BOX1_X0 + 0.15:
            sw = True
        d_all, who, _q2 = nearest_full(x, y)[0]
        if d_all < min_clear:
            min_clear = d_all
            min_clear_who = who
    return dict(sw_column=sw, min_clearance_m=round(min_clear, 4),
                min_clearance_obstacle=min_clear_who,
                enters_polygon_stop=min_clear < 0.25)


def route_split_cost(cost, xs, ys, start_xy, goal_xy):
    """The actual cost GAP between the two route classes from a given
    start: (1) the UNCONSTRAINED A* optimum (whatever class it happens to
    be), and (2) the best route available with the SW column EXCLUDED
    from the graph entirely (sw_column_mask). No real captured plan's
    full polyline is committed (only summary stats), so this is the only
    way, offline, to price "the SAFE route" directly rather than only
    observing which class an unconstrained search happens to prefer."""
    mask = sw_column_mask(xs, ys)
    unconstrained = astar_2d(cost, xs, ys, start_xy, goal_xy)
    safe_only = astar_2d(cost, xs, ys, start_xy, goal_xy, block_mask=mask)
    out = dict(unconstrained=unconstrained, safe_only=safe_only)
    if unconstrained.get('found') and safe_only.get('found'):
        cls_u = classify_route(unconstrained['path'])
        cls_s = classify_route(safe_only['path'])
        out['unconstrained_class'] = cls_u
        out['safe_only_class'] = cls_s
        dcost = safe_only['integrated_cost'] - unconstrained['integrated_cost']
        rel = dcost / unconstrained['integrated_cost'] if unconstrained['integrated_cost'] else float('nan')
        out['safe_minus_unconstrained_cost'] = round(dcost, 4)
        out['safe_minus_unconstrained_cost_pct'] = round(rel * 100, 2)
        dlen = safe_only['length_m'] - unconstrained['length_m']
        out['safe_minus_unconstrained_length_m'] = round(dlen, 4)
    return out


# ---------------------------------------------------------------------
# 3. SELF-TEST
# ---------------------------------------------------------------------

def self_test():
    hdr('SELF-TEST: reproduce known committed facts before trusting '
        'anything new')
    ok = True

    xs, ys, clr = build_clearance_grid()
    tau = bottleneck(clr, xs, ys, CORRIDOR_GATE_GOAL, GOAL_SHIFTED)
    print(f'  c2nav9 whole-corridor bottleneck: {tau*1000:.1f} mm  '
          f'want ~326.0 mm  {"PASS" if abs(tau*1000-326.0) < 1.0 else "FAIL"}')
    ok &= abs(tau * 1000 - 326.0) < 1.0

    print(f'  box_obstacle_1 SW corner: {(BOX1_X0, BOX1_Y0)}  want '
          f'{SW_CORNER}  {"PASS" if (BOX1_X0, BOX1_Y0) == SW_CORNER else "FAIL"}')
    ok &= (BOX1_X0, BOX1_Y0) == SW_CORNER

    d = math.dist(GOOD_START, BAD_START)
    print(f'  GOOD_START/BAD_START distance: {d*1000:.1f} mm  want 147.6 mm  '
          f'{"PASS" if abs(d*1000 - POSE_DELTA_MM) < 0.5 else "FAIL"}')
    ok &= abs(d * 1000 - POSE_DELTA_MM) < 0.5

    dyaw = math.degrees(GOOD_START_YAW - BAD_START_YAW)
    print(f'  GOOD/BAD yaw delta: {dyaw:.2f} deg  want ~10 deg  '
          f'{"PASS" if abs(dyaw - 9.99) < 0.5 else "FAIL"}')
    ok &= abs(dyaw - 9.99) < 0.5

    # Cross-check the reconstructed cost-field's own `dist` layer against
    # c2nav9_corridor's independently-built clearance grid at a shared
    # sample point (both computed from the same box list, different
    # resolution/extent bookkeeping -- must agree to within half a cell).
    myxs, myys, mycost, mydist = build_costmap()
    iy, ix = grid_index(myxs, myys, *DEADLOCK_POSE)
    d_mine = mydist[iy, ix]
    d_ref = nearest_full(*DEADLOCK_POSE)[0][0]
    print(f'  reconstructed dist-field @ DEADLOCK_POSE: {d_mine:.4f} m vs '
          f'nearest_full: {d_ref:.4f} m  '
          f'{"PASS" if abs(d_mine - d_ref) < RESOLUTION else "FAIL"}')
    ok &= abs(d_mine - d_ref) < RESOLUTION

    # At the goal, cost must be > 0 (inside inflation) but < INSCRIBED
    # (not blocked) -- c2nav9_corridor's own "0.63 m NW pinch, 0.30 m
    # free band" framing already implies the goal itself sits in the
    # inflated gradient, not in the lethal/inscribed zone.
    giy, gix = grid_index(myxs, myys, *GOAL_SHIFTED)
    gc = mycost[giy, gix]
    print(f'  cost at GOAL_SHIFTED {GOAL_SHIFTED}: {gc:.1f}  want 0 < cost < '
          f'{INSCRIBED_INFLATED_OBSTACLE:.0f}  '
          f'{"PASS" if 0 < gc < INSCRIBED_INFLATED_OBSTACLE else "FAIL"}')
    ok &= 0 < gc < INSCRIBED_INFLATED_OBSTACLE

    print()
    print('SELF-TEST: ALL PASS' if ok else 'SELF-TEST: FAILURE -- DO NOT '
          'TRUST ANYTHING BELOW')
    return ok


# ---------------------------------------------------------------------
# 4. REPLAY: does this reconstruction recover the REAL GOOD-safe /
#    BAD-SW split from the two real start poses?
# ---------------------------------------------------------------------

def replay():
    hdr('REPLAY: synthetic A* from the two REAL captured start poses')
    xs, ys, cost, dist = build_costmap()
    out = {}
    for name, start, real_clr, real_sw in (
            ('GOOD', GOOD_START, REAL_GOOD_MIN_CLEARANCE_M, REAL_GOOD_SW_COLUMN),
            ('BAD', BAD_START, REAL_BAD_MIN_CLEARANCE_M, REAL_BAD_SW_COLUMN)):
        r = astar_2d(cost, xs, ys, start, GOAL_SHIFTED)
        if not r['found']:
            print(f'  {name}: NO PATH ({r["reason"]})')
            out[name] = r
            continue
        cls = classify_route(r['path'])
        match = cls['sw_column'] == real_sw
        print(f'  {name}: start={start}  -> synthetic route: '
              f'len={r["length_m"]}m  integrated_cost={r["integrated_cost"]}  '
              f'min_clr={cls["min_clearance_m"]}m (real {real_clr}m)  '
              f'SW_column={cls["sw_column"]} (real {real_sw})  '
              f'{"MATCH" if match else "MISMATCH"}')
        r.update(cls)
        r['matches_real'] = match
        out[name] = r
    both_match = out.get('GOOD', {}).get('matches_real') and \
        out.get('BAD', {}).get('matches_real')
    print()
    print(f'  RECONSTRUCTION {"REPRODUCES" if both_match else "DOES NOT "
          "FULLY REPRODUCE"} the real GOOD-safe/BAD-SW split.')
    out['both_match'] = bool(both_match)

    print()
    print('  Since no real captured plan\'s full polyline is committed '
          '(only summary stats), the SAFE route\'s own cost is priced by '
          're-running A* with the SW column cells EXCLUDED from the graph '
          '-- the best available alternative -- and compared against the '
          'UNCONSTRAINED optimum from the same start:')
    for name, start in (('GOOD_START', GOOD_START), ('BAD_START', BAD_START)):
        rs = route_split_cost(cost, xs, ys, start, GOAL_SHIFTED)
        out[f'{name}_split'] = rs
        u, s = rs.get('unconstrained'), rs.get('safe_only')
        if not (u and u.get('found') and s and s.get('found')):
            print(f'    {name}: could not compute both alternatives')
            continue
        print(f'    {name}: unconstrained cost={u["integrated_cost"]} '
              f'(class {"SW" if rs["unconstrained_class"]["sw_column"] else "safe"}), '
              f'SW-excluded cost={s["integrated_cost"]} '
              f'(class {"SW" if rs["safe_only_class"]["sw_column"] else "safe"})  '
              f'gap = {rs["safe_minus_unconstrained_cost"]:+.3f} '
              f'({rs["safe_minus_unconstrained_cost_pct"]:+.2f}%)  '
              f'length gap = {rs["safe_minus_unconstrained_length_m"]*1000:+.0f} mm')
    return out


# ---------------------------------------------------------------------
# 5. POSITION SENSITIVITY SWEEP (brief section 8/10) + boundary line
#    (brief section 5's "route-selection boundary").
# ---------------------------------------------------------------------

def sweep_start_position(half=0.15, step=0.02):
    hdr(f'START-POSITION SENSITIVITY: +/-{half*1000:.0f} mm grid, '
        f'{step*1000:.0f} mm steps, around the GOOD/BAD midpoint')
    xs, ys, cost, dist = build_costmap()
    cx = (GOOD_START[0] + BAD_START[0]) / 2
    cy = (GOOD_START[1] + BAD_START[1]) / 2
    offs = np.arange(-half, half + 1e-9, step)
    rows = []
    n_safe = n_sw = n_nopath = 0
    for dy in offs:
        line = []
        for dx in offs:
            sx, sy = cx + dx, cy + dy
            r = astar_2d(cost, xs, ys, (sx, sy), GOAL_SHIFTED)
            if not r['found']:
                line.append('.')
                n_nopath += 1
                continue
            cls = classify_route(r['path'])
            if cls['sw_column']:
                line.append('S')
                n_sw += 1
            else:
                line.append('_')
                n_safe += 1
            rows.append(dict(x=round(sx, 4), y=round(sy, 4),
                              sw_column=cls['sw_column'],
                              min_clearance_m=cls['min_clearance_m'],
                              length_m=r['length_m'],
                              integrated_cost=r['integrated_cost']))
        print('  ' + ''.join(line))
    print()
    print(f'  legend: _ = SAFE route, S = SW-column route, . = start cell '
          f'in collision')
    print(f'  {n_safe} SAFE, {n_sw} SW, {n_nopath} no-path over '
          f'{len(offs)*len(offs)} start cells')
    # Where do GOOD_START/BAD_START fall relative to this grid?
    print(f'  GOOD_START offset from centre: '
          f'({(GOOD_START[0]-cx)*1000:+.0f}, {(GOOD_START[1]-cy)*1000:+.0f}) mm')
    print(f'  BAD_START  offset from centre: '
          f'({(BAD_START[0]-cx)*1000:+.0f}, {(BAD_START[1]-cy)*1000:+.0f}) mm')
    boundary_exists = n_safe > 0 and n_sw > 0
    verb = 'EXISTS' if boundary_exists else 'DOES NOT APPEAR'
    print()
    print(f'  A route-selection BOUNDARY {verb} inside this '
          f'+/-{half*1000:.0f} mm neighbourhood.')
    return dict(center=(cx, cy), half_m=half, step_m=step,
                n_safe=n_safe, n_sw=n_sw, n_nopath=n_nopath,
                boundary_exists=boundary_exists, rows=rows)


def boundary_line(n=41):
    """Interpolate directly along the GOOD_START -> BAD_START segment,
    extended 50% past each end, and find every sign change -- the most
    direct test of whether the measured 147.6 mm displacement itself
    crosses a boundary."""
    hdr('BOUNDARY ALONG THE GOOD_START -> BAD_START LINE (extended 50% '
        'each way)')
    xs, ys, cost, dist = build_costmap()
    x0, y0 = GOOD_START
    x1, y1 = BAD_START
    ext = 0.5
    ts = np.linspace(-ext, 1 + ext, n)
    rows = []
    prev_sw = None
    crossings = []
    for t in ts:
        sx = x0 + t * (x1 - x0)
        sy = y0 + t * (y1 - y0)
        r = astar_2d(cost, xs, ys, (sx, sy), GOAL_SHIFTED)
        if not r['found']:
            rows.append(dict(t=round(float(t), 4), x=round(sx, 4),
                              y=round(sy, 4), found=False))
            continue
        cls = classify_route(r['path'])
        rows.append(dict(t=round(float(t), 4), x=round(sx, 4), y=round(sy, 4),
                          found=True, sw_column=cls['sw_column'],
                          min_clearance_m=cls['min_clearance_m'],
                          length_m=r['length_m'],
                          integrated_cost=r['integrated_cost']))
        if prev_sw is not None and cls['sw_column'] != prev_sw:
            crossings.append(round(float(t), 4))
        prev_sw = cls['sw_column']
    for row in rows:
        tag = ('t=0.000 == GOOD_START' if abs(row['t']) < 1e-9 else
               't=1.000 == BAD_START' if abs(row['t'] - 1.0) < 1e-9 else '')
        if not row.get('found'):
            print(f'  t={row["t"]:+.3f}  ({row["x"]:+.3f},{row["y"]:+.3f})  '
                  f'NO PATH  {tag}')
            continue
        print(f'  t={row["t"]:+.3f}  ({row["x"]:+.3f},{row["y"]:+.3f})  '
              f'{"SW " if row["sw_column"] else "safe"}  '
              f'min_clr={row["min_clearance_m"]:.3f}m  '
              f'len={row["length_m"]:.3f}m  cost={row["integrated_cost"]:.3f}  {tag}')
    print()
    print(f'  route-class sign changes (t values, 0=GOOD_START, 1=BAD_START): '
          f'{crossings if crossings else "NONE in this range"}')
    return dict(rows=rows, crossings=crossings)


# ---------------------------------------------------------------------
# 6. HEADING SENSITIVITY (brief section 9) -- structural null result.
# ---------------------------------------------------------------------

def heading_sensitivity():
    hdr('HEADING SENSITIVITY')
    print('  SmacPlanner2D (nav2_smac_planner::Node2D) has NO heading state')
    print('  in its search graph -- OBSERVED directly from the installed')
    print('  header: Node2D::getCoords(index, width, angles) THROWS unless')
    print('  angles == 1 (/opt/ros/jazzy/include/nav2_smac_planner/'
          'node_2d.hpp).')
    print('  This function (astar_2d) accordingly takes no heading')
    print('  parameter -- not because heading was left out by omission,')
    print('  but because the object being modelled (Node2D) has none.')
    print()
    print('  Consequence: for a FIXED start (x, y), the global route')
    print('  SmacPlanner2D returns cannot depend on the robot\'s start yaw')
    print('  AT ALL. C2-NAV.13\'s own heading_sensitivity() already showed')
    print('  this for the COLLISION MONITOR (PolygonStop is yaw-invariant')
    print('  by construction, a circle); this session extends the same')
    print('  conclusion to the GLOBAL PLANNER, for a structurally different')
    print('  reason (no heading in the search space at all, not "yaw-')
    print('  invariant test geometry").')
    print()
    print('  The ~10 deg yaw difference C2-NAV.16 measured between')
    print('  GOOD_START and BAD_START (0.9792 vs 0.8049 rad) is therefore')
    print('  NOT a candidate mechanism for the route-selection difference,')
    print('  independent of anything this session\'s own A* reconstruction')
    print('  finds. Only the 147.6 mm POSITION difference can matter to')
    print('  SmacPlanner2D\'s route choice; C2-NAV.14\'s heading-correcting')
    print('  through-pose was accordingly never going to fix this by')
    print('  itself, which is consistent with what C2-NAV.14 measured.')
    return dict(node2d_has_heading_state=False,
                source='node_2d.hpp getCoords(), angle-quantization check')


# ---------------------------------------------------------------------
# 7. ROUTE COMPARISON TABLE (brief section 6) -- SAFE vs SW at the two
#    real starts, all the requested metrics in one place.
# ---------------------------------------------------------------------

def route_comparison():
    hdr('SAFE vs SW ROUTE COMPARISON (from the two real start poses)')
    xs, ys, cost, dist = build_costmap()
    out = {}
    for name, start in (('SAFE (from GOOD_START)', GOOD_START),
                        ('SW (from BAD_START)', BAD_START)):
        r = astar_2d(cost, xs, ys, start, GOAL_SHIFTED)
        if not r['found']:
            print(f'  {name}: NO PATH')
            continue
        cls = classify_route(r['path'])
        n_expensive = sum(1 for c in [cost[iy, ix] for iy, ix in r['path_cells']]
                          if c >= 100)
        n_zero = sum(1 for c in [cost[iy, ix] for iy, ix in r['path_cells']]
                    if c == 0)
        print(f'  {name}:')
        print(f'    path length            : {r["length_m"]} m')
        print(f'    integrated planner cost: {r["integrated_cost"]} '
              f'(cost_travel_multiplier=2.0, COST_NORM=252, see docstring)')
        print(f'    min clearance           : {cls["min_clearance_m"]} m to '
              f'{cls["min_clearance_obstacle"]}')
        print(f'    max cell cost on path   : {r["max_cost"]}')
        print(f'    mean cell cost on path  : {round(r["mean_cost"], 1)}')
        print(f'    cells with cost>=100    : {n_expensive} / {r["n_cells"]}')
        print(f'    cells at cost 0 (free)  : {n_zero} / {r["n_cells"]}')
        print(f'    enters PolygonStop reg. : {cls["enters_polygon_stop"]}')
        print(f'    enters SW column        : {cls["sw_column"]}')
        out[name] = dict(r, **cls, n_expensive_cells=n_expensive,
                          n_zero_cost_cells=n_zero)
    if len(out) == 2:
        vals = list(out.values())
        dlen = abs(vals[0]['length_m'] - vals[1]['length_m'])
        dcost = abs(vals[0]['integrated_cost'] - vals[1]['integrated_cost'])
        rel_cost = dcost / min(vals[0]['integrated_cost'], vals[1]['integrated_cost'])
        print()
        print(f'  |length difference|  = {dlen*1000:.1f} mm')
        print(f'  |integrated-cost diff| = {dcost:.3f} '
              f'({rel_cost*100:.2f}% of the smaller)')
    return out


# ---------------------------------------------------------------------
# 8. DUMP + VIZ
# ---------------------------------------------------------------------

def dump(out_path):
    record = dict(
        experiment='C2-NAV.17',
        question='is the C2-NAV.16 GOOD/BAD route split a genuine '
                 'near-tie in SmacPlanner2D\'s own cost objective?',
        config=dict(resolution=RESOLUTION, robot_radius=ROBOT_RADIUS,
                    inflation_radius=INFLATION_RADIUS,
                    cost_scaling_factor_global=CSF_GLOBAL,
                    cost_travel_multiplier=COST_TRAVEL_MULTIPLIER,
                    cost_norm_denominator_not_source_verified=COST_NORM),
        good_start=list(GOOD_START), good_start_yaw_rad=GOOD_START_YAW,
        bad_start=list(BAD_START), bad_start_yaw_rad=BAD_START_YAW,
        pose_delta_mm=POSE_DELTA_MM,
        self_test_pass=self_test(),
    )
    record['replay'] = replay()
    record['route_comparison'] = route_comparison()
    record['heading_sensitivity'] = heading_sensitivity()
    record['position_sweep'] = sweep_start_position()
    record['boundary_line'] = boundary_line()
    with open(out_path, 'w') as f:
        json.dump(record, f, indent=1, default=str)
    print(f'\nwrote {out_path}')
    return record


def visualize(out_path=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    out_path = out_path or os.path.join(
        REPO_ROOT, 'docs', 'images', 'c2nav17_routeselect.png')
    xs, ys, cost, dist = build_costmap()
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(15, 9), dpi=150,
                                  gridspec_kw={'width_ratios': [1.2, 1]})

    for axp in (ax, ax2):
        im = axp.contourf(xs, ys, cost, levels=np.linspace(0, 254, 40),
                          cmap=plt.get_cmap('inferno_r'), extend='neither')
        for b in list(BOXES) + list(EXTRA_BOXES):
            x0, x1, y0, y1 = rect(b)
            if x1 < GRID_X[0] or x0 > GRID_X[1] or y1 < GRID_Y[0] or y0 > GRID_Y[1]:
                continue
            axp.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                                    facecolor='none', edgecolor='cyan',
                                    linewidth=1.0, zorder=5))
        axp.add_patch(Circle(GOAL_SHIFTED, 0.25, fill=False, edgecolor='blue',
                             linewidth=1.3, linestyle='dashed', zorder=6))
        axp.plot(*GOAL_SHIFTED, marker='X', color='blue', markersize=10,
                 zorder=9, markeredgecolor='white')

    # left: whole region + both synthetic routes + real captured routes
    r_good = astar_2d(cost, xs, ys, GOOD_START, GOAL_SHIFTED)
    r_bad = astar_2d(cost, xs, ys, BAD_START, GOAL_SHIFTED)
    if r_good['found']:
        ax.plot([p[0] for p in r_good['path']], [p[1] for p in r_good['path']],
                color='lime', linewidth=2.2, zorder=10,
                label='synthetic route from GOOD_START')
    if r_bad['found']:
        ax.plot([p[0] for p in r_bad['path']], [p[1] for p in r_bad['path']],
                color='red', linewidth=2.2, zorder=10,
                label='synthetic route from BAD_START')
    ax.plot(*GOOD_START, marker='o', color='lime', markersize=9, zorder=11,
            markeredgecolor='black', label='GOOD_START (real, t0+9.0s)')
    ax.plot(*BAD_START, marker='o', color='red', markersize=9, zorder=11,
            markeredgecolor='black', label='BAD_START (real, t0+9.0s)')
    ax.set_xlim(*GRID_X)
    ax.set_ylim(*GRID_Y)
    ax.set_aspect('equal')
    ax.set_title('C2-NAV.17: reconstructed global costmap + synthetic '
                 'SmacPlanner2D replay', fontsize=9)
    ax.legend(loc='lower left', fontsize=7, framealpha=0.9)

    # right: zoom on the sweep region with SAFE/SW classification
    sw = sweep_start_position(half=0.15, step=0.01)
    for row in sw['rows']:
        c = 'red' if row['sw_column'] else 'lime'
        ax2.plot(row['x'], row['y'], marker='s', color=c, markersize=3,
                 alpha=0.7, zorder=8)
    ax2.plot(*GOOD_START, marker='*', color='white', markersize=16,
             markeredgecolor='black', zorder=11)
    ax2.plot(*BAD_START, marker='*', color='black', markersize=16,
             markeredgecolor='white', zorder=11)
    zx = (min(GOOD_START[0], BAD_START[0]) - 0.3,
          max(GOOD_START[0], BAD_START[0]) + 0.3)
    zy = (min(GOOD_START[1], BAD_START[1]) - 0.3,
          max(GOOD_START[1], BAD_START[1]) + 0.3)
    ax2.set_xlim(*zx)
    ax2.set_ylim(*zy)
    ax2.set_aspect('equal')
    ax2.set_title('start-position sweep: green=SAFE route, red=SW route\n'
                  '(white * = GOOD_START, black * = BAD_START)', fontsize=9)
    fig.suptitle('C2-NAV.17: route-selection sensitivity to start pose',
                 fontsize=11)
    fig.savefig(out_path, bbox_inches='tight')
    print(f'wrote {out_path}')


def all_(argv):
    ok = self_test()
    replay()
    route_comparison()
    heading_sensitivity()
    sweep_start_position()
    boundary_line()
    return 0 if ok else 1


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if cmd == 'selftest':
        return 0 if self_test() else 1
    if cmd == 'replay':
        replay()
        return 0
    if cmd == 'compare':
        route_comparison()
        return 0
    if cmd == 'sweep':
        sweep_start_position()
        return 0
    if cmd == 'heading':
        heading_sensitivity()
        return 0
    if cmd == 'boundary':
        boundary_line()
        return 0
    if cmd == 'dump':
        out = sys.argv[2] if len(sys.argv) > 2 else \
            os.path.join(HERE, 'c2nav17_bench.json')
        dump(out)
        return 0
    if cmd == 'viz':
        visualize()
        return 0
    if cmd == 'all':
        return all_(sys.argv[2:])
    print(f'unknown command: {cmd}')
    return 2


if __name__ == '__main__':
    sys.exit(main())
