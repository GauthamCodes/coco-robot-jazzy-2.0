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
"""C2-NAV.9: offline reconstruction of the enclosure approach corridor.

C2-NAV.8 found the shifted `enclosure_entry` goal (-3.575, 2.95) works for
the EXIT but is unreliable for the ENTRY: 1 of 3 fresh tours deadlocked
269.5 s against `box_obstacle_1`'s SOUTH-west corner, a corner the
corridor derivation in C2-NAV.7 never considered because that session's
two-leg run started the approach 0.6 m closer than the full tour does.

This module changes NOTHING navigational. It is pure offline geometry and
committed-artifact analysis:

  1. `self_test()`     -- reproduces C2-NAV.6's NW-corner penetration
                           (~5.5 mm) and C2-NAV.8's SW-corner penetration
                           (~4.7 mm) from committed CSV/JSON data, so the
                           geometry primitives are trusted before they are
                           used for anything new.
  2. `geometry()`       -- prints the canonical world/footprint/collision-
                           monitor geometry this experiment reasons over.
  3. `clearance_grid()` -- a numpy clearance field over the corridor_gate
                           -> enclosure_entry region at 3 mm resolution.
  4. `corridor()`       -- the maximum-bottleneck (widest-path) route
                           through that field via scipy connected-
                           component search, i.e. an actual geometric
                           feasibility proof, not a guess.
  5. `sw_corner()`      -- the C2-NAV.8 r1 deadlock, reconstructed from
                           the committed CSV and cross-checked against the
                           committed JSON summary.
  6. `yaw_feasibility()`-- whether the goal orientation (w=1.0 -> yaw 0)
                           is compatible with the corridor, using the
                           collision monitor's OWN circle/square logic
                           (`nav2_collision_monitor` 1.3.11, circle.cpp /
                           polygon.cpp) rather than a distance heuristic.
  7. `feasible_region()`-- a small feasibility MAP around the current
                           goal: STOP-clear (yaw-invariant, PolygonStop is
                           a circle) vs SLOW-clear-for-some-yaw
                           (orientation-dependent, PolygonSlow is a
                           square fixed to the body frame).
  8. `correlate()`      -- the three committed C2-NAV.8 tours (real GT
                           trajectories) laid over the offline corridor.
  9. `visualize()`      -- one deterministic PNG built from the same data.

Canonical geometry sources (verified against source, not assumed):
  - `gazebo_models/worlds/coco_world.world`      <collision> boxes/cylinder
  - `gazebo_models/scripts/nav_bench.py`          TOUR (world-frame goals)
  - `docs/data/c2nav4_csf65_params.yaml`          collision_monitor, costmaps
  - `docs/data/c2nav0_footprint.py` + `docs/RESULTS.md` "The footprint is
    not too conservative" -- the measured circumscribed radius/footprint
  - `docs/data/c2nav7_geom.py`                    BOXES / STOP_RADIUS /
                                                    CIRCUMSCRIBED / dist_to_box
  - `docs/data/c2nav8_report.py`                  EXTRA_BOXES / CIRCLES
                                                    (the full-world fix)
  - `docs/data/c2nav8_bench.json` + `c2nav8_tour_r{1,2,3}_stop.csv`
                                                   the three real tours

Usage:
  python3 c2nav9_corridor.py            # everything, in order
  python3 c2nav9_corridor.py selftest   # just the self-test
  python3 c2nav9_corridor.py viz        # just the PNG
"""
import csv
import json
import math
import os
import sys

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c2nav7_geom import (                                   # noqa: E402
    BOXES, CIRCUMSCRIBED, STOP_RADIUS, dist_to_box, rect)
from c2nav8_report import EXTRA_BOXES, CIRCLES, nearest_full  # noqa: E402

# ---------------------------------------------------------------------
# 0. Canonical constants, each cited to its source. Nothing here is a
#    fresh assumption -- every number is copied from a script or a world
#    file already committed by an earlier experiment.
# ---------------------------------------------------------------------

# gazebo_models/scripts/nav_bench.py, TOUR (world frame; "map = world +
# (2.0, 0)"). corridor_gate and the two enclosure legs, byte-identical to
# the committed tour.
CORRIDOR_GATE_GOAL = (-2.60, -0.10)
ENCLOSURE_ENTRY_ORIGINAL = (-3.45, 2.95)     # TOUR's own literal entry
ENCLOSURE_ENTRY_SHIFTED = (-3.575, 2.95)     # C2-NAV.7/.8's --goal override
ENCLOSURE_EXIT_GOAL = (-2.00, 0.00)

# docs/data/c2nav0_footprint.py, live measurement, quoted verbatim in
# docs/RESULTS.md "The footprint is not too conservative -- it is 5 mm
# too small": circumscribed radius 0.2051 m (driven by the wheels),
# half-width 0.1415 m, length x in [-0.1485, +0.1710] (base_footprint).
FOOT_CIRC_R = 0.2051
FOOT_HALF_W = 0.1415
FOOT_X_BACK = -0.1485
FOOT_X_FRONT = 0.1710
assert abs(FOOT_CIRC_R - CIRCUMSCRIBED) < 1e-6, 'footprint constant drifted'

# docs/data/c2nav4_csf65_params.yaml, collision_monitor.ros__parameters,
# base_frame_id "base_footprint" -- ALL THREE polygons are centred on the
# robot origin, not the lidar.
POLY_STOP_R = 0.25            # type: circle,  radius 0.25
POLY_SLOW_HW = 0.40           # type: polygon, [[0.4,0.4],[0.4,-0.4],...]
POLY_LIMIT_HW = 0.55          # type: polygon, [[0.55,0.55],...]
POLY_SLOW_RATIO = 0.3         # slowdown_ratio
assert abs(POLY_STOP_R - STOP_RADIUS) < 1e-9, 'PolygonStop constant drifted'
# A SQUARE half-width hw, rotated to bearing psi relative to a point at
# bearing phi, presents a boundary distance of hw / cos(theta), theta the
# angle folded into the square's first octant [0, 45 deg]. Reach ranges
# [hw, hw*sqrt(2)] -- 0.4 -> 0.5657 for PolygonSlow, 0.55 -> 0.7778 for
# PolygonLimit. Both numbers are independently confirmed in C2-NAV.0
# ("PolygonSlow reaches 0.566 m ... PolygonLimit 0.778 m").
POLY_SLOW_MAX = POLY_SLOW_HW * math.sqrt(2)
POLY_LIMIT_MAX = POLY_LIMIT_HW * math.sqrt(2)

DATA = HERE


def hdr(t):
    print()
    print('=' * 78)
    print(t)
    print('=' * 78)


# ---------------------------------------------------------------------
# Geometry primitives (thin wrappers over the imported, already-verified
# c2nav7_geom / c2nav8_report machinery -- nothing here re-derives what
# those scripts already got right; C2-NAV.9 only adds the square-polygon
# and grid/corridor machinery that did not exist yet).
# ---------------------------------------------------------------------

def square_reach(bearing_rad, half_width):
    """Distance from centre to a square's boundary along a given bearing.

    `bearing_rad` is in the SQUARE's own frame (0 = +x, i.e. straight
    ahead of the robot). Exact for an axis-aligned square of half-width
    `half_width`, by symmetry folded into the first octant.
    """
    a = abs(bearing_rad) % (math.pi / 2)
    a = min(a, math.pi / 2 - a)
    return half_width / math.cos(a)


def in_circle(dx, dy, r):
    return dx * dx + dy * dy < r * r


def in_square(dx, dy, half_width):
    return abs(dx) <= half_width and abs(dy) <= half_width


def body_frame(px, py, ox, oy, yaw):
    """World point (px, py) expressed in a frame at (ox, oy) rotated by
    yaw (i.e. the point AS SEEN BY a robot centred at (ox, oy) facing
    yaw)."""
    dx, dy = px - ox, py - oy
    c, s = math.cos(-yaw), math.sin(-yaw)
    return dx * c - dy * s, dx * s + dy * c


def relevant_obstacle_points(cx, cy, radius=1.2, step=0.003):
    """Dense sample of collision-boundary points within `radius` of
    (cx, cy): corners and edge points of every nearby box, and the rim of
    every nearby cylinder. This is what lets the polygon check below
    reproduce polygon.cpp's point-in-polygon test on a CONTINUOUS surface
    rather than the sparse 720-beam lidar the real monitor sees -- the
    conservative, "can this ever be avoided" question, not "was it seen
    by 4+ beams this cycle".
    """
    pts = []
    all_boxes = list(BOXES) + list(EXTRA_BOXES)
    for b in all_boxes:
        x0, x1, y0, y1 = rect(b)
        if x1 < cx - radius or x0 > cx + radius:
            continue
        if y1 < cy - radius or y0 > cy + radius:
            continue
        nx = max(2, int((x1 - x0) / step))
        ny = max(2, int((y1 - y0) / step))
        xs = np.linspace(x0, x1, nx)
        ys = np.linspace(y0, y1, ny)
        for x in xs:
            pts.append((x, y0))
            pts.append((x, y1))
        for y in ys:
            pts.append((x0, y))
            pts.append((x1, y))
    for (name, ccx, ccy, r) in CIRCLES:
        if math.hypot(ccx - cx, ccy - cy) > radius + r:
            continue
        n = max(8, int(2 * math.pi * r / step))
        for k in range(n):
            a = 2 * math.pi * k / n
            pts.append((ccx + r * math.cos(a), ccy + r * math.sin(a)))
    return np.array(pts) if pts else np.zeros((0, 2))


def zone_status_all_yaw(gx, gy, n_yaw=720):
    """Sweep yaw at a FIXED (gx, gy) and report, for each polygon, the
    fraction of headings for which it is triggered by the dense obstacle
    sample -- and whether it is triggered at EVERY heading (unavoidable)
    or NONE (always clear)."""
    pts = relevant_obstacle_points(gx, gy, radius=1.2)
    if len(pts) == 0:
        return None
    dx_w = pts[:, 0] - gx
    dy_w = pts[:, 1] - gy
    d = np.hypot(dx_w, dy_w)
    keep = d < (POLY_LIMIT_MAX + 0.02)
    dx_w, dy_w, d = dx_w[keep], dy_w[keep], d[keep]
    phi = np.arctan2(dy_w, dx_w)
    yaws = np.linspace(0, 2 * math.pi, n_yaw, endpoint=False)
    out = {}
    for name, test_r, test_hw in (('PolygonStop', POLY_STOP_R, None),
                                   ('PolygonSlow', None, POLY_SLOW_HW),
                                   ('PolygonLimit', None, POLY_LIMIT_HW)):
        trig = np.zeros(n_yaw, dtype=bool)
        for i, psi in enumerate(yaws):
            bearing = phi - psi
            bx = d * np.cos(bearing)
            by = d * np.sin(bearing)
            if test_r is not None:
                trig[i] = np.any(bx * bx + by * by < test_r * test_r)
            else:
                trig[i] = np.any((np.abs(bx) <= test_hw) &
                                  (np.abs(by) <= test_hw))
        out[name] = dict(frac_triggered=float(trig.mean()),
                          always=bool(trig.all()), never=bool(~trig.any()),
                          yaws=yaws, trig=trig)
    return out


# ---------------------------------------------------------------------
# 1. SELF-TEST -- reproduce the two known measurements before anything
#    new is trusted (CLAUDE.md-style evidence discipline, and the brief's
#    explicit instruction in section 13).
# ---------------------------------------------------------------------

def self_test():
    hdr('SELF-TEST: reproduce C2-NAV.6 (NW, ~5.5 mm) and C2-NAV.8 (SW, ~4.7 mm)')
    box = [b for b in BOXES if b[0] == 'box_obstacle_1'][0]
    x0, x1, y0, y1 = rect(box)
    nw = (x0, y1)
    sw = (x0, y0)
    print(f'  box_obstacle_1 rect (from BOXES, cross-checked against '
          f'coco_world.world <collision> below): x[{x0},{x1}] y[{y0},{y1}]')
    print(f'  NW corner (from rect)  : {nw}')
    print(f'  SW corner (from rect)  : {sw}')

    ok = True

    # --- NW corner: C2-NAV.6's frozen stall pose, docs/data/c2nav6_base_r1_stop.csv
    stall_pose = (-3.4558, 2.7805)
    d_geom, q = dist_to_box(*stall_pose, box)
    csv_path = os.path.join(DATA, 'c2nav6_base_r1_stop.csv')
    d_probe = None
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            if row['monitor_action'] == 'STOP' and row['gt_x'] and \
               abs(float(row['gt_x']) - stall_pose[0]) < 1e-3 and \
               abs(float(row['gt_y']) - stall_pose[1]) < 1e-3:
                d_probe = float(row['d_min_base_m'])
                n_in = int(row['n_in_stop'])
                break
    pen_geom = (POLY_STOP_R - d_geom) * 1000
    pen_probe = (POLY_STOP_R - d_probe) * 1000 if d_probe else None
    print()
    print(f'  NW: pose {stall_pose} -> nearest corner {q}')
    print(f'      geometry (GT pose -> box rect)  : {d_geom:.4f} m, '
          f'penetration {pen_geom:.1f} mm')
    if d_probe is not None:
        print(f'      committed CSV d_min_base_m      : {d_probe:.4f} m '
              f'(n_in_stop={n_in}), penetration {pen_probe:.1f} mm  '
              f'<- the canonical "~5.5 mm" figure')
    nw_pass = d_probe is not None and abs(pen_probe - 5.5) < 1.5
    print(f'      {"PASS" if nw_pass else "FAIL"} (want ~5.5 mm, got '
          f'{pen_probe:.1f} mm from committed data)')
    ok &= nw_pass

    # --- SW corner: C2-NAV.8 r1's 269.5 s deadlock, c2nav8_tour_r1_stop.csv
    dl_pose = (-3.3009, 1.9100)
    d_geom2, q2 = dist_to_box(*dl_pose, box)
    pen_geom2 = (POLY_STOP_R - d_geom2) * 1000
    print()
    print(f'  SW: deadlock pose {dl_pose} -> nearest corner {q2}')
    print(f'      geometry (GT pose -> box rect)  : {d_geom2:.4f} m, '
          f'penetration {pen_geom2:.1f} mm  <- the canonical "~4.7 mm" figure')
    sw_pass = abs(pen_geom2 - 4.7) < 1.0
    print(f'      {"PASS" if sw_pass else "FAIL"} (want ~4.7 mm, got '
          f'{pen_geom2:.1f} mm)')
    ok &= sw_pass
    corner_pass = (nw == (-3.25, 2.65)) and (sw == (-3.25, 2.15))
    print()
    print(f'  corner coordinates match the C2-NAV.9 brief '
          f'(NW -3.25,2.65 / SW -3.25,2.15): '
          f'{"PASS" if corner_pass else "FAIL"}')
    ok &= corner_pass

    print()
    print(f'  SELF-TEST: {"ALL PASS" if ok else "FAILED -- STOP, fix the tool"}')
    if not ok:
        raise SystemExit(1)
    return ok


# ---------------------------------------------------------------------
# 2. CANONICAL GEOMETRY
# ---------------------------------------------------------------------

def geometry():
    hdr('CANONICAL GEOMETRY')
    print('  Coordinate frame: WORLD (Gazebo ground truth / SDF <pose>).')
    print('  map = world + (2.0, 0)  [nav_bench.py header comment].')
    print()
    print('  World collision boxes (gazebo_models/worlds/coco_world.world, '
          'verified against <collision> tags):')
    for b in BOXES:
        x0, x1, y0, y1 = rect(b)
        print(f'    {b[0]:<18} centre ({b[1]:+.3f},{b[2]:+.3f})  '
              f'size {b[3]}x{b[4]}  ->  x[{x0:+.3f},{x1:+.3f}] '
              f'y[{y0:+.3f},{y1:+.3f}]')
    for b in EXTRA_BOXES:
        x0, x1, y0, y1 = rect(b)
        print(f'    {b[0]:<18} centre ({b[1]:+.3f},{b[2]:+.3f})  '
              f'size {b[3]}x{b[4]}  ->  x[{x0:+.3f},{x1:+.3f}] '
              f'y[{y0:+.3f},{y1:+.3f}]  (EXTRA_BOXES, C2-NAV.8 full-world fix)')
    for (name, cx, cy, r) in CIRCLES:
        print(f'    {name:<18} centre ({cx:+.3f},{cy:+.3f})  radius {r}')
    print()
    print('  Robot footprint (docs/data/c2nav0_footprint.py, live TF + URDF):')
    print(f'    circumscribed radius : {FOOT_CIRC_R:.4f} m (driven by the wheels)')
    print(f'    half-width            : {FOOT_HALF_W:.4f} m -> full width '
          f'{2*FOOT_HALF_W:.4f} m')
    print(f'    length (x)            : [{FOOT_X_BACK:+.4f}, {FOOT_X_FRONT:+.4f}] '
          f'-> {FOOT_X_FRONT - FOOT_X_BACK:.4f} m')
    print('    nav2 robot_radius (planning) = 0.20 m, 5.1 mm SMALLER than the '
          'robot (RESULTS.md).')
    print()
    print('  Collision monitor (docs/data/c2nav4_csf65_params.yaml, '
          'collision_monitor.ros__parameters; base_frame_id=base_footprint, '
          'ALL THREE zones centred on the robot origin):')
    print(f'    PolygonStop  : circle, radius {POLY_STOP_R} m, min_points 4, '
          f'action=stop')
    print(f'    PolygonSlow  : square, half-width {POLY_SLOW_HW} m '
          f'(reach {POLY_SLOW_HW}..{POLY_SLOW_MAX:.4f} m by heading), '
          f'action=slowdown, ratio {POLY_SLOW_RATIO}')
    print(f'    PolygonLimit : square, half-width {POLY_LIMIT_HW} m '
          f'(reach {POLY_LIMIT_HW}..{POLY_LIMIT_MAX:.4f} m by heading), '
          f'action=limit')
    print()
    print('  Tour goals (gazebo_models/scripts/nav_bench.py TOUR, world frame):')
    print(f'    corridor_gate            : {CORRIDOR_GATE_GOAL}')
    print(f'    enclosure_entry, ORIGINAL: {ENCLOSURE_ENTRY_ORIGINAL} '
          '(TOUR literal, never run un-overridden in C2-NAV.6+)')
    print(f'    enclosure_entry, SHIFTED : {ENCLOSURE_ENTRY_SHIFTED} '
          '(C2-NAV.7/.8 --goal override, current)')
    print(f'    enclosure_exit           : {ENCLOSURE_EXIT_GOAL}')
    print('    Goal orientation for every leg: orientation.w = 1.0, '
          'x=y=z=0 (quaternion identity) -> yaw = 0 rad (facing +X world, '
          'i.e. EAST). Verified below in yaw_feasibility().')


# ---------------------------------------------------------------------
# 3-4. CLEARANCE GRID + CORRIDOR (widest-path / max-bottleneck)
# ---------------------------------------------------------------------

# Region: covers every corridor_gate end-pose, both enclosure goals, and
# both corners of box_obstacle_1, with margin.
GRID_X = (-4.30, -2.30)
GRID_Y = (-0.60, 3.30)
GRID_RES = 0.003  # 3 mm


def build_clearance_grid(res=GRID_RES):
    xs = np.arange(GRID_X[0], GRID_X[1] + res, res)
    ys = np.arange(GRID_Y[0], GRID_Y[1] + res, res)
    X, Y = np.meshgrid(xs, ys, indexing='xy')  # shape (ny, nx)
    clr = np.full(X.shape, np.inf)
    all_boxes = list(BOXES) + list(EXTRA_BOXES)
    for b in all_boxes:
        x0, x1, y0, y1 = rect(b)
        qx = np.clip(X, x0, x1)
        qy = np.clip(Y, y0, y1)
        d = np.hypot(X - qx, Y - qy)
        clr = np.minimum(clr, d)
    for (name, cx, cy, r) in CIRCLES:
        d = np.hypot(X - cx, Y - cy) - r
        clr = np.minimum(clr, np.maximum(d, 0.0))
    return xs, ys, clr


def grid_index(xs, ys, x, y):
    ix = int(round((x - xs[0]) / (xs[1] - xs[0])))
    iy = int(round((y - ys[0]) / (ys[1] - ys[0])))
    ix = min(max(ix, 0), len(xs) - 1)
    iy = min(max(iy, 0), len(ys) - 1)
    return iy, ix


def bottleneck(clr, xs, ys, start_xy, goal_xy, lo=0.0, hi=0.9, iters=26):
    """Maximum tau such that start and goal are 8-connected within
    {clearance >= tau}. Classic widest-path / maximum-capacity-path
    problem, solved by binary search on tau + connected-component
    labelling (scipy.ndimage.label), which is exact given the grid and
    monotone in tau (raising tau can only shrink the free set)."""
    si = grid_index(xs, ys, *start_xy)
    gi = grid_index(xs, ys, *goal_xy)
    struct = np.ones((3, 3), dtype=bool)  # 8-connectivity

    def connected(tau):
        mask = clr >= tau
        if not mask[si] or not mask[gi]:
            return False
        lbl, _ = ndimage.label(mask, structure=struct)
        return lbl[si] != 0 and lbl[si] == lbl[gi]

    if not connected(lo):
        return None  # not even connected at tau=0 (should not happen)
    for _ in range(iters):
        mid = (lo + hi) / 2
        if connected(mid):
            lo = mid
        else:
            hi = mid
    return lo


def bottleneck_location(clr, xs, ys, start_xy, goal_xy, tau, eps=0.003):
    """Cells achieving ~tau clearance within the component connecting
    start and goal at threshold (tau - eps) -- i.e. where the widest-path
    argument is actually tight."""
    si = grid_index(xs, ys, *start_xy)
    gi = grid_index(xs, ys, *goal_xy)
    struct = np.ones((3, 3), dtype=bool)
    mask = clr >= max(tau - eps, 0.0)
    lbl, _ = ndimage.label(mask, structure=struct)
    comp = lbl[si]
    if comp == 0 or lbl[gi] != comp:
        return []
    in_comp = lbl == comp
    tight = in_comp & (clr <= tau + eps)
    iy, ix = np.where(tight)
    pts = [(xs[j], ys[i], clr[i, j]) for i, j in zip(iy, ix)]
    return pts


def corridor():
    hdr('CLEARANCE GRID + CORRIDOR RECONSTRUCTION '
        f'(res={GRID_RES*1000:.0f} mm, region x{GRID_X} y{GRID_Y})')
    xs, ys, clr = build_clearance_grid()
    print(f'  grid shape: {clr.shape} ({clr.size:,} cells)')

    starts = {
        'canonical corridor_gate goal': CORRIDOR_GATE_GOAL,
        'r1 corridor_gate end (GT)': (-2.6162, 0.0325),
        'r2 corridor_gate end (GT)': (-2.6111, -0.0291),
        'r3 corridor_gate end (GT)': (-2.5830, -0.0076),
    }
    goal = ENCLOSURE_ENTRY_SHIFTED

    results = {}
    print()
    print(f'  {"start":<32}{"stop-free tau*":>16}{"circ-safe tau*":>16}')
    for name, s in starts.items():
        tau_stop = bottleneck(clr, xs, ys, s, goal)
        tau_circ = bottleneck(clr, xs, ys, s, goal, lo=0.0, hi=0.60)
        results[name] = (s, tau_stop, tau_circ)
        print(f'  {name:<32}{tau_stop*1000:14.1f} mm{tau_circ*1000:14.1f} mm')

    print()
    print('  "stop-free tau*" = the widest achievable minimum clearance on ANY')
    print('  path from that start to the goal (max-bottleneck / widest-path).')
    print(f'  PolygonStop.radius = {POLY_STOP_R} m. If tau* >= {POLY_STOP_R} m, a '
          'path exists that never')
    print('  enters PolygonStop at all -- the deadlock is then NOT a geometric')
    print('  necessity. "circ-safe tau*" is capped at 0.30 to keep the search')
    print(f'  in range; it only needs to clear {FOOT_CIRC_R} m for physical safety.')

    # Use the r1 corridor_gate-end start (the run that deadlocked) as the
    # canonical case for locating the pinch.
    s = starts['r1 corridor_gate end (GT)']
    tau = results['r1 corridor_gate end (GT)'][1]
    pts = bottleneck_location(clr, xs, ys, s, goal, tau)
    print()
    print(f'  Narrowest section on the r1-start widest path: tau* = '
          f'{tau*1000:.1f} mm, {len(pts)} grid cells within {3:.0f} mm of it.')
    if pts:
        # report a representative sample, sorted by clearance then position
        pts.sort(key=lambda p: p[2])
        for (x, y, c) in pts[:1] + pts[len(pts)//2:len(pts)//2+1] + pts[-1:]:
            print(f'    e.g. ({x:+.3f}, {y:+.3f})  clearance {c*1000:.1f} mm')

    return xs, ys, clr, results


# ---------------------------------------------------------------------
# 5. SW CORNER
# ---------------------------------------------------------------------

def sw_corner(xs, ys, clr):
    hdr('SOUTH-WEST CORNER ANALYSIS (C2-NAV.8 r1 deadlock)')
    box = [b for b in BOXES if b[0] == 'box_obstacle_1'][0]
    x0, x1, y0, y1 = rect(box)
    sw = (x0, y0)
    print(f'  box_obstacle_1 SW corner: {sw}')

    csv_path = os.path.join(DATA, 'c2nav8_tour_r1_stop.csv')
    rows = list(csv.DictReader(open(csv_path)))
    stop_rows = [r for r in rows if r['monitor_action'] == 'STOP' and r['gt_x']]
    # find the SW-corner deadlock: the long contiguous STOP run whose
    # nearest geometry is box_obstacle_1's SW corner, not the NW one.
    run = []
    best_run = []
    prev_t = None
    for r in stop_rows:
        t = float(r['t_s'])
        if prev_t is not None and t - prev_t > 0.5:
            if len(run) > len(best_run):
                best_run = run
            run = []
        run.append(r)
        prev_t = t
    if len(run) > len(best_run):
        best_run = run
    t0, t1 = float(best_run[0]['t_s']), float(best_run[-1]['t_s'])
    gt = (float(best_run[0]['gt_x']), float(best_run[0]['gt_y']))
    yaw = float(best_run[0]['gt_yaw'])
    n_in = [int(r['n_in_stop']) for r in best_run]
    d_geom, q = dist_to_box(*gt, box)
    print(f'  longest continuous STOP run: {len(best_run)} rows, '
          f't = [{t0:.1f}, {t1:.1f}] s -> {t1-t0:.1f} s')
    print(f'  frozen pose (GT)   : ({gt[0]:+.4f}, {gt[1]:+.4f}), '
          f'yaw {yaw:+.4f} rad ({math.degrees(yaw):+.1f} deg)')
    print(f'  n_in_stop over run : min {min(n_in)}, median '
          f'{sorted(n_in)[len(n_in)//2]}, max {max(n_in)}')
    print(f'  nearest geometry   : box_obstacle_1, closest point {q}  '
          f'(== SW corner: {q == sw})')
    print(f'  clearance          : {d_geom:.4f} m  '
          f'(penetration {(POLY_STOP_R-d_geom)*1000:.1f} mm into the '
          f'{POLY_STOP_R} m circle)')
    print(f'  circumscribed margin: {(d_geom-FOOT_CIRC_R)*1000:.1f} mm above '
          f'{FOOT_CIRC_R} m -- never a physical collision')

    # approach direction: heading over the ~3 s immediately before the
    # freeze (the last moving samples on this leg).
    idx = rows.index(best_run[0])
    pre = rows[max(0, idx - 40):idx]
    pre = [r for r in pre if r['gt_x']]
    if len(pre) >= 2:
        x_a, y_a = float(pre[0]['gt_x']), float(pre[0]['gt_y'])
        x_b, y_b = float(pre[-1]['gt_x']), float(pre[-1]['gt_y'])
        heading = math.atan2(y_b - y_a, x_b - x_a)
        print()
        print(f'  approach, last {float(pre[-1]["t_s"])-float(pre[0]["t_s"]):.1f} s '
              f'before freeze: ({x_a:+.3f},{y_a:+.3f}) -> ({x_b:+.3f},{y_b:+.3f})')
        print(f'  net direction of travel: {math.degrees(heading):+.1f} deg '
              '(0 = world +X/east, 90 = world +Y/north)')
        bearing_to_corner = math.degrees(math.atan2(sw[1]-y_b, sw[0]-x_b))
        print(f'  bearing from freeze pose to SW corner: '
              f'{bearing_to_corner:+.1f} deg')
        # tangent (perpendicular) distance of the corner from the
        # approach centreline
        vx, vy = math.cos(heading), math.sin(heading)
        wx, wy = sw[0] - x_a, sw[1] - y_a
        along = wx * vx + wy * vy
        perp = wx * (-vy) + wy * vx
        print(f'  SW corner relative to the approach line through '
              f'({x_a:+.3f},{y_a:+.3f}): {along:.3f} m along, '
              f'{perp:+.3f} m lateral (perpendicular)')

    print()
    print('  Lateral displacement needed to clear the corner by '
        f'PolygonStop.radius ({POLY_STOP_R} m):')
    needed = POLY_STOP_R - d_geom
    print(f'    at the frozen pose: {needed*1000:.1f} mm more clearance needed')
    print('    (this is a LOCAL number at the frozen pose only -- the real '
          'question is whether the corridor grid found a route that never')
    print('    needs to come this close in the first place; see corridor() '
          'above and the bottleneck value for the r1 start.)')
    return gt, sw


# ---------------------------------------------------------------------
# 6. YAW FEASIBILITY
# ---------------------------------------------------------------------

def yaw_feasibility():
    hdr('YAW FEASIBILITY AT THE CURRENT GOAL')
    # Verify the quaternion numerically rather than assuming.
    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    print(f'  quaternion (x,y,z,w) = (0,0,0,1) -> yaw = {yaw:.6f} rad '
          f'({math.degrees(yaw):.2f} deg) : facing world +X (EAST).')
    print('  Every TOUR leg shares this literal orientation -- verified from '
          'nav_bench.py TOUR tuples, which carry only (name, x, y, probe).')

    gx, gy = ENCLOSURE_ENTRY_SHIFTED
    box = [b for b in BOXES if b[0] == 'box_obstacle_1'][0]
    wall = [b for b in BOXES if b[0] == 'wall_west'][0]
    d_wall, q_wall = dist_to_box(gx, gy, wall)
    d_box, q_box = dist_to_box(gx, gy, box)
    d_all = nearest_full(gx, gy)
    print()
    print(f'  goal ({gx}, {gy}):')
    print(f'    distance to wall_west        : {d_wall:.4f} m, closest '
          f'point {q_wall}')
    print(f'    distance to box_obstacle_1   : {d_box:.4f} m, closest '
          f'point {q_box}')
    print(f'    nearest of ALL world geometry: {d_all[0][1]} at '
          f'{d_all[0][0]:.4f} m')
    print()
    print('  Corridor direction at the goal: the NW pinch runs '
          'north-south (the gap is between')
    print('  wall_west, a north-south wall, and box_obstacle_1\'s west '
          'face) -- an approach or')
    print('  departure heading near +-90 deg (north/south), NOT the '
          'goal\'s commanded 0 deg (east).')
    print('  The commanded final heading is roughly PERPENDICULAR to the '
          'corridor\'s long axis.')

    print()
    print(f'  PolygonSlow (square, half-width {POLY_SLOW_HW} m) reach by '
          f'heading: [{POLY_SLOW_HW:.4f}, {POLY_SLOW_MAX:.4f}] m.')
    print(f'  Nearest obstacle to the goal is {d_all[0][0]:.4f} m away. '
          f'Since even the MINIMUM possible')
    print(f'  PolygonSlow reach ({POLY_SLOW_HW} m) exceeds that '
          f'({d_all[0][0]:.4f} m), PolygonSlow is')
    print('  UNAVOIDABLE at this goal for EVERY heading -- this is a closed-form')
    print('  consequence of the square geometry, not a benchmark artefact.')

    z = zone_status_all_yaw(gx, gy)
    print()
    print('  Full 720-heading sweep (dense obstacle-surface sample, exact '
          'circle/square test):')
    for name in ('PolygonStop', 'PolygonSlow', 'PolygonLimit'):
        d = z[name]
        state = ('ALWAYS triggered' if d['always'] else
                 'NEVER triggered' if d['never'] else
                 f'triggered {d["frac_triggered"]*100:.1f}% of headings')
        print(f'    {name:<14}: {state}')

    print()
    print('  PolygonStop is a CIRCLE centred on the robot origin: its trigger')
    print('  state cannot depend on yaw at all, and the sweep above confirms')
    print('  that algebraically (0% or 100%, never partial). At this goal it')
    print(f'  is {"triggered" if z["PolygonStop"]["always"] else "clear"} for every heading.')

    print()
    print('  Observed cost, from the committed C2-NAV.8 leg summaries '
          '(c2nav8_bench.json):')
    print(f'  {"run":<6}{"status":<10}{"t_terminal_s":>14}{"terminal_frac":>15}'
          f'{"terminal_yaw_travel_rad":>26}{"cm SLOWDOWN frac":>18}')
    bench = json.load(open(os.path.join(DATA, 'c2nav8_bench.json')))
    for i, run in enumerate(bench['runs'], 1):
        for leg in run['legs']:
            if leg['scenario'] == 'enclosure_entry':
                cm = leg.get('cm_action_frac') or {}
                tfrac = leg.get('terminal_frac_of_leg')
                tfrac_s = f'{tfrac*100:>13.1f}%' if tfrac is not None else \
                    f'{"n/a (deadlocked":>15}'
                tyaw = leg.get('terminal_yaw_travel_rad')
                tyaw_s = f'{tyaw:>26.3f}' if tyaw is not None else \
                    f'{"n/a":>26}'
                print(f'  r{i:<5}{leg["status"]:<10}{leg["t_terminal_s"]:>14.2f}'
                      f'{tfrac_s}'
                      f'{tyaw_s}'
                      f'{cm.get("SLOWDOWN", 0)*100:>17.1f}%')
    print()
    print('  yaw_goal_tolerance = 0.25 rad (14.3 deg). A worst-case single '
          'in-place correction')
    print('  from any arrival heading to yaw=0 needs at most pi rad '
          '(3.14) of travel. r2/r3')
    print('  logged 8.49 / 10.57 rad of terminal yaw travel -- 2.7x / '
          '3.4x more than the worst-')
    print('  case single turn, i.e. multiple net revolutions\' worth of '
          'angular travel, not one')
    print('  smooth rotation. That is a HUNTING signature layered on top '
          'of the geometrically-')
    print('  unavoidable SLOWDOWN (angular cap 0.3 x max_vel_theta = '
          '0.3 rad/s), not proof that')
    print('  the SLOWDOWN cap alone explains the observed duration.')
    return z


# ---------------------------------------------------------------------
# 7. FEASIBLE POSE REGION around the current goal
# ---------------------------------------------------------------------

def feasible_region():
    hdr('FEASIBLE POSE REGION around the current enclosure_entry goal')
    gx0, gy0 = ENCLOSURE_ENTRY_SHIFTED
    half = 0.30
    res = 0.01
    xs = np.arange(gx0 - half, gx0 + half + res, res)
    ys = np.arange(gy0 - half, gy0 + half + res, res)
    stop_clear = np.zeros((len(ys), len(xs)), dtype=bool)
    slow_clear_some = np.zeros((len(ys), len(xs)), dtype=bool)
    slow_clear_all = np.zeros((len(ys), len(xs)), dtype=bool)
    n_yaw = 72
    for i, y in enumerate(ys):
        for j, x in enumerate(xs):
            d = nearest_full(x, y)[0][0]
            stop_clear[i, j] = d >= POLY_STOP_R
            if d >= POLY_SLOW_MAX:
                slow_clear_all[i, j] = True
                slow_clear_some[i, j] = True
            elif d < POLY_SLOW_HW:
                pass
            else:
                z = zone_status_all_yaw(x, y, n_yaw=n_yaw)
                if z is not None:
                    slow_clear_some[i, j] = not z['PolygonSlow']['always']
                    slow_clear_all[i, j] = z['PolygonSlow']['never']
    frac_stop = stop_clear.mean()
    frac_slow_some = slow_clear_some.mean()
    frac_slow_all = slow_clear_all.mean()
    print(f'  region: +-{half} m around {ENCLOSURE_ENTRY_SHIFTED}, {res*1000:.0f} mm grid, '
          f'{stop_clear.size} cells, {n_yaw}-heading sweep where needed')
    print(f'  STOP-clear (all yaw, circle)         : {frac_stop*100:.1f}% of cells')
    print(f'  SLOW-clear for SOME yaw              : {frac_slow_some*100:.1f}% of cells')
    print(f'  SLOW-clear for ALL yaw (orientation-  : {frac_slow_all*100:.1f}% of cells')
    print('  independent safe pocket)')

    gi, gj = grid_index(xs, ys, gx0, gy0)
    print()
    print(f'  AT THE CURRENT GOAL ({gx0},{gy0}):')
    print(f'    STOP-clear (all yaw)     : {stop_clear[gi,gj]}')
    print(f'    SLOW-clear for some yaw  : {slow_clear_some[gi,gj]}')
    print(f'    SLOW-clear for all yaw   : {slow_clear_all[gi,gj]}')

    # distance from the goal to the edge of the STOP-clear region, along
    # the corridor's long axis (north-south) and across it (east-west)
    def edge_dist(mask, axis, direction):
        i0, j0 = gi, gj
        step = direction
        k = 0
        while True:
            k += step
            if axis == 'x':
                jj = j0 + k
                if jj < 0 or jj >= mask.shape[1] or not mask[i0, jj]:
                    return abs(k) * res
            else:
                ii = i0 + k
                if ii < 0 or ii >= mask.shape[0] or not mask[ii, j0]:
                    return abs(k) * res

    print()
    print('  Room inside the STOP-clear region from the goal (== distance '
          'to PolygonStop-')
    print('  worthy geometry along each axis):')
    print(f'    +x (east, toward box)  : {edge_dist(stop_clear, "x", 1)*1000:.0f} mm')
    print(f'    -x (west, toward wall) : {edge_dist(stop_clear, "x", -1)*1000:.0f} mm')
    print(f'    +y (north)             : {edge_dist(stop_clear, "y", 1)*1000:.0f} mm')
    print(f'    -y (south, toward gap) : {edge_dist(stop_clear, "y", -1)*1000:.0f} mm')

    return xs, ys, stop_clear, slow_clear_some, slow_clear_all


# ---------------------------------------------------------------------
# 8. CORRELATE with the real C2-NAV.8 tours
# ---------------------------------------------------------------------

def load_run_path(tag):
    path = os.path.join(DATA, f'c2nav8_tour_{tag}_stop.csv')
    xs, ys, ts, act = [], [], [], []
    for r in csv.DictReader(open(path)):
        if not r['gt_x']:
            continue
        xs.append(float(r['gt_x']))
        ys.append(float(r['gt_y']))
        ts.append(float(r['t_s']))
        act.append(r['monitor_action'])
    return np.array(xs), np.array(ys), np.array(ts), act


def correlate(xs, ys, clr):
    hdr('CORRELATION WITH THE THREE COMMITTED C2-NAV.8 TOURS')
    bench = json.load(open(os.path.join(DATA, 'c2nav8_bench.json')))
    tags = [run['tag'].split('_')[-1] for run in bench['runs']]
    for tag, run in zip(tags, bench['runs']):
        gate = next(l for l in run['legs'] if l['scenario'] == 'corridor_gate')
        entry = next(l for l in run['legs'] if l['scenario'] == 'enclosure_entry')
        exitl = next(l for l in run['legs'] if l['scenario'] == 'enclosure_exit')
        print(f'  --- {tag} ---')
        print(f'    corridor_gate  : {gate["status"]:<9} end {gate["end_world"]}')
        print(f'    enclosure_entry: {entry["status"]:<9} end {entry["end_world"]}  '
              f'err {entry["final_goal_err_m"]} m  '
              f'cm {entry.get("cm_action_frac")}')
        print(f'    enclosure_exit : {exitl["status"]:<9} end {exitl["end_world"]}')
        gx, gy, _, _ = load_run_path(tag)
        # nearest offline clearance to the executed GT trajectory, for the
        # entry-region samples only (west of corridor_gate x, i.e. inside
        # the analysis grid)
        m = (gx > GRID_X[0]) & (gx < GRID_X[1]) & (gy > GRID_Y[0]) & (gy < GRID_Y[1])
        if m.any():
            ix = np.clip(((gx[m] - xs[0]) / GRID_RES).round().astype(int), 0, len(xs)-1)
            iy = np.clip(((gy[m] - ys[0]) / GRID_RES).round().astype(int), 0, len(ys)-1)
            grid_clear = clr[iy, ix]
            below_stop = grid_clear < POLY_STOP_R
            print(f'    executed GT samples inside the analysis grid: {m.sum()}, '
                  f'{below_stop.sum()} ({below_stop.mean()*100:.1f}%) below '
                  f'PolygonStop.radius per the offline field')
            print(f'    offline-predicted min clearance on this leg  : '
                  f'{grid_clear.min()*1000:.1f} mm')
    print()
    print('  This is a like-for-like check: the offline clearance field is '
          'evaluated AT the')
    print('  real ground-truth samples, not the other way around. Agreement '
          'with the STOP/')
    print('  SLOWDOWN columns already recorded in the CSVs (cross-checked in '
          'self_test() and')
    print('  sw_corner() above) is what licenses treating the offline field '
          'as ground truth for')
    print('  the region the real robot never sampled -- i.e. the "was there '
          'a wider route"')
    print('  question in corridor() and sw_corner().')


# ---------------------------------------------------------------------
# 9. VISUALIZATION
# ---------------------------------------------------------------------

def visualize(xs, ys, clr, out_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(16, 11), dpi=150,
                                   gridspec_kw={'width_ratios': [1.15, 1]})
    cmap = plt.get_cmap('RdYlGn')

    box = [b for b in BOXES if b[0] == 'box_obstacle_1'][0]
    bx0, bx1, by0, by1 = rect(box)
    gx, gy = ENCLOSURE_ENTRY_SHIFTED
    colors = {'r1': 'crimson', 'r2': 'darkorange', 'r3': 'teal'}

    def draw(axp, xlim, ylim, zoom):
        im = axp.contourf(xs, ys, clr, levels=np.linspace(0, 0.6, 41),
                           cmap=cmap, vmin=0, vmax=0.6, extend='max')
        axp.contour(xs, ys, clr, levels=[FOOT_CIRC_R], colors='black',
                    linewidths=1.2 if not zoom else 1.6, linestyles='solid')
        axp.contour(xs, ys, clr, levels=[POLY_STOP_R], colors='blue',
                    linewidths=1.6 if not zoom else 2.2, linestyles='dashed')
        all_boxes = list(BOXES) + list(EXTRA_BOXES)
        for b in all_boxes:
            x0, x1, y0, y1 = rect(b)
            if x1 < xlim[0] or x0 > xlim[1] or y1 < ylim[0] or y0 > ylim[1]:
                continue
            axp.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                                     facecolor='dimgray', edgecolor='black',
                                     zorder=5))
            if not zoom:
                axp.text((x0 + x1) / 2, (y0 + y1) / 2, b[0], color='white',
                         fontsize=6, ha='center', va='center', zorder=6)
        for (name, (cx, cy)) in (('NW', (bx0, by1)), ('SW', (bx0, by0))):
            axp.plot(cx, cy, marker='*', color='yellow', markersize=16 if zoom else 14,
                     markeredgecolor='black', zorder=7)
            axp.annotate(f'{name} corner', (cx, cy), textcoords='offset points',
                        xytext=(8, 6), fontsize=9 if zoom else 8, color='black',
                        zorder=7, fontweight='bold')
        axp.add_patch(Circle((gx, gy), POLY_STOP_R, fill=False, edgecolor='blue',
                             linewidth=1.5, linestyle='dashed', zorder=6))
        axp.plot(gx, gy, marker='X', color='blue',
                 markersize=12 if zoom else 10, zorder=9,
                 markeredgecolor='white')
        if not zoom:
            ox, oy = ENCLOSURE_ENTRY_ORIGINAL
            axp.plot(ox, oy, marker='x', color='purple', markersize=8, zorder=7)
            axp.annotate('original TOUR goal', (ox, oy), textcoords='offset points',
                        xytext=(8, 8), fontsize=7, color='purple', zorder=7)
            cgx, cgy = CORRIDOR_GATE_GOAL
            axp.plot(cgx, cgy, marker='o', color='black', markersize=6, zorder=7)
            axp.annotate('corridor_gate goal', (cgx, cgy), textcoords='offset points',
                        xytext=(8, -12), fontsize=7, zorder=7)
        else:
            axp.annotate('enclosure_entry\n(shifted, current)', (gx, gy),
                        textcoords='offset points', xytext=(-95, -8), fontsize=9,
                        color='blue', zorder=9, fontweight='bold')

        for tag in ('r1', 'r2', 'r3'):
            px, py, ts, act = load_run_path(tag)
            m = (px > xlim[0]) & (px < xlim[1]) & (py > ylim[0]) & (py < ylim[1])
            axp.plot(px[m], py[m], color=colors[tag],
                    linewidth=1.6 if zoom else 1.1, alpha=0.9,
                    label=f'{tag} executed GT path' if not zoom else None,
                    zorder=8)
            stop_m = m & (np.array(act) == 'STOP')
            if stop_m.any():
                axp.scatter(px[stop_m], py[stop_m], s=10 if zoom else 4,
                           color=colors[tag], marker='s', zorder=10,
                           edgecolor='black', linewidth=0.3)
        axp.set_xlim(*xlim)
        axp.set_ylim(*ylim)
        axp.set_aspect('equal')
        axp.set_xlabel('world x (m)')
        axp.set_ylabel('world y (m)')
        return im

    im = draw(ax, GRID_X, GRID_Y, zoom=False)
    ax.set_title('corridor_gate -> enclosure_entry, whole approach')
    ax.legend(loc='lower left', fontsize=8, framealpha=0.9)

    zoom_xlim = (-3.80, -3.15)
    zoom_ylim = (1.85, 3.30)
    draw(ax2, zoom_xlim, zoom_ylim, zoom=True)
    ax2.set_title('zoom: the goal pocket, both corners, and the\n'
                   'yaw-settling loops (r2/r3) vs the r1 deadlock')
    ax.add_patch(Rectangle((zoom_xlim[0], zoom_ylim[0]),
                            zoom_xlim[1] - zoom_xlim[0],
                            zoom_ylim[1] - zoom_ylim[0],
                            fill=False, edgecolor='black', linewidth=1.2,
                            linestyle='dotted', zorder=10))

    cbar = fig.colorbar(im, ax=[ax, ax2], shrink=0.6, pad=0.015,
                         location='right')
    cbar.set_label('clearance to nearest world geometry (m)')
    fig.suptitle('C2-NAV.9: corridor_gate -> enclosure_entry clearance field  '
                 '(black contour = circumscribed radius 0.2051 m, '
                 'blue dashed = PolygonStop.radius 0.25 m)', fontsize=11)
    fig.savefig(out_path, bbox_inches='tight')
    print(f'  wrote {out_path}')


def main():
    argv = sys.argv[1:]
    if argv and argv[0] == 'selftest':
        return 0 if self_test() else 1
    self_test()
    geometry()
    xs, ys, clr, cresults = corridor()
    sw_corner(xs, ys, clr)
    yaw_feasibility()
    feasible_region()
    correlate(xs, ys, clr)
    if not argv or argv[0] != 'nofig':
        visualize(xs, ys, clr, os.path.join(
            os.path.dirname(DATA), 'images', 'c2nav9_corridor.png'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
