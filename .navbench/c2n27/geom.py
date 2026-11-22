#!/usr/bin/env python3
"""C2-NAV.27 -- offline costmap geometry for the "Start occupied" refusals.

OFFLINE ONLY. Reads the frozen map + frozen traces. Changes nothing, runs
no ROS, and starts no simulator.

WHAT THE PLANNER ACTUALLY TESTS
-------------------------------
`planner_server.GridBased` is `nav2_smac_planner::SmacPlanner2D`
(docs/data/c2nav11_ntp_params.yaml). Its start check is a SINGLE-CELL
test, not a footprint-polygon sweep:

    AStarAlgorithm<Node2D>::createPath()
        if (!_start->isNodeValid(traverse_unknown, collision_checker))
            throw nav2_core::StartOccupied("Start occupied");
    Node2D::isNodeValid  -> GridCollisionChecker::inCollision(index, ...)
        return cost >= INSCRIBED_INFLATED_OBSTACLE            // 253

and nav2_costmap_2d::InflationLayer assigns

    d == 0                      -> LETHAL_OBSTACLE       254
    d <= inscribed_radius       -> INSCRIBED_INFLATED    253
    otherwise                   -> 252 * exp(-k*(d - inscribed_radius))

with `inscribed_radius` = robot_radius = 0.20 m for a circular footprint.

    ==> "Start occupied" fires iff the distance from the START CELL
        CENTRE to the nearest LETHAL cell centre is <= 0.20 m.

DERIVED, from headers + params, not read from the installed .cpp (the
Nav2 sources are not installed here; only headers are). Labelled as such
everywhere it is used.

WHAT THIS RECONSTRUCTION CAN AND CANNOT SEE
-------------------------------------------
It rebuilds the STATIC layer only, from gazebo_models/maps/coco_world.
The live global costmap is static + voxel/obstacle layer + inflation, so
live lethal cells from the lidar can exist where the static map has
none. Every cost this prints is therefore a LOWER BOUND on the live
cost, and every clearance an UPPER BOUND on the live clearance. A pose
this module calls "occupied" was occupied live; a pose it calls "free"
may still have been refused live.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
MAP_PGM = os.path.join(ROOT, 'gazebo_models', 'maps', 'coco_world.pgm')

# From gazebo_models/maps/coco_world.yaml (OBSERVED)
RES = 0.050
ORIGIN = (-2.119, -4.910)
OCC_THRESH = 0.65

# From docs/data/c2nav11_ntp_params.yaml, global_costmap (OBSERVED)
ROBOT_RADIUS = 0.20          # == inscribed_radius, circular footprint
INFLATION_RADIUS = 0.50
COST_SCALING_GLOBAL = 5.0
INSCRIBED = 253
LETHAL = 254

# gazebo_models/scripts/nav_bench.py lines 88-89 (OBSERVED)
W2M = (2.0, 0.0)

# The measured true circumscribed radius of the robot body, from the
# C2-NAV.0 footprint work quoted in the C2-NAV.27 brief. NOT the
# configured robot_radius, which is 0.20.
TRUE_CIRCUMSCRIBED = 0.2051


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    # P5 binary greyscale: magic, w h, maxval, then raster.
    idx = 0
    fields = []
    while len(fields) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            while data[idx:idx + 1] not in (b'\n', b''):
                idx += 1
            continue
        j = idx
        while not data[j:j + 1].isspace():
            j += 1
        fields.append(data[idx:j])
        idx = j
    idx += 1
    w, h, mx = int(fields[1]), int(fields[2]), int(fields[3])
    assert fields[0] == b'P5' and mx == 255, fields[:1]
    img = np.frombuffer(data[idx:idx + w * h], dtype=np.uint8)
    return img.reshape(h, w), w, h


def occupied_centres():
    """(N,2) map-frame xy of every occupied static cell centre."""
    img, w, h = read_pgm(MAP_PGM)
    # map_server trinary: occ = (255 - px)/255; occupied if occ > thresh.
    occ = (255.0 - img.astype(np.float64)) / 255.0
    rows, cols = np.nonzero(occ > OCC_THRESH)
    # PGM row 0 is the TOP of the image == the HIGHEST y in the map.
    mx = ORIGIN[0] + (cols + 0.5) * RES
    my = ORIGIN[1] + (h - rows - 0.5) * RES
    return np.column_stack([mx, my]), (w, h)


OCC, SHAPE = occupied_centres()


def dist_to_lethal(mx, my):
    """Exact distance from a map-frame point to the nearest lethal cell
    centre. This is the same metric nav_bench's `min_clearance_m` uses
    and the same metric InflationLayer inflates over."""
    d = np.hypot(OCC[:, 0] - mx, OCC[:, 1] - my)
    i = int(np.argmin(d))
    return float(d[i]), (float(OCC[i, 0]), float(OCC[i, 1]))


def cell_centre(mx, my):
    """Centre of the costmap cell containing (mx,my), map frame."""
    cx = ORIGIN[0] + (math.floor((mx - ORIGIN[0]) / RES) + 0.5) * RES
    cy = ORIGIN[1] + (math.floor((my - ORIGIN[1]) / RES) + 0.5) * RES
    return cx, cy


def inflation_cost(d, k=COST_SCALING_GLOBAL):
    if d <= 1e-9:
        return LETHAL
    if d <= ROBOT_RADIUS:
        return INSCRIBED
    if d > INFLATION_RADIUS:
        return 0
    return int((INSCRIBED - 1) * math.exp(-k * (d - ROBOT_RADIUS)))


def verdict(mx, my, k=COST_SCALING_GLOBAL):
    """Reproduce the SmacPlanner2D start test at a map-frame pose."""
    cx, cy = cell_centre(mx, my)
    d_cell, near = dist_to_lethal(cx, cy)
    d_pt, _ = dist_to_lethal(mx, my)
    c = inflation_cost(d_cell, k)
    return dict(cell=(cx, cy), d_cell=d_cell, d_point=d_pt, cost=c,
                occupied=(c >= INSCRIBED), nearest=near)


def show(label, wx, wy, note=''):
    mx, my = wx + W2M[0], wy + W2M[1]
    v = verdict(mx, my)
    flag = 'START OCCUPIED' if v['occupied'] else 'plannable'
    print('%-42s world(%7.3f,%7.3f) map(%7.3f,%7.3f)  '
          'd_cell=%.3f  cost=%3d  %-14s %s'
          % (label, wx, wy, mx, my, v['d_cell'], v['cost'], flag, note))
    return v


def main():
    print('map %dx%d cells, res %.3f, origin %s, %d occupied cells'
          % (SHAPE[0], SHAPE[1], RES, ORIGIN, len(OCC)))
    print('SmacPlanner2D start test (DERIVED from headers+params): '
          'refuse iff cell cost >= %d, i.e. d(cell centre -> nearest '
          'lethal centre) <= %.2f m' % (INSCRIBED, ROBOT_RADIUS))
    print()

    print('=== 1. where is the wall behind the wall_adjacent goal? ===')
    gx, gy = -2.0, -3.0
    d, near = dist_to_lethal(gx + 2.0, gy)
    print('goal world(%.3f,%.3f) map(%.3f,%.3f): nearest lethal cell '
          'centre at map(%.3f,%.3f), d=%.3f m'
          % (gx, gy, gx + 2.0, gy, near[0], near[1], d))
    # scan the wall face along -y directly under the goal
    col = OCC[np.abs(OCC[:, 0] - 0.0) < 0.03]
    below = col[col[:, 1] < -3.0]
    if len(below):
        print('occupied cell centres at map x~0.00 with y<-3.00: '
              'highest y = %.3f (the wall face)' % below[:, 1].max())
    print()

    print('=== 2. the six c2n26_slow_r2 refusals, at the AMCL pose the '
          'planner was actually given (OBSERVED from nav log) ===')
    for sx, sy, tgt in [(0.01, -3.18, 'wall_adjacent  (0.00,-3.00)'),
                        (0.02, -3.19, 'wall_parallel  (2.50,-2.95)'),
                        (0.02, -3.19, 'obstacle_corner(2.30,-0.30)'),
                        (0.02, -3.19, 'corridor_gate  (-0.60,-0.10)'),
                        (0.02, -3.19, 'enclosure_entry(-1.00, 0.62)'),
                        (0.02, -3.19, 'enclosure_exit (0.00, 0.00)')]:
        show('AMCL start -> ' + tgt, sx - 2.0, sy)
    print()

    print('=== 3. the SAME instant in GROUND TRUTH (trace, OBSERVED) ===')
    show('gt at abort  c2n26_slow_r2/wall_adjacent', -1.9523, -3.0251,
         '<- final resting pose')
    show('gt deepest   c2n26_slow_r2/wall_adjacent', -1.9397, -3.0759,
         '<- max overshoot in ground truth')
    print()

    print('=== 4. the two other C2-NAV.26 "Start occupied" refusals ===')
    show('c2n26_bslow_r3 AMCL start (enclosure)', -1.12 - 2.0, 2.02)
    show('c2n26_bslow_r3 gt end', -3.167, 1.897)
    print()

    print('=== 5. prior-art refusals at BASELINE-family configs ===')
    show('c2n21_bbase_r1 AMCL start (baseline cfg)', 2.20 - 2.0, -1.63)
    show('c2n21_sim_r1   AMCL start', 2.20 - 2.0, -1.65)
    show('c2n21_sim_r2   AMCL start', 2.14 - 2.0, -1.67)
    show('c2n21_sim_r3   AMCL start', 2.21 - 2.0, -1.53)
    show('c2n12_tour_r1  AMCL start', -0.53 - 2.0, 2.33)
    print()

    print('=== 6. how much AMCL error does wall_adjacent tolerate? ===')
    print('Sweep the ESTIMATED pose in -y from the goal, holding x at the')
    print('goal, and report where the planner flips to refusing.')
    for dy in [0.00, 0.05, 0.10, 0.125, 0.15, 0.155, 0.16, 0.165,
               0.17, 0.175, 0.18, 0.19, 0.20, 0.25]:
        v = verdict(0.0, -3.0 - dy)
        print('   est y = %-8.3f  d_cell=%.3f  cost=%3d  %s'
              % (-3.0 - dy, v['d_cell'], v['cost'],
                 'REFUSED' if v['occupied'] else 'ok'))
    print()
    print('true robot circumscribed radius %.4f m vs configured '
          'robot_radius %.2f m -- the planner inflates by the CONFIGURED '
          'value, so the inscribed band is %.4f m SHORT of the body.'
          % (TRUE_CIRCUMSCRIBED, ROBOT_RADIUS,
             TRUE_CIRCUMSCRIBED - ROBOT_RADIUS))


if __name__ == '__main__':
    sys.exit(main())
