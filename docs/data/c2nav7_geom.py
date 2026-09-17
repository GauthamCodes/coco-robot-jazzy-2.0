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
"""C2-NAV.7 pre-edit geometry: where the enclosure_entry goal actually
sits, and where a robot may pass without entering PolygonStop.

C2-NAV.6 proved the exit trap is a real convex corner 5.5 mm inside a
0.25 m stop circle, and that raising `min_points` only lets the robot
advance to a pose where the corner is deeper. That leaves the pose
itself. This computes, from geometry rather than from a run:

  1. WHICH obstacle. Not assumed -- the six laser returns C2-NAV.6
     recorded inside the circle are transformed into world coordinates
     using the ground-truth pose recorded in the same session, and
     compared against the world file's boxes. If the hypothesis is about
     a stand-off, the thing being stood off from has to be identified by
     measurement first.
  2. The true stand-off at the goal, which is NOT the "0.35 m" the brief
     carries forward and is not measured from a wall.
  3. The PolygonStop-free corridor through the NW pinch: the band of x
     in which the base origin is more than `PolygonStop.radius` from
     BOTH the west wall and box_obstacle_1. The exit path has to fit
     through it, and the current goal's relationship to it is the whole
     question.

Everything here is offline arithmetic over the world file's collision
boxes and C2-NAV.6's committed CSV. It launches nothing and changes
nothing.

  4. The TRUE clearance along a recorded run, which is the arbiter when
     the other two clearance numbers disagree -- and on these runs they
     disagree badly. `nav_bench`'s `min_clearance_m` is the distance to
     the nearest OCCUPIED MAP CELL: 360 deg, but quantised to the 5 cm
     grid and measured to cell centres. The probe's `d_min_base_m` is the
     distance to the nearest LASER RETURN: exact, but the lidar is a
     240 deg scanner and is BLIND to the rear 120 deg. Neither is
     trustworthy alone. Distance to the nearest FACE of a world-file
     collision box is both 360 deg and unquantised.

Usage:
  python3 c2nav7_geom.py                       # the pre-edit analysis
  python3 c2nav7_geom.py track <csv> <label> [<csv> <label> ...]
"""
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# gazebo_models/worlds/coco_world.world, <collision> boxes, world frame.
# (name, centre_x, centre_y, size_x, size_y)
BOXES = [
    ('wall_north', 2.0, 3.5, 12.0, 0.2),
    ('wall_south', 2.0, -3.5, 12.0, 0.2),
    ('wall_west', -4.0, 0.0, 0.2, 7.2),
    ('wall_east', 8.0, 0.0, 0.2, 7.2),
    ('box_obstacle_1', -3.0, 2.4, 0.5, 0.5),
    ('box_obstacle_2', 0.8, -1.4, 0.5, 0.5),
    ('gate_cube_north', -1.1, 1.05, 0.5, 0.5),
    ('gate_cube_south', -1.1, -0.75, 0.5, 0.5),
]

# nav2_params.yaml collision_monitor.PolygonStop, and C2-NAV.0's measured
# footprint. robot_radius 0.20 is the PLANNER's number and is 5.1 mm
# smaller than the robot; 0.2051 is the measured circumscribed radius.
STOP_RADIUS = 0.25
CIRCUMSCRIBED = 0.2051

# nav_bench.py TOUR, world frame.
GOAL_ENTRY = (-3.45, 2.95)
GOAL_EXIT = (-2.00, 0.00)

# C2-NAV.6, docs/data/c2nav6_base_r1_stop.csv: the pose held for all 1470
# STOP frames, and docs/data/c2nav6_base_r1_geom.json's six inside points.
STALL_POSE = (-3.4558, 2.7805, -1.8124)


def rect(b):
    _, cx, cy, sx, sy = b
    return (cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2)


def dist_to_box(px, py, b):
    """Distance from a point to the closest point of an axis-aligned box.

    Zero inside. Returns (distance, closest point).
    """
    x0, x1, y0, y1 = rect(b)
    qx = min(max(px, x0), x1)
    qy = min(max(py, y0), y1)
    return math.hypot(px - qx, py - qy), (qx, qy)


def nearest(px, py, exclude=()):
    out = []
    for b in BOXES:
        if b[0] in exclude:
            continue
        d, q = dist_to_box(px, py, b)
        out.append((d, b[0], q))
    out.sort()
    return out


def hdr(t):
    print()
    print('=' * 72)
    print(t)
    print('=' * 72)


def identify_obstacle():
    """Transform C2-NAV.6's inside-circle returns into the world.

    The point of this is that the obstacle is IDENTIFIED, not assumed.
    """
    hdr('1. WHICH obstacle fires PolygonStop  (from C2-NAV.6 measurement)')
    import json
    p = os.path.join(HERE, 'c2nav6_base_r1_geom.json')
    if not os.path.exists(p):
        print(f'  missing {p}')
        return None
    doc = json.load(open(p))
    x, y, th = STALL_POSE
    c, s = math.cos(th), math.sin(th)
    print(f'  stall pose (world)      : ({x:.4f}, {y:.4f}, '
          f'{th:.4f} rad = {math.degrees(th):.2f} deg)')
    print('  held for                : 1470 of 1470 STOP frames, stationary')
    print(f'  returns inside the circle: {doc["n_inside"]}')
    print()
    print('   i   base(x,y)          -> world(x,y)        nearest box      '
          'gap')
    worst = None
    for pt in doc['inside']:
        bx, by = pt['x_base'], pt['y_base']
        wx = x + c * bx - s * by
        wy = y + s * bx + c * by
        d, name, q = nearest(wx, wy)[0]
        print(f'  {pt["i"]:4d}  ({bx:+.4f}, {by:+.4f}) -> '
              f'({wx:+.4f}, {wy:+.4f})  {name:<15} {d * 1000:6.1f} mm')
        if worst is None or d > worst:
            worst = d
    print()
    print(f'  Every return lands on box_obstacle_1 to within {worst*1000:.1f} mm.')
    x0, x1, y0, y1 = rect([b for b in BOXES if b[0] == 'box_obstacle_1'][0])
    # NORTH-WEST is min x, max y. (x1, y1) would be the north-EAST corner,
    # 0.5 m along the box and not the one anything here touches.
    print(f'  box_obstacle_1 spans x [{x0}, {x1}], y [{y0}, {y1}];'
          f' its NW corner is ({x0}, {y1}).')
    d_corner = math.hypot(x - x0, y - y1)
    print(f'  stall pose to that corner: {d_corner:.4f} m  '
          f'(probe measured d_min_base = 0.2445 m, '
          f'delta {abs(d_corner - 0.2445)*1000:.1f} mm)')
    print()
    print("  => The obstacle is box_obstacle_1's NORTH-WEST CORNER. It is a")
    print('     convex corner, which is why only ~6 beams fall inside the')
    print('     circle: the surface turns away on both sides.')
    return d_corner


def standoff_at_goal():
    hdr('2. The true stand-off at the enclosure_entry goal')
    gx, gy = GOAL_ENTRY
    print(f'  goal (world)  : ({gx}, {gy})   [nav_bench.py TOUR, '
          'position only; yaw is a shared orientation.w = 1.0]')
    print()
    print('  distance from the goal to every relevant collision box:')
    for d, name, q in nearest(gx, gy)[:4]:
        print(f'    {name:<16} {d:.4f} m   closest point ({q[0]:+.3f}, '
              f'{q[1]:+.3f})')
    d0, n0, q0 = nearest(gx, gy)[0]
    print()
    print(f'  nearest geometry: {n0} at {d0:.4f} m')
    print(f'  PolygonStop.radius = {STOP_RADIUS} m  =>  the goal itself is '
          f'{(d0 - STOP_RADIUS)*1000:.0f} mm OUTSIDE the stop circle.')
    print(f'  circumscribed radius {CIRCUMSCRIBED} m => '
          f'{(d0 - CIRCUMSCRIBED)*1000:.0f} mm of physical margin.')
    print()
    print('  NOTE. The brief carries "approximately 0.35 m" forward. The')
    print(f'  measured value is {d0:.4f} m and it is NOT to a wall -- it is')
    print("  to box_obstacle_1's NW corner, the same corner that fires the")
    print('  stop. So the goal is not itself inside PolygonStop, and the')
    print('  hypothesis has to be about the EXIT PATH, not the goal pose in')
    print('  isolation.')
    return d0


def corridor():
    hdr('3. The PolygonStop-free corridor through the NW pinch')
    wall = [b for b in BOXES if b[0] == 'wall_west'][0]
    box = [b for b in BOXES if b[0] == 'box_obstacle_1'][0]
    wx0, wx1, wy0, wy1 = rect(wall)
    bx0, bx1, by0, by1 = rect(box)
    gap = bx0 - wx1
    print(f'  wall_west east face   x = {wx1:+.3f}')
    print(f'  box_obstacle_1 west face x = {bx0:+.3f}, y band '
          f'[{by0:+.3f}, {by1:+.3f}]')
    print(f'  pinch gap             = {gap:.3f} m')
    print()
    lo = wx1 + STOP_RADIUS
    hi = bx0 - STOP_RADIUS
    print(f'  To keep the BASE ORIGIN more than PolygonStop.radius '
          f'({STOP_RADIUS} m) from')
    print(f'  both, x must satisfy   {lo:+.3f} <= x <= {hi:+.3f}')
    print(f'  corridor width        = {hi - lo:.3f} m')
    print(f'  corridor centre       = {(lo + hi) / 2:+.4f}')
    print()
    gx, _ = GOAL_ENTRY
    print(f'  enclosure_entry goal x = {gx:+.3f}')
    if lo <= gx <= hi:
        print('  -> the goal IS inside the corridor')
    else:
        off = gx - hi if gx > hi else lo - gx
        print(f'  -> the goal is OUTSIDE the corridor by {off*1000:.0f} mm '
              f'(too far EAST, i.e. too close to box_obstacle_1)')
    sx = STALL_POSE[0]
    print(f'  C2-NAV.6 stall pose  x = {sx:+.4f}   '
          f'({"inside" if lo <= sx <= hi else "OUTSIDE"} the corridor)')
    print()
    print('  This is the mechanism the goal position controls. The exit leg')
    print('  must traverse the pinch southward. Anywhere east of '
          f'{hi:+.3f} the')
    print('  base origin is within 0.25 m of box_obstacle_1 and PolygonStop')
    print('  fires; anywhere west of ' f'{lo:+.3f}' ' the west wall fires.')
    return lo, hi


def candidates(lo, hi):
    hdr('4. Candidate goals, scored against the corridor')
    gx, gy = GOAL_ENTRY
    centre = (lo + hi) / 2
    cands = [
        ('current', gx),
        ('shift 0.05 m west', gx - 0.05),
        ('shift 0.10 m west', gx - 0.10),
        ('corridor centre', centre),
    ]
    print(f'{"candidate":<22}{"x":>9}{"min gap":>10}{"nearest":>17}'
          f'{"east margin":>13}{"west margin":>13}')
    for name, cx in cands:
        d, n, _ = nearest(cx, gy)[0]
        em = hi - cx          # room before box_obstacle_1 stops it
        wm = cx - lo          # room before wall_west stops it
        print(f'{name:<22}{cx:>9.4f}{d:>10.4f}{n:>17}'
              f'{em * 1000:>11.0f} mm{wm * 1000:>11.0f} mm')
    print()
    print('  "east/west margin" is the lateral error the robot may carry')
    print('  through the pinch before the base origin enters PolygonStop on')
    print('  that side. C2-NAV.6 measured the entry goal error at 0.080 m')
    print('  and 0.096 m, so a candidate whose margin is under ~0.10 m can')
    print('  be swallowed by ordinary tracking error.')
    print()
    print(f'  CHOSEN: x = {centre:+.4f} (corridor centre), y unchanged at '
          f'{gy}.')
    print(f'  Shift = {abs(centre - gx)*1000:.0f} mm west, one coordinate.')
    print('  It is the only choice that is inside the corridor with')
    print('  symmetric margin; a 0.05 m shift lands on the corridor EDGE')
    print('  with zero margin, and 0.10 m leaves only 50 mm on the east.')
    return centre


def sanity(newx):
    hdr('5. Is the shifted goal still a sensible mission pose?')
    gy = GOAL_ENTRY[1]
    wall = [b for b in BOXES if b[0] == 'wall_west'][0]
    north = [b for b in BOXES if b[0] == 'wall_north'][0]
    print(f'  candidate goal ({newx:+.4f}, {gy})')
    for d, n, q in nearest(newx, gy)[:3]:
        print(f'    {n:<16} {d:.4f} m')
    print()
    print(f'  still inside the corner pocket: west wall '
          f'{newx - rect(wall)[1]:.3f} m to the west, north wall '
          f'{rect(north)[2] - gy:.3f} m to the north.')
    print(f'  clearance {nearest(newx, gy)[0][0]:.4f} m > planner '
          f'robot_radius 0.20 and > inscribed 0.2059, so the cell is')
    print('  plannable rather than lethal.')
    print()
    print('  Manipulation relevance is OBSERVED here, not changed:')
    print("  nav_bench.py's TOUR is the NAVIGATION benchmark. The fetch")
    print("  mission's target bay and grasp poses live in coco_perception")
    print('  and coco_mission and are not read from this list, so moving a')
    print('  benchmark waypoint cannot alter grasp geometry. Verified by')
    print('  grep in this session; see the C2-NAV.7 record.')


def track(argv):
    """Exact minimum clearance along each leg of a recorded stop CSV.

    Legs are labelled by the goal the run actually drove to, read off the
    `goal_map_x/y` columns, so a shifted goal cannot be silently scored
    as the original one.
    """
    hdr('TRUE clearance along recorded runs (world-file boxes, 360 deg, '
        'unquantised)')
    known = {(-1.575, 2.95): 'enclosure_entry(goal -3.575)',
             (-1.45, 2.95): 'enclosure_entry(goal -3.45)',
             (0.0, 0.0): 'enclosure_exit'}
    for path, label in zip(argv[0::2], argv[1::2]):
        legs = {}
        for r in csv.DictReader(open(path)):
            if not r['gt_x'] or not r['goal_map_x']:
                continue
            gx, gy = float(r['goal_map_x']), float(r['goal_map_y'])
            name = next((v for (kx, ky), v in known.items()
                         if abs(gx - kx) < 0.02 and abs(gy - ky) < 0.02),
                        None)
            if name is None:
                continue
            legs.setdefault(name, []).append(
                (float(r['gt_x']), float(r['gt_y'])))
        print(f'  --- {label} ---')
        for name, pts in legs.items():
            (d, who, q), (bx, by) = min(
                (nearest(x, y)[0], (x, y)) for x, y in pts)
            note = ''
            if d < CIRCUMSCRIBED:
                note = '   <<< BELOW THE CIRCUMSCRIBED RADIUS'
            elif d < STOP_RADIUS:
                note = f'   (inside PolygonStop by {(STOP_RADIUS-d)*1000:.1f} mm)'
            else:
                note = f'   (clear of PolygonStop by {(d-STOP_RADIUS)*1000:.1f} mm)'
            print(f'    {name:<28} n={len(pts):5d}  min {d:.4f} m to '
                  f'{who:<15} at ({bx:+.4f}, {by:+.4f}){note}')
        print()
    return 0


def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'track':
        return track(sys.argv[2:])
    identify_obstacle()
    standoff_at_goal()
    lo, hi = corridor()
    newx = candidates(lo, hi)
    sanity(newx)
    hdr('SUMMARY')
    print('  obstacle          : box_obstacle_1 NW corner (-3.25, 2.65)')
    print(f'  goal stand-off    : {nearest(*GOAL_ENTRY)[0][0]:.4f} m '
          '(to that corner, not a wall)')
    print(f'  stop-free corridor: x in [{lo:+.3f}, {hi:+.3f}], '
          f'width {hi-lo:.3f} m')
    print(f'  current goal x    : {GOAL_ENTRY[0]:+.3f}  -> OUTSIDE by '
          f'{(GOAL_ENTRY[0]-hi)*1000:.0f} mm')
    print(f'  candidate goal x  : {newx:+.4f}  -> corridor centre')
    return 0


if __name__ == '__main__':
    sys.exit(main())
