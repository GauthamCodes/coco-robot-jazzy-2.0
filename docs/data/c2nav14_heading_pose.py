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
"""C2-NAV.14: offline derivation and validation of ONE heading-correcting
through-pose on the corridor_gate -> enclosure_entry approach, added
BEFORE the existing corridor waypoint (-3.40, 1.35), which itself stays
unmoved.

C2-NAV.13 found the tour's corridor_gate-exit yaw is REVERSED in sign
relative to the fresh two-leg approach (tour -0.29..-0.50 rad vs. fresh
+0.32..+0.51 rad, same six runs, position agreeing within 5 cm), so the
tour needs 36-58 deg MORE turn to face the waypoint's bearing -- present
at t=0 of enclosure_entry, before any RemovePassedGoals tick can act.
Hypothesis A (this experiment's target): an extra through-pose, placed
so the robot must travel and turn onto the corridor_gate->waypoint
bearing well before the narrow section, re-establishes a corridor-
aligned heading regardless of which sign the preceding leg left the
robot facing.

OFFLINE ONLY. No simulator, no Nav2 parameter touched. Reuses C2-NAV.9's
geometry machinery and C2-NAV.12/.13's constants BY IMPORT -- nothing
here re-derives what those scripts already got right, and nothing here
picks a NEW waypoint or final goal: both stay exactly
WAYPOINT=(-3.40, 1.35) and GOAL_SHIFTED=(-3.575, 2.95).

Usage:
  python3 c2nav14_heading_pose.py            # everything, self-test first
  python3 c2nav14_heading_pose.py selftest
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c2nav9_corridor import (                                # noqa: E402
    BOXES, dist_to_box, rect, zone_status_all_yaw, in_circle,
    POLY_STOP_R, POLY_SLOW_HW, POLY_LIMIT_HW, POLY_SLOW_MAX, POLY_LIMIT_MAX,
    FOOT_CIRC_R, build_clearance_grid, bottleneck, bottleneck_location,
    CORRIDOR_GATE_GOAL, nearest_full,
)
from c2nav12_report import (                                  # noqa: E402
    WAYPOINT, GOAL_SHIFTED, SW_CORNER, DEADLOCK_POSE,
)
from c2nav13_heading import FRESH_ENTRY, TOUR_ENTRY            # noqa: E402

BOX1 = [b for b in BOXES if b[0] == 'box_obstacle_1'][0]

# ---------------------------------------------------------------------
# The chosen pose. NOT tuned -- see derive_pose() below for the closed-
# form construction: the midpoint of the straight line from the
# canonical corridor_gate goal to the existing WAYPOINT, i.e. the point
# that keeps the incoming leg (corridor_gate -> heading pose) and the
# outgoing leg (heading pose -> waypoint) on the IDENTICAL bearing, so
# there is no additional turn demanded at the waypoint beyond what
# C2-NAV.10 already measured (22.6 deg, waypoint -> goal).
# ---------------------------------------------------------------------
HEADING_POSE = (-3.00, 0.625)


def hdr(t):
    print()
    print('=' * 78)
    print(t)
    print('=' * 78)


def bearing_deg(a, b):
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def ang_diff_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------------
# 1. SELF-TEST -- reproduce C2-NAV.10's committed static-report numbers
#    (docs/RESULTS.md "The waypoint, derived from C2-NAV.9's own
#    committed geometry") before trusting anything new.
# ---------------------------------------------------------------------

def self_test():
    hdr('SELF-TEST: reproduce C2-NAV.10\'s committed waypoint figures')
    ok = True

    b_cg_wp = bearing_deg(CORRIDOR_GATE_GOAL, WAYPOINT)
    print(f'  bearing corridor_gate -> waypoint : {b_cg_wp:+.1f} deg  '
          f'want 118.9 deg')
    p1 = abs(b_cg_wp - 118.9) < 0.2
    print(f'    {"PASS" if p1 else "FAIL"}')
    ok &= p1

    d_wp = nearest_full(*WAYPOINT)[0][0]
    print(f'  clearance at waypoint (nearest_full)  : {d_wp*1000:.1f} mm  '
          f'want 500 mm')
    p2 = abs(d_wp * 1000 - 500.0) < 2.0
    print(f'    {"PASS" if p2 else "FAIL"}')
    ok &= p2

    xs, ys, clr = build_clearance_grid()
    tau_cg_wp = bottleneck(clr, xs, ys, CORRIDOR_GATE_GOAL, WAYPOINT,
                            lo=0.0, hi=0.60)
    tau_wp_goal = bottleneck(clr, xs, ys, WAYPOINT, GOAL_SHIFTED,
                              lo=0.0, hi=0.60)
    print(f'  tau*(corridor_gate -> waypoint)       : {tau_cg_wp*1000:.1f} '
          f'mm  want 500 mm')
    p3 = abs(tau_cg_wp * 1000 - 500.0) < 2.0
    print(f'    {"PASS" if p3 else "FAIL"}')
    ok &= p3
    print(f'  tau*(waypoint -> final goal)          : '
          f'{tau_wp_goal*1000:.1f} mm  want 326 mm')
    p4 = abs(tau_wp_goal * 1000 - 326.0) < 2.0
    print(f'    {"PASS" if p4 else "FAIL"}')
    ok &= p4

    print()
    print(f'  SELF-TEST: {"ALL PASS" if ok else "FAILED -- STOP, fix the tool"}')
    if not ok:
        raise SystemExit(1)
    return ok, (xs, ys, clr)


# ---------------------------------------------------------------------
# 2. DERIVE THE POSE -- closed form, not a search.
# ---------------------------------------------------------------------

def derive_pose():
    hdr('DERIVE THE HEADING-CORRECTING POSE')
    b_cg_wp = bearing_deg(CORRIDOR_GATE_GOAL, WAYPOINT)
    dx = WAYPOINT[0] - CORRIDOR_GATE_GOAL[0]
    dy = WAYPOINT[1] - CORRIDOR_GATE_GOAL[1]
    mid = (CORRIDOR_GATE_GOAL[0] + 0.5 * dx, CORRIDOR_GATE_GOAL[1] + 0.5 * dy)
    print(f'  corridor_gate goal (canonical)  : {CORRIDOR_GATE_GOAL}')
    print(f'  existing waypoint (UNCHANGED)   : {WAYPOINT}')
    print(f'  vector corridor_gate -> waypoint: ({dx:+.3f}, {dy:+.3f}), '
          f'length {math.hypot(dx,dy):.3f} m, bearing {b_cg_wp:+.2f} deg')
    print(f'  midpoint (t=0.5)                : ({mid[0]:+.4f}, {mid[1]:+.4f})')
    print(f'  chosen HEADING_POSE (rounded)   : {HEADING_POSE}')
    d_round = math.hypot(HEADING_POSE[0] - mid[0], HEADING_POSE[1] - mid[1])
    print(f'  rounding error vs exact midpoint: {d_round*1000:.1f} mm')

    b1 = bearing_deg(CORRIDOR_GATE_GOAL, HEADING_POSE)
    b2 = bearing_deg(HEADING_POSE, WAYPOINT)
    b3 = bearing_deg(WAYPOINT, GOAL_SHIFTED)
    print()
    print('  Desired heading = bearing FROM heading pose TO the existing '
          'waypoint (per brief: do NOT use the final-goal bearing here).')
    print(f'    corridor_gate -> heading pose bearing : {b1:+.2f} deg')
    print(f'    heading pose  -> waypoint bearing      : {b2:+.2f} deg  '
          f'<- DESIRED HEADING at the pose')
    turn_at_pose = ang_diff_deg(b2, b1)
    print(f'    turn required AT the pose (b1 -> b2)   : {turn_at_pose:+.2f} '
          f'deg (0 => perfectly collinear, no extra turn demanded before '
          f'the waypoint)')
    print(f'    waypoint -> final goal bearing         : {b3:+.2f} deg '
          f'(C2-NAV.10\'s own 96.2 deg figure, reproduced for reference)')
    d_cg = math.dist(CORRIDOR_GATE_GOAL, HEADING_POSE)
    d_wp = math.dist(HEADING_POSE, WAYPOINT)
    print()
    print(f'  leg length corridor_gate -> heading pose : {d_cg:.3f} m')
    print(f'  leg length heading pose -> waypoint       : {d_wp:.3f} m')
    print('  (both well above the 0.2051 m circumscribed radius -- a '
          'genuine travel segment, not a teleport)')
    return b2


# ---------------------------------------------------------------------
# 3. CLEARANCE / SAFETY AT THE POSE
# ---------------------------------------------------------------------

def clearance_check(xs, ys, clr):
    hdr('CLEARANCE + COLLISION-MONITOR CHECK AT THE HEADING POSE')
    d_all = nearest_full(*HEADING_POSE)
    print(f'  nearest world geometry to {HEADING_POSE}: {d_all[0][1]} at '
          f'{d_all[0][0]*1000:.1f} mm')
    for d, name, q in d_all[:3]:
        print(f'    {name:<20} {d*1000:8.1f} mm  closest point {q}')

    print()
    print(f'  PolygonStop.radius  {POLY_STOP_R*1000:.0f} mm  -> margin '
          f'{(d_all[0][0]-POLY_STOP_R)*1000:.1f} mm')
    print(f'  PolygonSlow max reach {POLY_SLOW_MAX*1000:.1f} mm -> margin '
          f'{(d_all[0][0]-POLY_SLOW_MAX)*1000:.1f} mm')
    print(f'  PolygonLimit max reach {POLY_LIMIT_MAX*1000:.1f} mm -> margin '
          f'{(d_all[0][0]-POLY_LIMIT_MAX)*1000:.1f} mm')

    z = zone_status_all_yaw(*HEADING_POSE, n_yaw=720)
    print()
    print('  720-heading sweep at the pose (exact circle/square test, '
          'dense obstacle sample):')
    for name in ('PolygonStop', 'PolygonSlow', 'PolygonLimit'):
        d = z[name]
        state = ('ALWAYS triggered' if d['always'] else
                  'NEVER triggered' if d['never'] else
                  f'triggered {d["frac_triggered"]*100:.1f}% of headings')
        print(f'    {name:<14}: {state}')
    ok = z['PolygonStop']['never'] and z['PolygonSlow']['never'] and \
        z['PolygonLimit']['never']
    print()
    print(f'  {"PASS" if ok else "FAIL"}: pose is clear of every '
          f'collision-monitor zone at EVERY heading -- rotation in place '
          f'at this pose is safe regardless of which way the robot is '
          f'still turning when it arrives.')

    print()
    print('  Segment widest-path (max-bottleneck) clearance, so the ROUTE '
          '-- not just the endpoint -- is checked:')
    tau_cg_hp = bottleneck(clr, xs, ys, CORRIDOR_GATE_GOAL, HEADING_POSE,
                            lo=0.0, hi=0.60)
    tau_hp_wp = bottleneck(clr, xs, ys, HEADING_POSE, WAYPOINT,
                            lo=0.0, hi=0.60)
    tau_wp_goal = bottleneck(clr, xs, ys, WAYPOINT, GOAL_SHIFTED,
                              lo=0.0, hi=0.60)
    print(f'    tau*(corridor_gate -> heading pose) : {tau_cg_hp*1000:.1f} mm')
    print(f'    tau*(heading pose -> waypoint)      : {tau_hp_wp*1000:.1f} mm')
    print(f'    tau*(waypoint -> final goal)        : {tau_wp_goal*1000:.1f} '
          f'mm  (UNCHANGED from C2-NAV.9/.10 -- this segment is untouched)')
    route_min = min(tau_cg_hp, tau_hp_wp, tau_wp_goal)
    print(f'    combined three-leg route minimum    : {route_min*1000:.1f} mm')
    print(f'    PolygonStop.radius {POLY_STOP_R*1000:.0f} mm -> '
          f'{"STOP-FREE end to end" if route_min >= POLY_STOP_R else "STOP INTERSECTION -- FAIL, choose another pose"}')
    ok &= route_min >= POLY_STOP_R

    d_sw = math.dist(HEADING_POSE, SW_CORNER)
    d_dl = math.dist(HEADING_POSE, DEADLOCK_POSE)
    print()
    print(f'  distance from heading pose to SW corner {SW_CORNER}: '
          f'{d_sw:.3f} m')
    print(f'  distance from heading pose to r1/r2 SW-corner deadlock '
          f'pose {DEADLOCK_POSE}: {d_dl:.3f} m')
    print('  (the pose sits far from the SW-corner region by construction '
          '-- it is on the WIDE corridor_gate->waypoint segment, not the '
          'narrower waypoint->goal segment where the trap lives)')
    return ok


# ---------------------------------------------------------------------
# 4. WHAT THE POSE CHANGES AT t=0 OF enclosure_entry, using the SIX REAL
#    corridor_gate-exit states C2-NAV.13 already measured (not new data
#    -- FRESH_ENTRY / TOUR_ENTRY are C2-NAV.13's own committed constants,
#    imported, not retyped).
# ---------------------------------------------------------------------

def entry_state_comparison():
    hdr('EFFECT ON THE IMMEDIATE TARGET AT t=0 OF enclosure_entry '
        '(using C2-NAV.13\'s six real corridor_gate-exit poses)')
    print(f'  {"tag":<16}{"yaw":>9}{"dev(->WAYPOINT), old plan":>28}'
          f'{"dev(->HEADING_POSE), new plan":>32}')
    rows = []
    for group, table in (('fresh', FRESH_ENTRY), ('tour', TOUR_ENTRY)):
        for tag, (x, y, yaw) in table.items():
            yaw_deg = math.degrees(yaw)
            dev_old = ang_diff_deg(bearing_deg((x, y), WAYPOINT), yaw_deg)
            dev_new = ang_diff_deg(bearing_deg((x, y), HEADING_POSE), yaw_deg)
            rows.append((group, tag, yaw_deg, dev_old, dev_new))
            print(f'  {tag:<16}{yaw_deg:>8.1f}d{dev_old:>27.1f}d'
                  f'{dev_new:>31.1f}d')
    print()
    print('  dev(->X) is the turn needed at t=0 to face X. Because '
          'HEADING_POSE is closer and ~collinear with the '
          'corridor_gate->waypoint bearing, dev(->HEADING_POSE) is '
          'necessarily close to dev(->WAYPOINT) (both targets share '
          'almost the same bearing from every observed exit pose, since '
          'position varies <=5 cm across all six runs) -- this experiment '
          'does NOT reduce the magnitude of the turn required at t=0. '
          'What it changes is what DWB is asked to track over the '
          'SHORT segment right after corridor_gate, before the robot is '
          'anywhere near the SW-corner region, rather than over the full '
          'distance to a target 1.7-3.3 m away. That is a plan-shape '
          'claim, not a turn-magnitude claim, and is NOT provable '
          'offline -- it is the live prediction this experiment tests.')

    print()
    fresh_devs = [r[3] for r in rows if r[0] == 'fresh']
    tour_devs = [r[3] for r in rows if r[0] == 'tour']
    print(f'  fresh dev(->WAYPOINT) range: [{min(fresh_devs):.1f}, '
          f'{max(fresh_devs):.1f}] deg')
    print(f'  tour  dev(->WAYPOINT) range: [{min(tour_devs):.1f}, '
          f'{max(tour_devs):.1f}] deg')
    spread = [abs(t - f) for f, t in zip(
        sorted(fresh_devs), sorted(tour_devs))]
    print(f'  elementwise |tour - fresh| spread (unordered pairing, for '
          f'reference only): {[round(s,1) for s in spread]} deg -- '
          f'consistent with C2-NAV.13\'s reported 36-58 deg')
    return rows


def main():
    argv = sys.argv[1:]
    ok, grid = self_test()
    if argv and argv[0] == 'selftest':
        return 0 if ok else 1
    xs, ys, clr = grid
    derive_pose()
    ok2 = clearance_check(xs, ys, clr)
    entry_state_comparison()
    hdr('VERDICT')
    print(f'  HEADING_POSE = {HEADING_POSE}')
    print(f'  {"GEOMETRICALLY SAFE -- proceed to live validation" if ok2 else "FAILED -- choose another pose before running Gazebo"}')
    return 0 if ok2 else 1


if __name__ == '__main__':
    sys.exit(main())
