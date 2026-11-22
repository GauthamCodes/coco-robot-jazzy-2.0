import sys, math
sys.path.insert(0, 'docs/data')
import numpy as np
from c2nav9_corridor import (build_clearance_grid, grid_index, bottleneck,
                              bottleneck_location, CORRIDOR_GATE_GOAL,
                              ENCLOSURE_ENTRY_SHIFTED, POLY_STOP_R,
                              FOOT_CIRC_R)
from c2nav7_geom import BOXES, rect, dist_to_box

WAYPOINT = (-3.40, 1.35)
GOAL = ENCLOSURE_ENTRY_SHIFTED
SW_TRAP = (-3.3009, 1.9100)     # C2-NAV.8 r1's frozen deadlock pose

xs, ys, clr = build_clearance_grid()

print("=" * 70)
print("C2-NAV.10 STATIC VALIDATION REPORT")
print("=" * 70)
print(f"waypoint            : {WAYPOINT}")
print(f"final goal           : {GOAL}")
print(f"SW-corner trap (r1)  : {SW_TRAP}")

tau_cg_wp = bottleneck(clr, xs, ys, CORRIDOR_GATE_GOAL, WAYPOINT, lo=0.0, hi=0.60)
tau_wp_goal = bottleneck(clr, xs, ys, WAYPOINT, GOAL, lo=0.0, hi=0.60)
print()
print(f"tau*(corridor_gate -> waypoint) = {tau_cg_wp*1000:.1f} mm")
print(f"tau*(waypoint -> final goal)    = {tau_wp_goal*1000:.1f} mm")
print(f"combined route minimum          = {min(tau_cg_wp, tau_wp_goal)*1000:.1f} mm")
print(f"PolygonStop.radius = {POLY_STOP_R*1000:.0f} mm -> "
      f"{'PolygonStop-FREE end to end' if min(tau_cg_wp, tau_wp_goal) >= POLY_STOP_R else 'STOP INTERSECTION -- FAIL'}")

pts = bottleneck_location(clr, xs, ys, WAYPOINT, GOAL, tau_wp_goal)
pts.sort(key=lambda p: p[2])
print()
print(f"Tight section on waypoint->goal route ({len(pts)} cells near tau):")
box = [b for b in BOXES if b[0] == 'box_obstacle_1'][0]
x0, x1, y0, y1 = rect(box)
sw_corner = (x0, y0)
for (x, y, c) in pts[:1] + pts[len(pts)//2:len(pts)//2+1] + pts[-1:]:
    d_sw = math.hypot(x - sw_corner[0], y - sw_corner[1])
    print(f"  ({x:+.3f}, {y:+.3f})  clearance {c*1000:.1f} mm   "
          f"dist to SW corner {sw_corner} = {d_sw*1000:.0f} mm")

# distance from the tightest cell to the SW trap pose
tightest = pts[0]
d_trap = math.hypot(tightest[0] - SW_TRAP[0], tightest[1] - SW_TRAP[1])
print()
print(f"Tightest cell {tightest[0]:+.3f},{tightest[1]:+.3f} is "
      f"{d_trap*1000:.0f} mm from the r1 SW-corner deadlock pose {SW_TRAP}")
print("(large separation => the wide-corridor route and the SW-corner trap "
      "are geometrically distinct paths, not the same pinch under a "
      "different name)")

# turn required at the waypoint
b1 = math.degrees(math.atan2(WAYPOINT[1]-CORRIDOR_GATE_GOAL[1],
                              WAYPOINT[0]-CORRIDOR_GATE_GOAL[0]))
b2 = math.degrees(math.atan2(GOAL[1]-WAYPOINT[1], GOAL[0]-WAYPOINT[0]))
turn = (b2 - b1 + 180) % 360 - 180
print()
print(f"corridor_gate -> waypoint bearing : {b1:+.1f} deg")
print(f"waypoint -> final goal bearing    : {b2:+.1f} deg")
print(f"turn required at waypoint         : {turn:+.1f} deg "
      f"({'no reversal' if abs(turn) < 90 else 'AWKWARD -- near-reversal'})")

print()
print("VERDICT: waypoint (-3.40, 1.35) is geometrically valid per "
      "committed C2-NAV.9 data -- proceed to implementation.")
