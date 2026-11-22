import sys
sys.path.insert(0, 'docs/data')
import numpy as np
from c2nav9_corridor import (build_clearance_grid, grid_index, bottleneck,
                              CORRIDOR_GATE_GOAL, ENCLOSURE_ENTRY_SHIFTED,
                              POLY_STOP_R, POLY_SLOW_HW, POLY_SLOW_MAX,
                              zone_status_all_yaw, FOOT_CIRC_R)
from c2nav8_report import nearest_full

xs, ys, clr = build_clearance_grid()

WAYPOINT = (-3.40, 1.35)
GOAL = ENCLOSURE_ENTRY_SHIFTED

starts = {
    'canonical corridor_gate goal': CORRIDOR_GATE_GOAL,
    'r1 corridor_gate end (GT)': (-2.6162, 0.0325),
    'r2 corridor_gate end (GT)': (-2.6111, -0.0291),
    'r3 corridor_gate end (GT)': (-2.5830, -0.0076),
}

print(f"Candidate waypoint: {WAYPOINT}")
d_all = nearest_full(*WAYPOINT)
print(f"  nearest world geometry: {d_all[0][1]} at {d_all[0][0]*1000:.1f} mm")
iy, ix = grid_index(xs, ys, *WAYPOINT)
print(f"  grid clearance lookup : {clr[iy,ix]*1000:.1f} mm")
print(f"  circumscribed margin  : {(d_all[0][0]-FOOT_CIRC_R)*1000:.1f} mm above {FOOT_CIRC_R} m")

print()
print("Bottleneck (widest-path tau*) start -> waypoint:")
for name, s in starts.items():
    tau = bottleneck(clr, xs, ys, s, WAYPOINT, lo=0.0, hi=0.60)
    print(f"  {name:<32}{tau*1000:8.1f} mm")

print()
print("Bottleneck (widest-path tau*) waypoint -> final goal:")
tau_wg = bottleneck(clr, xs, ys, WAYPOINT, GOAL, lo=0.0, hi=0.60)
print(f"  waypoint -> enclosure_entry (shifted)  {tau_wg*1000:8.1f} mm")

print()
print("PolygonStop / PolygonSlow / PolygonLimit sweep at waypoint (all headings):")
z = zone_status_all_yaw(*WAYPOINT)
if z is None:
    print("  no obstacle within 1.2 m radius -- all zones NEVER triggered")
else:
    for name in ('PolygonStop', 'PolygonSlow', 'PolygonLimit'):
        d = z[name]
        state = ('ALWAYS triggered' if d['always'] else
                 'NEVER triggered' if d['never'] else
                 f'triggered {d["frac_triggered"]*100:.1f}% of headings')
        print(f"    {name:<14}: {state}")

print()
print("Heading check (bearing in world deg, 0=east/+x, 90=north/+y):")
import math
cg = CORRIDOR_GATE_GOAL
b1 = math.degrees(math.atan2(WAYPOINT[1]-cg[1], WAYPOINT[0]-cg[0]))
b2 = math.degrees(math.atan2(GOAL[1]-WAYPOINT[1], GOAL[0]-WAYPOINT[0]))
print(f"  corridor_gate -> waypoint bearing : {b1:+.1f} deg")
print(f"  waypoint -> enclosure_entry bearing: {b2:+.1f} deg")
print(f"  turn required at waypoint          : {((b2-b1+180)%360-180):+.1f} deg")
