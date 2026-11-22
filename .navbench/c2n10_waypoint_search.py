import sys, os
sys.path.insert(0, 'docs/data')
import numpy as np
from c2nav9_corridor import (build_clearance_grid, grid_index, bottleneck,
                              CORRIDOR_GATE_GOAL, ENCLOSURE_ENTRY_SHIFTED,
                              GRID_X, GRID_Y, GRID_RES, POLY_STOP_R,
                              POLY_SLOW_HW, POLY_SLOW_MAX, zone_status_all_yaw)
from c2nav8_report import nearest_full

xs, ys, clr = build_clearance_grid()
print("grid shape", clr.shape)

starts = {
    'canonical corridor_gate goal': CORRIDOR_GATE_GOAL,
    'r1 corridor_gate end (GT)': (-2.6162, 0.0325),
    'r2 corridor_gate end (GT)': (-2.6111, -0.0291),
    'r3 corridor_gate end (GT)': (-2.5830, -0.0076),
}
goal = ENCLOSURE_ENTRY_SHIFTED

print()
print("Clearance survey near suggested region (x in [-3.75,-3.30], y in [1.0,1.8]):")
xs_scan = np.arange(-3.75, -3.29, 0.05)
for y in np.arange(1.00, 1.81, 0.05):
    row = []
    for x in xs_scan:
        iy, ix = grid_index(xs, ys, x, y)
        c = clr[iy, ix]
        row.append(c)
    print(f"y={y:5.2f}  " + " ".join(f"{c*1000:5.0f}" for c in row))
print("x axis:    " + " ".join(f"{x:5.2f}" for x in xs_scan))
