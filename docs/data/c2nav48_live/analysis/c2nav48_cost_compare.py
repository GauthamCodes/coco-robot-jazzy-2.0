#!/usr/bin/env python3
"""Why the defect is LOCAL, not global: inflation cost in the fatal band."""
import math

ap = math.cos(math.pi / 16)
ins20 = (0.20 + 0.01) * ap


def cost(d, csf, ins=ins20):
    if d <= ins:
        return 253.0
    return 252 * math.exp(-csf * (d - ins))


print(f"inscribed radius at robot_radius 0.20 = {ins20:.6f} m")
print("(C2-NAV.0 measured 0.205879)\n")
print("cost of a robot-centre pose at clearance d  (253 = inscribed-lethal):")
print(f"{'d (m)':>9} {'GLOBAL csf 5.0':>16} {'LOCAL csf 65.0':>16}")
for d in (0.2486, 0.25, 0.2724, 0.30, 0.35):
    print(f"{d:>9.4f} {cost(d, 5.0):>16.1f} {cost(d, 65.0):>16.1f}")
print()
print("GLOBAL planner pays ~202/254 to pass at 0.25 m -> it already avoids the band.")
print("LOCAL controller prices the same pose at ~14/254 -> open floor.")
print("=> the defect is LOCAL. Raising the LOCAL robot_radius is the minimal")
print("   correct target; the global costmap needs no change.")
