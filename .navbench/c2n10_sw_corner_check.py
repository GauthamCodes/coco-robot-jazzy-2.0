import csv, math, sys

SW_CORNER = (-3.25, 2.15)
DEADLOCK_POSE = (-3.3009, 1.9100)   # C2-NAV.8 r1's frozen pose

path = sys.argv[1] if len(sys.argv) > 1 else '.navbench/results/c2n10_appr_r1_stop.csv'
rows = list(csv.DictReader(open(path)))
gt = [(float(r['gt_x']), float(r['gt_y']), float(r['t_s']), r['monitor_action'])
      for r in rows if r['gt_x']]

d_sw = [(math.hypot(x - SW_CORNER[0], y - SW_CORNER[1]), t, x, y, act)
        for (x, y, t, act) in gt]
d_sw.sort()
print(f"rows with GT: {len(gt)}")
print(f"closest approach to SW corner {SW_CORNER}:")
for d, t, x, y, act in d_sw[:5]:
    print(f"  t={t:7.2f}s  pose=({x:+.3f},{y:+.3f})  dist={d*1000:.0f} mm  monitor={act!r}")

d_dl = [(math.hypot(x - DEADLOCK_POSE[0], y - DEADLOCK_POSE[1]), t, x, y)
        for (x, y, t, act) in gt]
d_dl.sort()
print()
print(f"closest approach to C2-NAV.8 r1's deadlock pose {DEADLOCK_POSE}:")
for d, t, x, y in d_dl[:3]:
    print(f"  t={t:7.2f}s  pose=({x:+.3f},{y:+.3f})  dist={d*1000:.0f} mm")

# STOP frame count, whole run
from collections import Counter
c = Counter(a for (_, _, _, a) in gt)
print()
print(f"monitor_action counts (whole run, all legs): {dict(c)}")
print(f"STOP frames: {c.get('STOP', 0)} of {len(gt)} ({100*c.get('STOP',0)/len(gt):.2f}%)")

# time spent within 400mm of the SW corner at all (a proxy for "entered
# the SW-corner danger region")
near = [d for d in d_sw if d[0] < 0.40]
print()
print(f"samples within 400 mm of SW corner: {len(near)} of {len(gt)} "
      f"({100*len(near)/len(gt):.2f}%)")
