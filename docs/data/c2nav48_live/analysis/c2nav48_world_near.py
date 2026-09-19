#!/usr/bin/env python3
"""C2-NAV.48 TASK C: what real world geometry sits near the deadlock pose?

Deadlock ground-truth pose: x=0.1698 y=0.346 yaw=-2.3208 rad.
Robot base_footprint; LIDAR_MOUNT_XYZ = (-0.09, 0.10, 0.20).
PolygonStop = circle r=0.25 m about base_footprint, min_points 4.
"""
import math
import xml.etree.ElementTree as ET

WORLD = "/home/gautham/.claude/jobs/d213ad33/tmp/coco_world.clean.world"
PX, PY, PYAW = 0.1698, 0.346, -2.3208
LX, LY = -0.09, 0.10          # lidar mount in base_footprint
STOP_R = 0.25

tree = ET.parse(WORLD)
root = tree.getroot()


def pose_of(el):
    p = el.find("pose")
    if p is None or not (p.text or "").strip():
        return [0.0] * 6
    v = [float(x) for x in p.text.split()]
    return (v + [0.0] * 6)[:6]


def walk(el, px=0.0, py=0.0, pz=0.0, pyaw=0.0, path=""):
    """Yield (path, x, y, z, yaw, geom_kind, dims) for every collision/visual box."""
    for child in el:
        if child.tag not in ("model", "link", "collision", "visual", "include"):
            continue
        name = child.get("name", child.tag)
        x, y, z, _, _, yaw = pose_of(child)
        # compose parent yaw
        cx = px + x * math.cos(pyaw) - y * math.sin(pyaw)
        cy = py + x * math.sin(pyaw) + y * math.cos(pyaw)
        cz = pz + z
        cyaw = pyaw + yaw
        p2 = f"{path}/{name}"
        geo = child.find("geometry")
        if geo is not None:
            box = geo.find("box/size")
            cyl = geo.find("cylinder")
            pl = geo.find("plane")
            if box is not None and box.text:
                yield (p2, cx, cy, cz, cyaw, "box", [float(t) for t in box.text.split()])
            elif cyl is not None:
                r = cyl.find("radius"); l = cyl.find("length")
                yield (p2, cx, cy, cz, cyaw, "cylinder",
                       [float(r.text) if r is not None else 0,
                        float(l.text) if l is not None else 0])
            elif pl is not None:
                yield (p2, cx, cy, cz, cyaw, "plane", [])
        yield from walk(child, cx, cy, cz, cyaw, p2)


items = list(walk(root))
print(f"parsed {len(items)} geometry elements from coco_world.world\n")

print(f"deadlock base_footprint = ({PX:.4f}, {PY:.4f}) yaw={PYAW:.4f} rad "
      f"({math.degrees(PYAW):.2f} deg)")
lwx = PX + LX * math.cos(PYAW) - LY * math.sin(PYAW)
lwy = PY + LX * math.sin(PYAW) + LY * math.cos(PYAW)
print(f"lidar world position    = ({lwx:.4f}, {lwy:.4f})  "
      f"offset from base = {math.hypot(LX, LY):.4f} m\n")

rows = []
for (p, x, y, z, yaw, kind, dims) in items:
    if kind == "plane":
        continue
    # horizontal distance from the deadlock base_footprint to the element's
    # nearest face (axis-aligned approximation in the element's own frame)
    dx, dy = x - PX, y - PY
    d_centre = math.hypot(dx, dy)
    if kind == "box" and len(dims) == 3:
        hx, hy = dims[0] / 2.0, dims[1] / 2.0
        # rotate the query point into the box frame
        c, s = math.cos(-yaw), math.sin(-yaw)
        bx = dx * c - dy * s
        by = dx * s + dy * c
        ox = max(abs(bx) - hx, 0.0)
        oy = max(abs(by) - hy, 0.0)
        d_face = math.hypot(ox, oy)
        top = z + dims[2] / 2.0
    elif kind == "cylinder":
        d_face = max(d_centre - dims[0], 0.0)
        top = z + dims[1] / 2.0
    else:
        continue
    if d_face < 1.5:
        rows.append((d_face, d_centre, top, p, kind, dims, x, y, z, yaw))

rows.sort()
print(f"{'d_face':>7} {'d_ctr':>7} {'top_z':>7}  {'kind':<9} {'dims':<22} "
      f"{'centre(x,y,z)':<26} path")
for (d_face, d_centre, top, p, kind, dims, x, y, z, yaw) in rows[:30]:
    dd = ",".join(f"{v:g}" for v in dims)
    ctr = f"({x:.3f},{y:.3f},{z:.3f})"
    flag = "  <== INSIDE STOP CIRCLE" if d_face < STOP_R else ""
    print(f"{d_face:7.4f} {d_centre:7.4f} {top:7.3f}  {kind:<9} {dd:<22} {ctr:<26} {p}{flag}")
