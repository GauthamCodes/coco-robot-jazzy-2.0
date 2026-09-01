#!/usr/bin/env python3
"""Measure the robot's real planar extent from live TF + the URDF.

Section 8 of the C2-NAV.0 brief asks whether robot_radius: 0.20 is
conservative, realistic or too permissive.

Doing this by eye from the xacro is a trap: chassis_link's collision box
is authored as "0.24 0.06 0.274" in a frame that is ROLLED 90 degrees, so
its nominal y is the robot's z, and its collision origin carries a large
offset that is easy to double-count. So this transforms the eight actual
CORNERS of every collision box (through the collision origin's own
rotation, then through the link's TF) into base_footprint and takes the
true planar hull. Cylinders are treated as their bounding box about the
cylinder axis, which is exact for the wheels here.

The number robot_radius must cover is the circumscribed radius. The
number that decides whether a gap is passable is the half-width.
"""
import math
import sys
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener


def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def quat_matrix(x, y, z, w):
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def mul(m, v):
    return [sum(m[i][j] * v[j] for j in range(3)) for i in range(3)]


def collision_boxes(urdf_xml):
    """{link: [(half_extents, origin_xyz, origin_rpy), ...]}"""
    root = ET.fromstring(urdf_xml)
    out = {}
    for link in root.findall('link'):
        boxes = []
        for col in link.findall('collision'):
            geom = col.find('geometry')
            if geom is None:
                continue
            box, cyl, sph = (geom.find('box'), geom.find('cylinder'),
                             geom.find('sphere'))
            if box is not None:
                sx, sy, sz = (float(v) for v in box.get('size').split())
                half = ('box', sx / 2, sy / 2, sz / 2)
            elif cyl is not None:
                # NOT a bounding box: treating a cylinder as its bounding
                # box inflates its radial extent by r*(sqrt(2)-1), which
                # for these 58.5 mm wheels is 24 mm -- enough to change
                # the verdict on robot_radius. Sample the rim instead.
                rr, ll = float(cyl.get('radius')), float(cyl.get('length'))
                half = ('cyl', rr, rr, ll / 2)
            elif sph is not None:
                rr = float(sph.get('radius'))
                half = ('box', rr, rr, rr)
            else:
                continue
            org = col.find('origin')
            xyz, rpy = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
            if org is not None:
                if org.get('xyz'):
                    xyz = tuple(float(v) for v in org.get('xyz').split())
                if org.get('rpy'):
                    rpy = tuple(float(v) for v in org.get('rpy').split())
            boxes.append((half, xyz, rpy))
        if boxes:
            out[link.get('name')] = boxes
    return out


def main():
    rclpy.init()
    node = Node('footprint_probe')
    buf = Buffer()
    TransformListener(buf, node)
    urdf = {}

    node.create_subscription(
        String, '/robot_description', lambda m: urdf.setdefault('x', m.data),
        QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                   reliability=QoSReliabilityPolicy.RELIABLE,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
    for _ in range(150):
        rclpy.spin_once(node, timeout_sec=0.1)
        if urdf:
            break
    if not urdf:
        print('no /robot_description')
        return 1
    boxes = collision_boxes(urdf['x'])
    for _ in range(60):
        rclpy.spin_once(node, timeout_sec=0.1)

    rows = []
    for link, blist in boxes.items():
        try:
            tf = buf.lookup_transform('base_footprint', link,
                                      rclpy.time.Time())
        except Exception:
            continue
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_matrix(q.x, q.y, q.z, q.w)
        pts = []
        for (half, xyz, rpy) in blist:
            Rc = rpy_matrix(*rpy)
            kind, hx, hy, hz = half
            locals_ = []
            if kind == 'box':
                for sx in (-1, 1):
                    for sy in (-1, 1):
                        for sz in (-1, 1):
                            locals_.append([sx * hx, sy * hy, sz * hz])
            else:                      # cylinder: rim of both end caps
                for k in range(64):
                    a = 2 * math.pi * k / 64
                    for sz in (-1, 1):
                        locals_.append([hx * math.cos(a),
                                        hy * math.sin(a), sz * hz])
            for local in locals_:
                p = mul(Rc, local)
                p = [p[0] + xyz[0], p[1] + xyz[1], p[2] + xyz[2]]
                p = mul(R, p)
                pts.append((p[0] + t.x, p[1] + t.y, p[2] + t.z))
        # Ignore geometry entirely above the tallest thing a 0.25 m-high
        # planar lidar or a 1.0 m wall can interact with? No -- Nav2 plans
        # in 2D and cares about the full projection, so keep everything.
        cr = max(math.hypot(px, py) for (px, py, _) in pts)
        hw = max(abs(py) for (_, py, _) in pts)
        fx = max(px for (px, _, _) in pts)
        bx = min(px for (px, _, _) in pts)
        rows.append((link, cr, hw, fx, bx))

    rows.sort(key=lambda r: -r[1])
    print(f'{"link":<20}{"circ_r":>9}{"half_w":>9}{"front_x":>9}{"back_x":>9}')
    for (n, cr, hw, fx, bx) in rows:
        print(f'{n:<20}{cr:9.4f}{hw:9.4f}{fx:9.4f}{bx:9.4f}')

    circ = max(r[1] for r in rows)
    halfw = max(r[2] for r in rows)
    front = max(r[3] for r in rows)
    back = min(r[4] for r in rows)
    print(f'\ncircumscribed radius : {circ:.4f} m   (driven by '
          f'{max(rows, key=lambda r: r[1])[0]})')
    print(f'half-width           : {halfw:.4f} m  -> full width '
          f'{2 * halfw:.4f} m')
    print(f'length               : {front - back:.4f} m '
          f'(x {back:+.4f} .. {front:+.4f})')
    print(f'\nnav2 robot_radius    : 0.2000 m')
    d = 0.200 - circ
    print(f'  vs circumscribed   : {d:+.4f} m  -> '
          f'{"covers the robot" if d >= 0 else "SMALLER THAN THE ROBOT"}')
    print(f'  vs half-width      : {0.200 - halfw:+.4f} m')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
