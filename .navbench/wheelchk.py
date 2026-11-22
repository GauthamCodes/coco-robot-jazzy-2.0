#!/usr/bin/env python3
"""Cross-check one wheel's transform and collision geometry by hand."""
import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node('wheelchk')
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
for _ in range(60):
    rclpy.spin_once(node, timeout_sec=0.1)

root = ET.fromstring(urdf['x'])
for name in ('wheel4', 'wheel3', 'chassis_link'):
    link = root.find(f"link[@name='{name}']")
    print(f'=== {name} URDF collision ===')
    for col in link.findall('collision'):
        org = col.find('origin')
        g = col.find('geometry')
        kind = list(g)[0].tag
        attrs = list(g)[0].attrib
        print(f'  geom {kind} {attrs}')
        print(f'  origin xyz={org.get("xyz") if org is not None else None} '
              f'rpy={org.get("rpy") if org is not None else None}')
    tf = buf.lookup_transform('base_footprint', name, rclpy.time.Time())
    t, q = tf.transform.translation, tf.transform.rotation
    print(f'  TF  xyz=({t.x:.4f}, {t.y:.4f}, {t.z:.4f})')
    print(f'  TF quat=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})')
    # where does the link's own +z axis point in base_footprint?
    R = [[1 - 2 * (q.y ** 2 + q.z ** 2), 2 * (q.x * q.y - q.z * q.w),
          2 * (q.x * q.z + q.y * q.w)],
         [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x ** 2 + q.z ** 2),
          2 * (q.y * q.z - q.x * q.w)],
         [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w),
          1 - 2 * (q.x ** 2 + q.y ** 2)]]
    for axis, vec in (('x', (1, 0, 0)), ('y', (0, 1, 0)), ('z', (0, 0, 1))):
        w = [sum(R[i][j] * vec[j] for j in range(3)) for i in range(3)]
        print(f'  link +{axis} -> base ({w[0]:+.3f}, {w[1]:+.3f}, '
              f'{w[2]:+.3f})')
    print()
node.destroy_node()
rclpy.shutdown()
