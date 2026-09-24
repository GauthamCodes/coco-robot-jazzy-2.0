"""Jazzy-side counterpart of ros_probe.py. /opt/ros/jazzy only."""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
OUT = sys.argv[2] if len(sys.argv) > 2 else 'jazzy_result.json'

rclpy.init()
node = rclpy.create_node('coco_isaac_probe')
st = {k: {'n': 0, 't_first': None, 't_last': None} for k in
      ('clock', 'odom', 'tf', 'rgb', 'depth', 'scan')}
sample = {}
T0 = time.time()


def seen(k, extra=None):
    s = st[k]
    s['n'] += 1
    now = time.time() - T0
    if s['t_first'] is None:
        s['t_first'] = round(now, 2)
        if extra:
            sample[k + '_first'] = extra
    s['t_last'] = round(now, 2)


def on_clock(m):
    seen('clock')
    sample['clock_last'] = m.clock.sec + m.clock.nanosec * 1e-9


def on_odom(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    d = {'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw_deg': round(yaw, 1),
         'vx': round(m.twist.twist.linear.x, 3), 'wz': round(m.twist.twist.angular.z, 3),
         'frame': m.header.frame_id, 'child': m.child_frame_id}
    seen('odom', d)
    sample['odom_last'] = d


def on_tf(m):
    for t in m.transforms:
        seen('tf', {'parent': t.header.frame_id, 'child': t.child_frame_id})
        sample['tf_last'] = {'parent': t.header.frame_id, 'child': t.child_frame_id,
                             'x': round(t.transform.translation.x, 3)}


def img(k):
    def cb(m):
        d = {'w': m.width, 'h': m.height, 'encoding': m.encoding, 'step': m.step,
             'bytes': len(m.data), 'frame': m.header.frame_id}
        import numpy as np
        if m.encoding == '32FC1':
            a = np.frombuffer(bytes(m.data), dtype=np.float32)
            fin = a[np.isfinite(a)]
            d['finite'] = int(fin.size)
            if fin.size:
                d['min'] = round(float(fin.min()), 3)
                d['max'] = round(float(fin.max()), 3)
        else:
            d['nonzero_bytes'] = int(np.count_nonzero(np.frombuffer(bytes(m.data), dtype=np.uint8)))
        seen(k, d)
        sample[k + '_last'] = d
    return cb


def on_scan(m):
    fin = [r for r in m.ranges if math.isfinite(r) and m.range_min <= r <= m.range_max]
    d = {'n': len(m.ranges), 'valid': len(fin), 'min': round(min(fin), 3) if fin else None,
         'angle_min': round(m.angle_min, 3), 'angle_max': round(m.angle_max, 3),
         'frame': m.header.frame_id}
    seen('scan', d)
    sample['scan_last'] = d


q = qos_profile_sensor_data
node.create_subscription(Clock, '/clock', on_clock, q)
node.create_subscription(Odometry, '/isaac/odom', on_odom, q)
node.create_subscription(TFMessage, '/tf', on_tf, q)
import os  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
iq = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE) \
    if os.environ.get('IMG_RELIABLE') else q
node.create_subscription(Image, '/isaac/rgb', img('rgb'), iq)
node.create_subscription(Image, '/isaac/depth', img('depth'), iq)
node.create_subscription(LaserScan, '/isaac/scan', on_scan, q)
pub = node.create_publisher(Twist, '/isaac/cmd_vel', 10)

published = 0
while time.time() - T0 < DURATION:
    el = time.time() - T0
    tw = Twist()
    if 5.0 <= el < 13.0:
        tw.linear.x = 0.5
    elif 13.0 <= el < 19.0:
        tw.angular.z = 0.5
    pub.publish(tw)
    published += 1
    end = time.time() + 0.1
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)

res = {'duration_s': DURATION, 'twist_published': published, 'topics': st,
       'samples': sample,
       'graph_topics': sorted(n for n, _ in node.get_topic_names_and_types())}
print(json.dumps(res, indent=1))
open(OUT, 'w').write(json.dumps(res, indent=1))
node.destroy_node()
rclpy.shutdown()
