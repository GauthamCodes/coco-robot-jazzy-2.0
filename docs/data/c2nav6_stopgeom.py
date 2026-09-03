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
"""C2-NAV.6: WHICH returns are inside PolygonStop, not just how many.

`c2nav6_stopprobe.py` answers "how many", and on the baseline exit stall
the answer is exactly 6 on every one of 1470 STOP frames. Six is a
surprising number and the surprise is the point: the scan is 480 samples
over 240 deg, so beams are 2.2 mm apart at 0.25 m, and a flat wall
0.2445 m from the circle's centre cuts a 0.104 m chord out of a 0.25 m
circle -- which ought to return something like fifty beams, not six.

A count alone cannot distinguish "the obstacle is small" from "the
obstacle is large and the sensor can only see a sliver of it". This dumps
the individual returns so the difference is visible: for every valid beam
it prints the beam index, its bearing in the LIDAR frame, its range, and
where it lands relative to the base_footprint origin the circle is
centred on. Then it reports the run of indices that fall inside.

Subscribe-only, sends nothing, and needs no Nav2 -- only the simulator
and the scan. It is meant to be run against a robot already parked in the
pose under diagnosis.

Usage:
  python3 c2nav6_stopgeom.py <out.json>
"""
import json
import math
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

STOP_RADIUS = 0.25
# base_footprint <- lidar_link, read off live TF in this session's
# baseline run and identical to the URDF joint: (-0.09, +0.10), no
# rotation. Hard-coded here only because this tool is meant to run
# against a torn-down Nav2 stack where the TF tree may already be gone;
# c2nav6_stopprobe.py looks it up instead and printed this exact pair.
LIDAR_XY = (-0.09, 0.10)


class Grab(Node):

    def __init__(self):
        super().__init__('c2nav6_stopgeom')
        self.msg = None
        self.lock = threading.Lock()
        self.create_subscription(
            LaserScan, '/scan', self._cb, qos_profile_sensor_data)

    def _cb(self, m):
        with self.lock:
            self.msg = m


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/c2nav6_geom.json'
    rclpy.init()
    node = Grab()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    t_end = time.time() + 20.0
    while time.time() < t_end:
        with node.lock:
            m = node.msg
        if m is not None:
            break
        time.sleep(0.2)
    else:
        print('[stopgeom] NO SCAN')
        return 1

    tx, ty = LIDAR_XY
    a = m.angle_min
    pts = []
    for i, r in enumerate(m.ranges):
        if m.range_min <= r <= m.range_max:
            xb = tx + r * math.cos(a)
            yb = ty + r * math.sin(a)
            d = math.hypot(xb, yb)
            pts.append({'i': i, 'bearing_lidar_rad': round(a, 5),
                        'bearing_lidar_deg': round(math.degrees(a), 3),
                        'range_m': round(r, 4),
                        'x_base': round(xb, 4), 'y_base': round(yb, 4),
                        'd_base_m': round(d, 4),
                        'inside': d < STOP_RADIUS})
        a += m.angle_increment
    inside = [p for p in pts if p['inside']]
    doc = {
        'frame_id': m.header.frame_id,
        'n_samples': len(m.ranges),
        'angle_min_rad': round(m.angle_min, 5),
        'angle_max_rad': round(m.angle_max, 5),
        'angle_increment_rad': round(m.angle_increment, 7),
        'range_min_m': m.range_min, 'range_max_m': m.range_max,
        'lidar_xy_in_base': list(LIDAR_XY),
        'stop_radius_m': STOP_RADIUS,
        'n_valid': len(pts),
        'n_inside': len(inside),
        'inside': inside,
        'nearest_valid': min(pts, key=lambda p: p['d_base_m']) if pts else None,
    }
    if inside:
        idx = [p['i'] for p in inside]
        doc['inside_index_span'] = [min(idx), max(idx)]
        doc['inside_indices_contiguous'] = (max(idx) - min(idx) + 1) == len(idx)
        doc['fov_edge_indices'] = [0, len(m.ranges) - 1]
        doc['inside_touches_fov_edge'] = (
            min(idx) == 0 or max(idx) == len(m.ranges) - 1)
    with open(out, 'w') as f:
        json.dump(doc, f, indent=1)
    print(f'[stopgeom] {len(pts)} valid returns, {len(inside)} inside the '
          f'{STOP_RADIUS} m circle -> {out}')
    for p in inside:
        print(f'  i={p["i"]:4d} bearing={p["bearing_lidar_deg"]:8.3f} deg '
              f'range={p["range_m"]:.4f} base=({p["x_base"]:+.4f}, '
              f'{p["y_base"]:+.4f}) d={p["d_base_m"]:.4f}')
    if inside:
        print(f'  index span {doc["inside_index_span"]}, contiguous='
              f'{doc["inside_indices_contiguous"]}, touches FOV edge='
              f'{doc["inside_touches_fov_edge"]}')
    ex.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
