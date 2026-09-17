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
"""C2-NAV.43 -- does vertical geometry change the local representation?

The robot already carries the sensor: coco_robo2.xacro's rgbd_camera is
bridged as /camera/depth/image_raw (32FC1), /camera/camera_info and
/camera/points (PointCloud2). This instrument checks that sensor on a live
simulator and measures what a depth source does to a Nav2 costmap, against
the world file's geometry. Nothing here commands velocity.

  sensors  --out DIR [--duration S]
      Part I, at whatever pose the robot is in. /scan, the depth image,
      camera_info and /camera/points: arrival, rate, stamps advancing,
      encoding, finite fraction, frame_id; TF base_footprint <- sensor
      frames against the xacro; and two geometry checks that need no
      assumption about the cloud's frame convention:
        convention  the organized cloud against a pinhole projection of the
                    depth image in the OPTICAL convention (z forward) and in
                    the x-forward LINK convention. Whichever matches is the
                    frame the points are really in, whatever frame_id says.
        projection  every finite point, through TF and ground truth, into
                    the world: ground points must land at z ~ 0, points on
                    the ramp at the wedge's surface height, points on a box
                    no higher than the box.

  capture  --out DIR --lidar-params F --fused-params F [--poses a,b,...]
      Part G. Starts THREE standalone nav2_costmap_2d nodes built from the
      local_costmap block of the two parameter files, inflation removed so a
      cell is a mark or not:
        lidar  the shipped local costmap: obstacle_layer + voxel_layer on /scan
        depth  voxel_layer on the depth source only
        fused  the fusion local costmap: obstacle_layer on /scan, voxel_layer
               on scan + depth
      They run side by side, so at each pose all three see the same sensor
      messages. The robot is teleported (gz set_pose) to each named pose,
      the costmaps are cleared, and after SETTLE_S of sim time every marked
      cell is mapped into the world (ground truth composed with the odom TF)
      and classified against the world geometry: which obstacle it belongs
      to, 'ramp', or 'phantom' (farther than CLASS_TOL from any geometry).

  record   --out DIR
      Part K, passive, alongside a tour: every /local_costmap/costmap message,
      lethal cells classified the same way, and the Nav2 container's CPU.

  selftest
      Geometry, voxel decoding, frame composition and the convention test on
      synthetic data. No ROS.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'coco_config'))
from coco_config import robot as ROBOT  # noqa: E402  ramp geometry, one source of truth

import numpy as np  # noqa: E402

# --- world geometry (gazebo_models/worlds/coco_world.world, world frame) ------
# (name, centre_x, centre_y, size_x, size_y, height). Walls and boxes are the
# same list c2nav7_geom.BOXES uses, plus the heights and the two pilasters.
BOXES = (
    ('wall_north', 2.0, 3.5, 12.0, 0.2, 1.0),
    ('wall_south', 2.0, -3.5, 12.0, 0.2, 1.0),
    ('wall_west', -4.0, 0.0, 0.2, 7.2, 1.0),
    ('wall_east', 8.0, 0.0, 0.2, 7.2, 1.0),
    ('box_obstacle_1', -3.0, 2.4, 0.5, 0.5, 0.5),
    ('box_obstacle_2', 0.8, -1.4, 0.5, 0.5, 0.5),
    ('gate_cube_north', -1.1, 1.05, 0.5, 0.5, 0.5),
    ('gate_cube_south', -1.1, -0.75, 0.5, 0.5, 0.5),
    ('feature_pilaster_north', 7.72, 2.0, 0.3, 0.5, 0.6),
    ('feature_pilaster_south', 7.72, -1.4, 0.3, 0.5, 0.6),
)
# (name, centre_x, centre_y, radius, height)
CYLINDERS = (('cylinder_obstacle', -0.2, 0.6, 0.2, 0.6),)
# The launch file's default wedge (full_world_robo.launch.py ramp_angle).
RAMP = {
    'x0': ROBOT.RAMP_FOOT_X, 'x1': ROBOT.RAMP_SUMMIT_X,
    'y0': -ROBOT.RAMP_WIDTH / 2.0, 'y1': ROBOT.RAMP_WIDTH / 2.0,
    'grade_rad': math.radians(ROBOT.RAMP_ANGLE_DEG),
}
LIDAR_Z = 0.20 + 0.0135  # lidar_joint z + base_footprint_joint z (xacro)

# A marked cell is attributed to the nearest geometry within this distance.
# 0.05 m cells put a cell centre up to 0.035 m from the surface it marks;
# the rest is scan/depth noise and the odom->world composition.
CLASS_TOL = 0.10
SETTLE_S = 4.0
RANGE_M = 2.5            # the local costmap's obstacle_max_range

# Poses for Part G: (x, y, yaw) in the world frame.
POSES = {
    'ramp_entrance': (0.30, 0.00, 0.0),
    'ramp_surface': (0.55, 0.50, 0.0),
    'ramp_edge': (2.00, -2.00, math.pi / 2),
    'box_obstacles': (-2.00, -0.75, 0.0),
    'enclosure_entrance': (-3.55, 1.55, math.pi / 2),
    'enclosure_exit': (-3.45, 2.95, -math.pi / 2),
}

CAPTURE_ARMS = ('lidar', 'depth', 'fused')
NS = {'lidar': 'c2n43_lidar', 'depth': 'c2n43_depth', 'fused': 'c2n43_fused'}


def ramp_height(x, y):
    """Wedge surface height at (x, y); 0 off the wedge. Vectorised."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    on = (x >= RAMP['x0']) & (x <= RAMP['x1']) & (y >= RAMP['y0']) & (y <= RAMP['y1'])
    return np.where(on, (x - RAMP['x0']) * math.tan(RAMP['grade_rad']), 0.0)


def _rect_dist(x, y, x0, x1, y0, y1):
    dx = np.maximum(np.maximum(x0 - x, 0.0), x - x1)
    dy = np.maximum(np.maximum(y0 - y, 0.0), y - y1)
    return np.hypot(dx, dy)


def geometry_distance(x, y):
    """(distance, index) to the nearest footprint; names() maps the index.

    Index order: BOXES, then CYLINDERS, then the ramp. Vectorised.
    """
    x = np.atleast_1d(np.asarray(x, dtype=float))
    y = np.atleast_1d(np.asarray(y, dtype=float))
    cols = []
    for _, cx, cy, sx, sy, _ in BOXES:
        cols.append(_rect_dist(x, y, cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2))
    for _, cx, cy, r, _ in CYLINDERS:
        cols.append(np.maximum(np.hypot(x - cx, y - cy) - r, 0.0))
    cols.append(_rect_dist(x, y, RAMP['x0'], RAMP['x1'], RAMP['y0'], RAMP['y1']))
    d = np.stack(cols, axis=0)
    idx = np.argmin(d, axis=0)
    return d[idx, np.arange(d.shape[1])], idx


def names():
    return [b[0] for b in BOXES] + [c[0] for c in CYLINDERS] + ['ramp']


def geometry_height(idx):
    heights = [b[5] for b in BOXES] + [c[4] for c in CYLINDERS] + [None]
    return heights[idx]


def classify(x, y, tol=CLASS_TOL):
    """Label per point: a geometry name, or 'phantom'. Vectorised."""
    d, idx = geometry_distance(x, y)
    labels = np.array(names(), dtype=object)[idx]
    labels[d > tol] = 'phantom'
    return labels, d


def yaw_of(q):
    """Yaw of a quaternion given as (x, y, z, w)."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def compose2d(a, b):
    """a o b for (x, y, yaw) poses."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, math.atan2(math.sin(at + bt), math.cos(at + bt)))


def inverse2d(a):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (-(c * x + s * y), -(-s * x + c * y), -t)


def world_from_odom(gt_world_base, odom_base):
    """world_T_odom = world_T_base o inv(odom_T_base), all (x, y, yaw)."""
    return compose2d(gt_world_base, inverse2d(odom_base))


def grid_marks(data, width, height, res, origin_xy, lethal=100):
    """Cell centres (grid frame) of cells whose value equals `lethal`."""
    arr = np.asarray(data, dtype=np.int16).reshape(height, width)
    j, i = np.nonzero(arr == lethal)
    return origin_xy[0] + (i + 0.5) * res, origin_xy[1] + (j + 0.5) * res, i, j


def voxel_top(data, size_x, size_y, size_z, origin_z, z_res):
    """Top of the highest MARKED voxel per column (NaN where none).

    nav2_voxel_grid (voxel_grid.hpp) packs a column into one uint32 with two
    bits per voxel k, bit k and bit k+16: both set = MARKED, one set = UNKNOWN
    (reset leaves the low 16 bits set), none = FREE. Counting the low bit alone
    reads every unknown voxel as marked -- the first version of this function
    did, and put every column's top at z_voxels * z_res.
    """
    cols = np.asarray(data, dtype=np.uint64).reshape(size_y, size_x)
    top = np.full(cols.shape, np.nan)
    for k in range(size_z):
        marked = ((cols >> k) & 1) & ((cols >> (k + 16)) & 1)
        top = np.where(marked, origin_z + (k + 1) * z_res, top)
    return top


def pinhole_convention(depth, K, cloud_xyz, stride=8):
    """Compare an organized cloud with the depth image under both conventions.

    depth: (H, W) z-depth; K: (fx, fy, cx, cy); cloud_xyz: (H, W, 3).
    Returns median point error (m) for the optical and the link convention.
    """
    fx, fy, cx, cy = K
    h, w = depth.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    d = depth[v, u]
    c = cloud_xyz[v, u]
    ok = np.isfinite(d) & np.all(np.isfinite(c), axis=-1) & (d > 0)
    if not np.any(ok):
        return {'pixels': 0, 'optical_median_m': None, 'link_median_m': None}
    d, u, v, c = d[ok], u[ok], v[ok], c[ok]
    xo = (u - cx) * d / fx
    yo = (v - cy) * d / fy
    optical = np.stack([xo, yo, d], axis=-1)
    link = np.stack([d, -xo, -yo], axis=-1)
    return {
        'pixels': int(ok.sum()),
        'optical_median_m': float(np.median(np.linalg.norm(c - optical, axis=-1))),
        'link_median_m': float(np.median(np.linalg.norm(c - link, axis=-1))),
    }


def projection_stats(p_world, origin_world, range_m=RANGE_M, tol=CLASS_TOL):
    """Where points land in the world, against the geometry they should be on."""
    p = p_world[np.all(np.isfinite(p_world), axis=1)]
    rng = np.linalg.norm(p[:, :2] - np.asarray(origin_world[:2]), axis=1)
    p = p[rng <= range_m]
    out = {'points': int(len(p))}
    if not len(p):
        return out
    d, idx = geometry_distance(p[:, 0], p[:, 1])
    nm = np.array(names(), dtype=object)[idx]
    ground = d > tol
    gz = p[ground, 2]
    out['ground'] = _zstats(gz)
    ramp = (nm == 'ramp') & (d == 0.0)
    if np.any(ramp):
        resid = p[ramp, 2] - ramp_height(p[ramp, 0], p[ramp, 1])
        # A point on the wedge's side face or foot region is below the
        # surface; the surface residual is taken over points within 0.10 m of it.
        near = np.abs(resid) <= 0.10
        out['ramp'] = {'points': int(ramp.sum()),
                       'near_surface': int(near.sum()),
                       'surface_residual': _zstats(resid[near]),
                       'above_surface_gt_0.05': int((resid > 0.05).sum())}
    objs = {}
    for name in set(nm[(d <= tol) & (nm != 'ramp')]):
        sel = (nm == name) & (d <= tol)
        top = geometry_height(names().index(name))
        objs[name] = {'points': int(sel.sum()),
                      'max_z': round(float(p[sel, 2].max()), 4),
                      'height': top,
                      'above_top_gt_0.05': int((p[sel, 2] > top + 0.05).sum())}
    out['objects'] = objs
    return out


def _zstats(z):
    z = np.asarray(z, dtype=float)
    if not len(z):
        return {'n': 0}
    a = np.abs(z)
    return {'n': int(len(z)), 'median': round(float(np.median(z)), 4),
            'p01': round(float(np.percentile(z, 1)), 4),
            'p99': round(float(np.percentile(z, 99)), 4),
            'abs_p99': round(float(np.percentile(a, 99)), 4),
            'abs_max': round(float(a.max()), 4),
            'frac_abs_gt_0.03': round(float((a > 0.03).mean()), 5),
            'frac_abs_gt_0.05': round(float((a > 0.05).mean()), 5)}


def mark_summary(xw, yw, robot_xy, tops=None):
    """Classify world-frame marks and summarise them per geometry."""
    xw = np.asarray(xw, dtype=float)
    yw = np.asarray(yw, dtype=float)
    out = {'marked': int(len(xw))}
    if not len(xw):
        out.update({'labels': {}, 'phantoms': [], 'objects': {}})
        return out
    labels, d = classify(xw, yw)
    rng = np.hypot(xw - robot_xy[0], yw - robot_xy[1])
    counts = {}
    for lab in labels:
        counts[lab] = counts.get(lab, 0) + 1
    out['labels'] = dict(sorted(counts.items()))
    ph = labels == 'phantom'
    out['phantoms'] = [[round(float(a), 3), round(float(b), 3), round(float(c), 3)]
                       for a, b, c in zip(xw[ph], yw[ph], d[ph])]
    objs = {}
    for lab in counts:
        if lab == 'phantom':
            continue
        sel = labels == lab
        entry = {'cells': int(sel.sum()),
                 'nearest_mark_range_m': round(float(rng[sel].min()), 3)}
        if tops is not None:
            t = np.asarray(tops, dtype=float)[sel]
            t = t[np.isfinite(t)]
            if len(t):
                entry['voxel_top_max_m'] = round(float(t.max()), 3)
                entry['voxel_top_median_m'] = round(float(np.median(t)), 3)
        if lab == 'ramp':
            entry['min_x'] = round(float(xw[sel].min()), 3)
            entry['max_x'] = round(float(xw[sel].max()), 3)
            h = ramp_height(xw[sel], yw[sel])
            entry['surface_height_at_marks_min_m'] = round(float(h.min()), 3)
            entry['surface_height_at_marks_max_m'] = round(float(h.max()), 3)
        objs[lab] = entry
    out['objects'] = objs
    return out


def true_range(robot_xy, name):
    """Range from the robot to a named geometry's footprint."""
    d, idx = geometry_distance([robot_xy[0]], [robot_xy[1]])
    all_d = []
    for i, nm in enumerate(names()):
        if nm == name:
            if i < len(BOXES):
                _, cx, cy, sx, sy, _ = BOXES[i]
                all_d.append(float(_rect_dist(np.array([robot_xy[0]]), np.array([robot_xy[1]]),
                                              cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2)[0]))
            elif i < len(BOXES) + len(CYLINDERS):
                _, cx, cy, r, _ = CYLINDERS[i - len(BOXES)]
                all_d.append(max(math.hypot(robot_xy[0] - cx, robot_xy[1] - cy) - r, 0.0))
            else:
                all_d.append(float(_rect_dist(np.array([robot_xy[0]]), np.array([robot_xy[1]]),
                                              RAMP['x0'], RAMP['x1'], RAMP['y0'], RAMP['y1'])[0]))
    return round(all_d[0], 3) if all_d else None


# --- ROS side ---------------------------------------------------------------

def _ros_imports():
    global rclpy, Node, MultiThreadedExecutor, qos, tf2_ros, pc2
    global Image, CameraInfo, LaserScan, PointCloud2, Odometry, OccupancyGrid
    global VoxelGrid, ChangeState, Transition, ClearEntireCostmap
    import rclpy  # noqa: F811
    from rclpy.node import Node  # noqa: F811
    from rclpy.executors import MultiThreadedExecutor  # noqa: F811
    import rclpy.qos as qos  # noqa: F811
    import tf2_ros  # noqa: F811
    import sensor_msgs_py.point_cloud2 as pc2  # noqa: F811
    from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2  # noqa: F811
    from nav_msgs.msg import Odometry, OccupancyGrid  # noqa: F811
    from nav2_msgs.msg import VoxelGrid  # noqa: F811
    from nav2_msgs.srv import ClearEntireCostmap  # noqa: F811
    from lifecycle_msgs.srv import ChangeState  # noqa: F811
    from lifecycle_msgs.msg import Transition  # noqa: F811


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Probe:
    """One node, spun by its own executor thread; callbacks store messages."""

    BEST_EFFORT = None

    def __init__(self, name):
        _ros_imports()
        rclpy.init()
        self.node = Node(name, parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        self.lock = threading.Lock()
        self.latest = {}
        self.series = {}
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.gt = []
        self.exec = MultiThreadedExecutor(num_threads=4)
        self.exec.add_node(self.node)
        self.thread = threading.Thread(target=self.exec.spin, daemon=True)
        self.thread.start()

    def qos_any(self, depth=5):
        return qos.QoSProfile(depth=depth, reliability=qos.ReliabilityPolicy.BEST_EFFORT,
                              durability=qos.DurabilityPolicy.VOLATILE)

    def watch(self, key, msg_type, topic, keep_series=True, depth=5, on_msg=None):
        def cb(msg, key=key):
            with self.lock:
                self.latest[key] = msg
                if keep_series:
                    self.series.setdefault(key, []).append(
                        (time.monotonic(), _stamp(msg), getattr(msg.header, 'frame_id', '')))
            if on_msg is not None:
                on_msg(msg)
        self.node.create_subscription(msg_type, topic, cb, self.qos_any(depth))

    def watch_gt(self):
        def cb(msg):
            p = msg.pose.pose
            with self.lock:
                self.gt.append((_stamp(msg), p.position.x, p.position.y, p.position.z,
                                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)))
                if len(self.gt) > 3000:
                    del self.gt[:1000]
        self.node.create_subscription(Odometry, '/model/coco/odometry', cb, self.qos_any(50))

    def gt_at(self, t=None):
        with self.lock:
            if not self.gt:
                return None
            if t is None:
                return self.gt[-1]
            return min(self.gt, key=lambda g: abs(g[0] - t))

    def sim_now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def wait_sim(self, seconds, wall_budget=120.0):
        start = self.sim_now()
        deadline = time.monotonic() + wall_budget
        while self.sim_now() - start < seconds:
            if time.monotonic() > deadline:
                return False
            time.sleep(0.05)
        return True

    def lookup(self, target, source, t=None):
        stamp = rclpy.time.Time() if t is None else rclpy.time.Time(
            seconds=int(t), nanoseconds=int((t - int(t)) * 1e9), clock_type=rclpy.clock.ClockType.ROS_TIME)
        tf = self.tf_buffer.lookup_transform(target, source, stamp, timeout=rclpy.duration.Duration(seconds=0.5))
        tr, q = tf.transform.translation, tf.transform.rotation
        return (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w)

    def call(self, srv_type, name, request, timeout=10.0):
        cli = self.node.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(request)
        deadline = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def close(self):
        self.exec.shutdown()
        self.node.destroy_node()
        rclpy.try_shutdown()


def _rate(series):
    if not series or len(series) < 3:
        return None
    st = [s for _, s, _ in series]
    span = st[-1] - st[0]
    return round((len(st) - 1) / span, 2) if span > 0 else None


def _monotonic(series):
    st = [s for _, s, _ in series]
    return all(b > a for a, b in zip(st, st[1:]))


def _cloud_xyz(msg):
    arr = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=False)
    return np.asarray(arr, dtype=np.float64).reshape(-1, 3)


def _depth_array(msg):
    dt = np.dtype(np.float32).newbyteorder('>' if msg.is_bigendian else '<')
    return np.frombuffer(bytes(msg.data), dtype=dt).reshape(msg.height, msg.width)


def _xacro_expectations():
    # base_footprint_joint z 0.0135; camera_joint (0.125, 0, 0.055);
    # lidar_joint (-0.09, 0.10, 0.20); camera_optical_joint rpy (-pi/2, 0, -pi/2).
    return {
        'camera_optical_frame': {'t': (0.125, 0.0, 0.0685),
                                 'R': np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)},
        'lidar_link': {'t': (-0.09, 0.10, 0.2135), 'R': np.eye(3)},
    }


def sensor_checks(probe, duration, cloud_topic='/camera/points'):
    """Part I checks over `duration` wall seconds at the current pose."""
    probe.watch('scan', LaserScan, '/scan')
    probe.watch('depth', Image, '/camera/depth/image_raw')
    probe.watch('info', CameraInfo, '/camera/camera_info')
    probe.watch('points', PointCloud2, cloud_topic)
    probe.watch_gt()
    time.sleep(duration)
    out = {'duration_wall_s': duration}
    checks = []

    def chk(name, ok, detail):
        checks.append({'check': name, 'ok': bool(ok), 'detail': detail})

    with probe.lock:
        latest = dict(probe.latest)
        series = {k: list(v) for k, v in probe.series.items()}
    for key in ('scan', 'depth', 'info', 'points'):
        s = series.get(key, [])
        frames = sorted({f for _, _, f in s})
        out[key] = {'messages': len(s), 'rate_sim_hz': _rate(s), 'frame_ids': frames,
                    'stamps_strictly_increasing': _monotonic(s) if s else None}
        chk(f'{key} arrives', len(s) > 3, f'{len(s)} messages')
        chk(f'{key} stamps advance', bool(s) and _monotonic(s), '')

    scan = latest.get('scan')
    if scan is not None:
        r = np.asarray(scan.ranges, dtype=float)
        valid = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
        out['scan'].update({'samples': len(r), 'valid_frac': round(float(valid.mean()), 4),
                            'angle_min': round(scan.angle_min, 4), 'angle_max': round(scan.angle_max, 4),
                            'range_min': scan.range_min, 'range_max': scan.range_max})
        chk('scan frame is lidar_link', scan.header.frame_id == 'lidar_link', scan.header.frame_id)
        chk('scan has valid ranges', valid.any(), f"valid_frac {out['scan']['valid_frac']}")

    depth = latest.get('depth')
    info = latest.get('info')
    cloud = latest.get('points')
    if depth is not None:
        d = _depth_array(depth)
        fin = np.isfinite(d)
        out['depth'].update({'encoding': depth.encoding, 'width': depth.width, 'height': depth.height,
                             'finite_frac': round(float(fin.mean()), 4),
                             'finite_min_m': round(float(d[fin].min()), 4) if fin.any() else None,
                             'finite_max_m': round(float(d[fin].max()), 4) if fin.any() else None})
        chk('depth encoding is 32FC1', depth.encoding == '32FC1', depth.encoding)
        chk('depth has finite values', fin.any(), f"finite_frac {out['depth']['finite_frac']}")
        chk('depth frame is camera_optical_frame', depth.header.frame_id == 'camera_optical_frame',
            depth.header.frame_id)
    if info is not None:
        K = tuple(info.k)
        out['info'].update({'width': info.width, 'height': info.height,
                            'fx': K[0], 'fy': K[4], 'cx': K[2], 'cy': K[5],
                            'hfov_rad_from_K': round(2 * math.atan(info.width / (2 * K[0])), 4)
                            if K[0] else None})
        chk('camera_info frame matches depth', depth is not None and info.header.frame_id == depth.header.frame_id,
            info.header.frame_id)
    if cloud is not None:
        xyz = _cloud_xyz(cloud)
        fin = np.all(np.isfinite(xyz), axis=1)
        out['points'].update({'width': cloud.width, 'height': cloud.height,
                              'fields': [f.name for f in cloud.fields],
                              'finite_frac': round(float(fin.mean()), 4)})
        chk('points have finite values', fin.any(), f"finite_frac {out['points']['finite_frac']}")

    exp = _xacro_expectations()
    out['tf'] = {}
    for frame, want in exp.items():
        try:
            t, q = probe.lookup('base_footprint', frame)
            R = quat_matrix(q)
            terr = float(np.linalg.norm(np.asarray(t) - np.asarray(want['t'])))
            rerr = float(np.abs(R - want['R']).max())
            out['tf'][frame] = {'t': [round(v, 4) for v in t], 'q': [round(v, 5) for v in q],
                                't_err_m': round(terr, 5), 'R_err_max': round(rerr, 5)}
            chk(f'TF base_footprint <- {frame} matches the xacro', terr < 1e-3 and rerr < 1e-3,
                f't_err {terr:.5f} m, R_err {rerr:.5f}')
        except Exception as e:  # noqa: BLE001
            out['tf'][frame] = {'error': str(e)}
            chk(f'TF base_footprint <- {frame} resolves', False, str(e))

    if depth is not None and info is not None and cloud is not None:
        xyz = _cloud_xyz(cloud)
        s = depth.width // cloud.width if cloud.width else 0
        if (s >= 1 and cloud.height > 1 and depth.width == s * cloud.width
                and depth.height == s * cloud.height):
            # A cloud at 1/s resolution (nearest-neighbour: pixel u' = s*u) is
            # compared with every s-th depth pixel under intrinsics / s.
            K = (info.k[0] / s, info.k[4] / s, info.k[2] / s, info.k[5] / s)
            conv = pinhole_convention(_depth_array(depth)[::s, ::s], K,
                                      xyz.reshape(cloud.height, cloud.width, 3), stride=max(1, 8 // s))
            conv['cloud_scale'] = 1.0 / s
            conv['stamp_gap_s'] = round(abs(_stamp(cloud) - _stamp(depth)), 4)
            if conv['optical_median_m'] is None:
                conv['verdict'] = 'undetermined'
            elif conv['optical_median_m'] < 0.01 and conv['link_median_m'] > 0.05:
                conv['verdict'] = 'optical'
            elif conv['link_median_m'] < 0.01 and conv['optical_median_m'] > 0.05:
                conv['verdict'] = 'link'
            else:
                conv['verdict'] = 'undetermined'
        else:
            conv = {'verdict': 'unorganized cloud', 'cloud_hw': [cloud.height, cloud.width]}
        out['convention'] = conv
        chk(f'{cloud_topic} is in the frame its frame_id names (optical)', conv.get('verdict') == 'optical',
            json.dumps(conv))

    out['projection'] = {}
    gt = probe.gt_at()
    if gt is not None:
        world_base = (np.array(gt[1:4]), quat_matrix(gt[4]))
        out['gt_pose'] = [round(gt[1], 4), round(gt[2], 4), round(gt[3], 4), round(yaw_of(gt[4]), 4)]
        for key, cloud_key in (('cloud_as_labelled', 'points'), ('depth_pinhole_optical', 'depth')):
            try:
                if cloud_key == 'points' and cloud is not None:
                    pts = _cloud_xyz(cloud)
                    frame = cloud.header.frame_id
                elif cloud_key == 'depth' and depth is not None and info is not None:
                    d = _depth_array(depth)
                    v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]]
                    K = (info.k[0], info.k[4], info.k[2], info.k[5])
                    pts = np.stack([(u - K[2]) * d / K[0], (v - K[3]) * d / K[1], d], axis=-1).reshape(-1, 3)
                    frame = depth.header.frame_id
                else:
                    continue
                t, q = probe.lookup('base_footprint', frame)
                p_base = pts @ quat_matrix(q).T + np.asarray(t)
                p_world = p_base @ world_base[1].T + world_base[0]
                out['projection'][key] = projection_stats(p_world, gt[1:3])
            except Exception as e:  # noqa: BLE001
                out['projection'][key] = {'error': str(e)}
        for key in ('cloud_as_labelled', 'depth_pinhole_optical'):
            pr = out['projection'].get(key, {})
            g = pr.get('ground', {})
            chk(f'{key}: ground points land at z ~ 0 (|z| p99 <= 0.03 m)',
                g.get('n', 0) > 0 and g.get('abs_p99', 1.0) <= 0.03, json.dumps(g))
            if 'ramp' in pr:
                # Points on the wedge's side faces sit BELOW the surface height
                # at their (x, y), so the gate is "nothing floats above the
                # wedge"; the surface residual is recorded alongside.
                chk(f'{key}: no ramp point above the wedge surface (> 0.05 m)',
                    pr['ramp']['above_surface_gt_0.05'] == 0, json.dumps(pr['ramp']))
            for name, o in pr.get('objects', {}).items():
                chk(f'{key}: {name} points no higher than it is', o['above_top_gt_0.05'] == 0,
                    json.dumps(o))
    out['checks'] = checks
    out['all_ok'] = all(c['ok'] for c in checks)
    return out


def sensor_checks_mode(args):
    os.makedirs(args.out, exist_ok=True)
    probe = Probe('c2nav43_sensors')
    try:
        if not probe.wait_sim(1.0, wall_budget=60.0):
            print('no /clock -- refusing', file=sys.stderr)
            return 3
        teleported = None
        if args.pose:
            x, y, yaw = POSES[args.pose]
            teleported = teleport(x, y, yaw)
            probe.wait_sim(3.0)
        res = sensor_checks(probe, args.duration, args.cloud_topic)
        res['cloud_topic'] = args.cloud_topic
        res['pose'] = args.pose or 'spawn'
        if teleported is not None:
            res['teleport_ok'] = teleported[0]
    finally:
        probe.close()
    name = 'sensors' + (f'_{args.pose}' if args.pose else '') + (f'_{args.tag}' if args.tag else '') + '.json'
    with open(os.path.join(args.out, name), 'w') as f:
        json.dump(res, f, indent=1)
    for c in res['checks']:
        print(f"{'ok  ' if c['ok'] else 'FAIL'} {c['check']}  {c['detail'][:160]}")
    print('ALL OK' if res['all_ok'] else 'SOME CHECKS FAILED')
    return 0 if res['all_ok'] else 1


def rates_mode(args):
    """Delivered message rate per topic, subscribed RAW (no deserialisation).

    Best-effort, keep-last 5: rclpy's qos_profile_sensor_data, the same policy
    nav2_costmap_2d's ObstacleLayer subscribes its sources with. A 1.2 MB
    cloud fragments on the wire and best effort drops a sample whose
    fragments do not all arrive, so a Python probe receiving few clouds could
    be transport loss or deserialisation cost; raw subscription removes the
    second.
    """
    os.makedirs(args.out, exist_ok=True)
    probe = Probe('c2nav43_rates')
    arrivals = {t: [] for t in args.topics}
    sizes = {t: [] for t in args.topics}
    try:
        from rosidl_runtime_py.utilities import get_message
        time.sleep(2.0)  # graph discovery
        names = dict(probe.node.get_topic_names_and_types())
        for topic in args.topics:
            if topic not in names:
                continue
            msg_type = get_message(names[topic][0])

            def cb(raw, topic=topic):
                arrivals[topic].append(time.monotonic())
                sizes[topic].append(len(raw))
            probe.node.create_subscription(msg_type, topic, cb, qos.qos_profile_sensor_data, raw=True)
        time.sleep(args.duration)
    finally:
        probe.close()
    out = {'duration_wall_s': args.duration, 'qos': 'sensor_data (best effort, keep last 5), raw', 'topics': {}}
    for topic in args.topics:
        a = arrivals[topic]
        out['topics'][topic] = {
            'messages': len(a),
            'wall_hz': round((len(a) - 1) / (a[-1] - a[0]), 2) if len(a) > 2 and a[-1] > a[0] else None,
            'bytes_median': int(np.median(sizes[topic])) if sizes[topic] else None,
        }
        print(f"{topic}: {out['topics'][topic]}")
    name = 'rates' + (f'_{args.tag}' if args.tag else '') + '.json'
    with open(os.path.join(args.out, name), 'w') as f:
        json.dump(out, f, indent=1)
    return 0


# --- capture ----------------------------------------------------------------

def _load_yaml(path):
    import yaml
    with open(path) as f:
        return yaml.safe_load(f)


def capture_params(lidar_params, fused_params):
    """{arm: ros__parameters} for the three capture costmaps.

    Taken from the local_costmap block of the two files; the inflation layer
    is dropped from every arm so a published cell is a mark (100) or not.
    """
    lid = _load_yaml(lidar_params)['local_costmap']['local_costmap']['ros__parameters']
    fus = _load_yaml(fused_params)['local_costmap']['local_costmap']['ros__parameters']

    def strip(p, plugins):
        q = json.loads(json.dumps(p))
        q['plugins'] = plugins
        q.pop('inflation_layer', None)
        q['use_sim_time'] = True
        return q
    arms = {
        'lidar': strip(lid, [x for x in lid['plugins'] if x != 'inflation_layer']),
        'fused': strip(fus, [x for x in fus['plugins'] if x != 'inflation_layer']),
    }
    depth = strip(fus, ['voxel_layer'])
    depth.pop('obstacle_layer', None)
    sources = fus['voxel_layer']['observation_sources'].split()
    extra = [s for s in sources if s != 'scan']
    if not extra:
        raise SystemExit('fused params carry no non-scan voxel source: nothing to capture')
    depth['voxel_layer']['observation_sources'] = ' '.join(extra)
    arms['depth'] = depth
    return arms


def teleport(x, y, yaw, z=0.02):
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    req = (f'name: "coco" position: {{x: {x} y: {y} z: {z}}} '
           f'orientation: {{x: 0 y: 0 z: {qz} w: {qw}}}')
    r = subprocess.run(['gz', 'service', '-s', '/world/coco_world/set_pose', '--reqtype', 'gz.msgs.Pose',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', req],
                       capture_output=True, text=True, timeout=20)
    return 'data: true' in r.stdout, (r.stdout + r.stderr).strip()


def capture_mode(args):
    import yaml
    os.makedirs(args.out, exist_ok=True)
    arms = capture_params(args.lidar_params, args.fused_params)
    procs = []
    for arm, params in arms.items():
        path = os.path.join(args.out, f'capture_{arm}.yaml')
        with open(path, 'w') as f:
            yaml.safe_dump({f'/{NS[arm]}/costmap': {'ros__parameters': params}}, f, sort_keys=False)
        log = open(os.path.join(args.out, f'costmap_{arm}.log'), 'w')
        procs.append(subprocess.Popen(
            ['ros2', 'run', 'nav2_costmap_2d', 'nav2_costmap_2d', '--ros-args',
             '-r', f'__ns:=/{NS[arm]}', '-r', '__node:=costmap', '--params-file', path],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
    probe = Probe('c2nav43_capture')
    result = {'lidar_params': os.path.abspath(args.lidar_params),
              'fused_params': os.path.abspath(args.fused_params),
              'arms': {a: {'plugins': p['plugins'],
                           'voxel_sources': p.get('voxel_layer', {}).get('observation_sources')}
                       for a, p in arms.items()},
              'class_tol_m': CLASS_TOL, 'settle_sim_s': SETTLE_S, 'poses': {}}
    rc = 0
    try:
        probe.watch_gt()
        probe.watch('depth', Image, '/camera/depth/image_raw', keep_series=False)
        probe.watch('info', CameraInfo, '/camera/camera_info', keep_series=False)
        probe.watch('points', PointCloud2, args.cloud_topic, keep_series=False)
        for arm in CAPTURE_ARMS:
            probe.watch(f'grid_{arm}', OccupancyGrid, f'/{NS[arm]}/costmap', keep_series=False, depth=1)
            probe.watch(f'voxel_{arm}', VoxelGrid, f'/{NS[arm]}/voxel_grid', keep_series=False, depth=1)
        if not probe.wait_sim(1.0, wall_budget=60.0):
            print('no /clock -- refusing', file=sys.stderr)
            return 3
        for arm in CAPTURE_ARMS:
            node = f'/{NS[arm]}/costmap/change_state'
            for tid in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                req = ChangeState.Request()
                req.transition.id = tid
                res = probe.call(ChangeState, node, req, timeout=30.0)
                if res is None or not res.success:
                    print(f'{node} transition {tid} failed', file=sys.stderr)
                    return 4
        poses = args.poses.split(',') if args.poses else list(POSES)
        for name in poses:
            x, y, yaw = POSES[name]
            ok, detail = teleport(x, y, yaw)
            probe.wait_sim(2.0)
            for arm in CAPTURE_ARMS:
                probe.call(ClearEntireCostmap, f'/{NS[arm]}/clear_entirely_costmap',
                           ClearEntireCostmap.Request(), timeout=10.0)
            t_clear = probe.sim_now()
            probe.wait_sim(SETTLE_S)
            entry = {'requested': [x, y, round(yaw, 4)], 'teleport_ok': ok, 'teleport_reply': detail[:200]}
            gt = probe.gt_at()
            if gt is None:
                entry['error'] = 'no ground truth'
                result['poses'][name] = entry
                rc = 1
                continue
            gt2d = (gt[1], gt[2], yaw_of(gt[4]))
            entry['gt'] = [round(v, 4) for v in gt2d]
            entry['gt_err_m'] = round(math.hypot(gt2d[0] - x, gt2d[1] - y), 4)
            try:
                t_ob, q_ob = probe.lookup('odom', 'base_footprint')
            except Exception as e:  # noqa: BLE001
                entry['error'] = f'odom tf: {e}'
                result['poses'][name] = entry
                rc = 1
                continue
            odom2d = (t_ob[0], t_ob[1], yaw_of(q_ob))
            w_o = world_from_odom(gt2d, odom2d)
            entry['world_T_odom'] = [round(v, 4) for v in w_o]
            entry['true_range_m'] = {n: true_range(gt2d[:2], n) for n in names()}
            with probe.lock:
                latest = dict(probe.latest)
            for arm in CAPTURE_ARMS:
                grid = latest.get(f'grid_{arm}')
                if grid is None or _stamp(grid) < t_clear + SETTLE_S - 1.0:
                    entry[arm] = {'error': 'no costmap published after the clear'}
                    rc = 1
                    continue
                info = grid.info
                gx, gy, gi, gj = grid_marks(grid.data, info.width, info.height, info.resolution,
                                            (info.origin.position.x, info.origin.position.y))
                c, s = math.cos(w_o[2]), math.sin(w_o[2])
                xw = w_o[0] + c * gx - s * gy
                yw = w_o[1] + s * gx + c * gy
                tops = None
                vox = latest.get(f'voxel_{arm}')
                if vox is not None and vox.size_x == info.width and vox.size_y == info.height:
                    top = voxel_top(vox.data, vox.size_x, vox.size_y, vox.size_z,
                                    vox.origin.z, vox.resolutions.z)
                    tops = top[gj, gi]
                entry[arm] = mark_summary(xw, yw, gt2d[:2], tops)
                entry[arm]['stamp'] = round(_stamp(grid), 3)
                entry[arm]['cells_xy'] = [[round(float(a), 3), round(float(b), 3)] for a, b in zip(xw, yw)]
            cloud = latest.get('points')
            if cloud is not None:
                try:
                    t, q = probe.lookup('base_footprint', cloud.header.frame_id)
                    pts = _cloud_xyz(cloud)
                    p_world = (pts @ quat_matrix(q).T + np.asarray(t)) @ quat_matrix(gt[4]).T + np.asarray(gt[1:4])
                    entry['cloud_projection'] = projection_stats(p_world, gt[1:3])
                except Exception as e:  # noqa: BLE001
                    entry['cloud_projection'] = {'error': str(e)}
            result['poses'][name] = entry
            print(render_pose(name, entry), flush=True)
    finally:
        probe.close()
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for p in procs:
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
    with open(os.path.join(args.out, 'capture.json'), 'w') as f:
        json.dump(result, f, indent=1)
    try:
        plot_capture(result, args.out)
    except Exception as e:  # noqa: BLE001
        print(f'plot failed: {e}', file=sys.stderr)
    return rc


def render_pose(name, entry):
    lines = [f"== {name}  gt {entry.get('gt')}  (requested {entry['requested']}, err {entry.get('gt_err_m')} m)"]
    for arm in CAPTURE_ARMS:
        a = entry.get(arm, {})
        if 'error' in a:
            lines.append(f'  {arm:6s} {a["error"]}')
            continue
        objs = ', '.join(f"{k}:{v['cells']}@{v['nearest_mark_range_m']}"
                         + (f"^{v['voxel_top_max_m']}" if 'voxel_top_max_m' in v else '')
                         for k, v in sorted(a.get('objects', {}).items()))
        lines.append(f"  {arm:6s} marked {a.get('marked')}  phantom {a.get('labels', {}).get('phantom', 0)}  {objs}")
    return '\n'.join(lines)


def plot_capture(result, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Rectangle
    for name, entry in result['poses'].items():
        if 'gt' not in entry:
            continue
        fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.8))
        gx, gy, gyaw = entry['gt']
        for ax, arm in zip(axes, CAPTURE_ARMS):
            for _, cx, cy, sx, sy, _ in BOXES:
                ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, color='0.75'))
            for _, cx, cy, r, _ in CYLINDERS:
                ax.add_patch(Circle((cx, cy), r, color='0.75'))
            ax.add_patch(Polygon([[RAMP['x0'], RAMP['y0']], [RAMP['x1'], RAMP['y0']],
                                  [RAMP['x1'], RAMP['y1']], [RAMP['x0'], RAMP['y1']]],
                                 closed=True, fill=False, hatch='//', color='0.55'))
            a = entry.get(arm, {})
            cells = np.asarray(a.get('cells_xy', []), dtype=float).reshape(-1, 2)
            if len(cells):
                lab, _ = classify(cells[:, 0], cells[:, 1])
                ph = lab == 'phantom'
                ax.scatter(cells[~ph, 0], cells[~ph, 1], s=5, marker='s', color='#1f5fae')
                ax.scatter(cells[ph, 0], cells[ph, 1], s=9, marker='s', color='#c0392b')
            ax.add_patch(Circle((gx, gy), 0.2051, fill=False, color='k'))
            ax.arrow(gx, gy, 0.3 * math.cos(gyaw), 0.3 * math.sin(gyaw), width=0.01, color='k')
            ax.add_patch(Rectangle((gx - 1.5, gy - 1.5), 3.0, 3.0, fill=False, ls='--', color='0.4'))
            ax.set_xlim(gx - 1.7, gx + 1.7)
            ax.set_ylim(gy - 1.7, gy + 1.7)
            ax.set_aspect('equal')
            ph_n = a.get('labels', {}).get('phantom', 0)
            ax.set_title(f"{arm}: {a.get('marked', 0)} marks, {ph_n} phantom", fontsize=10)
        fig.suptitle(f'C2-NAV.43 {name}: blue = mark on world geometry, red = phantom '
                     f'(> {CLASS_TOL} m from any), hatched = ramp footprint', fontsize=10)
        fig.tight_layout()
        fig.savefig(os.path.join(out, f'pose_{name}.png'), dpi=110)
        plt.close(fig)


# --- record -----------------------------------------------------------------

def _proc_cpu(pattern):
    """(pids, total utime+stime seconds) of processes whose cmdline contains pattern."""
    tick = os.sysconf('SC_CLK_TCK')
    pids, total = [], 0.0
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace')
            if pattern not in cmd or 'c2nav43_perception' in cmd:
                continue
            with open(f'/proc/{pid}/stat') as f:
                parts = f.read().rsplit(')', 1)[1].split()
            total += (int(parts[11]) + int(parts[12])) / tick
            pids.append(int(pid))
        except (OSError, IndexError, ValueError):
            continue
    return pids, total


def record_mode(args):
    os.makedirs(args.out, exist_ok=True)
    probe = Probe('c2nav43_record')
    stop = threading.Event()
    rows = []
    cpu = []

    def on_grid(msg):
        t = _stamp(msg)
        gt = probe.gt_at(t)
        if gt is None or abs(gt[0] - t) > 0.1:
            return
        try:
            t_ob, q_ob = probe.lookup('odom', 'base_footprint', t)
        except Exception:  # noqa: BLE001
            try:
                t_ob, q_ob = probe.lookup('odom', 'base_footprint')
            except Exception:  # noqa: BLE001
                return
        gt2d = (gt[1], gt[2], yaw_of(gt[4]))
        w_o = world_from_odom(gt2d, (t_ob[0], t_ob[1], yaw_of(q_ob)))
        info = msg.info
        gx, gy, _, _ = grid_marks(msg.data, info.width, info.height, info.resolution,
                                  (info.origin.position.x, info.origin.position.y))
        c, s = math.cos(w_o[2]), math.sin(w_o[2])
        xw = w_o[0] + c * gx - s * gy
        yw = w_o[1] + s * gx + c * gy
        labels, d = classify(xw, yw)
        ph = labels == 'phantom'
        ramp = labels == 'ramp'
        rows.append({'t': round(t, 3), 'x': round(gt2d[0], 3), 'y': round(gt2d[1], 3),
                     'marked': int(len(xw)), 'phantom': int(ph.sum()), 'ramp': int(ramp.sum()),
                     'phantom_max_d': round(float(d[ph].max()), 3) if ph.any() else 0.0,
                     'phantom_xy': [[round(float(a), 2), round(float(b), 2)]
                                    for a, b in zip(xw[ph][:20], yw[ph][:20])]})

    probe.watch_gt()
    probe.watch('grid', OccupancyGrid, '/local_costmap/costmap', keep_series=False, depth=2, on_msg=on_grid)

    # Process groups whose CPU is sampled: every Nav2 server (bringup composes
    # them into one container) and depth_cloud.launch.py's two nodes.
    groups = {'nav2_container': ('component_container_isolated',),
              'depth_cloud': ('image_proc/resize_node', 'depth_image_proc/point_cloud_xyz_node')}
    cpu_groups = {g: [] for g in groups}

    def sample_cpu():
        last = {}
        while not stop.is_set():
            now = time.monotonic()
            for g, patterns in groups.items():
                pids, tot = [], 0.0
                for pat in patterns:
                    p, t = _proc_cpu(pat)
                    pids += p
                    tot += t
                prev = last.get(g)
                if pids and prev is not None and set(pids) == set(prev[0]):
                    cpu_groups[g].append((now - prev[2], tot - prev[1]))
                    if g == 'nav2_container':
                        cpu.append((now - prev[2], tot - prev[1]))
                last[g] = (pids, tot, now) if pids else None
            stop.wait(2.0)
    th = threading.Thread(target=sample_cpu, daemon=True)
    th.start()

    def finish(*_):
        stop.set()
    signal.signal(signal.SIGINT, finish)
    signal.signal(signal.SIGTERM, finish)
    deadline = time.monotonic() + args.duration if args.duration else None
    while not stop.is_set():
        if deadline and time.monotonic() > deadline:
            break
        time.sleep(0.5)
    stop.set()
    th.join(timeout=5)
    probe.close()
    wall = sum(w for w, _ in cpu)
    summary = record_summary(rows)
    summary['nav2_container_cpu'] = {
        'wall_s': round(wall, 1),
        'mean_cores': round(sum(c for _, c in cpu) / wall, 3) if wall else None,
    }
    for g, samples in cpu_groups.items():
        w = sum(s for s, _ in samples)
        summary[f'cpu_{g}'] = {'wall_s': round(w, 1),
                               'mean_cores': round(sum(c for _, c in samples) / w, 3) if w else None}
    with open(os.path.join(args.out, 'record_rows.json'), 'w') as f:
        json.dump(rows, f)
    with open(os.path.join(args.out, 'record_summary.json'), 'w') as f:
        json.dump(summary, f, indent=1)
    print(json.dumps(summary, indent=1))
    return 0


def record_summary(rows):
    if not rows:
        return {'grids': 0}
    ph = [r['phantom'] for r in rows]
    return {
        'grids': len(rows),
        'grids_with_phantom': sum(1 for p in ph if p),
        'phantom_cells_total': sum(ph),
        'phantom_cells_max_in_one_grid': max(ph),
        'phantom_max_distance_m': max(r['phantom_max_d'] for r in rows),
        'marked_median': float(np.median([r['marked'] for r in rows])),
        'ramp_cells_max': max(r['ramp'] for r in rows),
        'grids_with_ramp_marks': sum(1 for r in rows if r['ramp']),
    }


# --- selftest ----------------------------------------------------------------

def selftest():
    n = fail = 0

    def chk(name, ok):
        nonlocal n, fail
        n += 1
        fail += 0 if ok else 1
        print(f"{'ok  ' if ok else 'FAIL'} {n:2d} {name}")

    chk('ramp foot is at world x 1.0 and the crest at 3.0',
        RAMP['x0'] == 1.0 and RAMP['x1'] == 3.0)
    h = float(ramp_height(3.0, 0.0))
    chk(f'18 deg wedge rises {h:.4f} m over 2.0 m', abs(h - 2.0 * math.tan(math.radians(18))) < 1e-9)
    chk('off the wedge the height is 0', float(ramp_height(0.5, 0.0)) == 0.0
        and float(ramp_height(2.0, 1.3)) == 0.0)
    x_scan = RAMP['x0'] + LIDAR_Z / math.tan(RAMP['grade_rad'])
    chk(f'the scan plane meets the ramp surface at x {x_scan:.3f}', 1.6 < x_scan < 1.7)
    lab, d = classify([-3.0, 0.0, 2.0, -2.0], [2.4, -3.39, 0.0, 0.0])
    chk(f'classify: box, wall, ramp, open ground -> {list(lab)}',
        list(lab) == ['box_obstacle_1', 'wall_south', 'ramp', 'phantom'])
    chk('a point 0.09 m off a box face is that box', classify([-3.34], [2.4])[0][0] == 'box_obstacle_1')
    chk('a point 0.11 m off a box face is a phantom', classify([-3.36], [2.4])[0][0] == 'phantom')
    chk('the cylinder is attributed by radius', classify([-0.2], [0.89])[0][0] == 'cylinder_obstacle'
        and classify([-0.2], [0.92])[0][0] == 'phantom')
    w_o = world_from_odom((1.0, 2.0, math.pi / 2), (0.5, 0.0, 0.0))
    back = compose2d(w_o, (0.5, 0.0, 0.0))
    chk(f'world_T_odom o odom_T_base recovers ground truth {tuple(round(v, 6) for v in back)}',
        abs(back[0] - 1.0) < 1e-9 and abs(back[1] - 2.0) < 1e-9 and abs(back[2] - math.pi / 2) < 1e-9)
    unknown = 0xFFFF   # what VoxelGrid::reset leaves: every voxel UNKNOWN
    data = [unknown,
            (unknown & ~(1 << 4)) | (1 << 4) | (1 << 20),          # voxel 4 marked
            (1 << 0) | (1 << 16) | (1 << 7) | (1 << 23) | (1 << 12),  # 0, 7 marked; 12 unknown
            (1 << 18)]                                              # malformed half: not marked
    top = voxel_top(data, 2, 2, 16, 0.0, 0.05)
    chk(f'voxel columns decode MARKED (both bits) only, not unknown: {top.ravel().tolist()}',
        np.isnan(top[0, 0]) and abs(top[0, 1] - 0.25) < 1e-9 and abs(top[1, 0] - 0.40) < 1e-9
        and np.isnan(top[1, 1]))
    xs, ys, _, _ = grid_marks([0, 100, 99, 100], 2, 2, 0.05, (1.0, 2.0))
    chk('grid_marks takes lethal (100) cells only, not inscribed (99)',
        np.allclose(xs, [1.075, 1.075]) and np.allclose(ys, [2.025, 2.075]))
    K = (230.0, 230.0, 160.0, 120.0)
    depth = np.full((240, 320), 2.0, dtype=np.float32)
    depth[:10, :10] = np.nan
    v, u = np.mgrid[0:240, 0:320]
    xo, yo = (u - K[2]) * depth / K[0], (v - K[3]) * depth / K[1]
    optical = np.stack([xo, yo, depth], axis=-1)
    link = np.stack([depth, -xo, -yo], axis=-1)
    a = pinhole_convention(depth, K, optical)
    b = pinhole_convention(depth, K, link)
    chk(f'convention test names an optical cloud optical {a}',
        a['optical_median_m'] < 1e-6 and a['link_median_m'] > 0.05)
    chk(f'convention test names a link cloud link {b}',
        b['link_median_m'] < 1e-6 and b['optical_median_m'] > 0.05)
    R_bo = _xacro_expectations()['camera_optical_frame']['R']
    q = (-0.5, 0.5, -0.5, 0.5)  # rpy (-pi/2, 0, -pi/2)
    chk('xacro optical rotation matches rpy (-pi/2, 0, -pi/2)', np.allclose(quat_matrix(q), R_bo))
    chk('optical z (forward) is base +x', np.allclose(R_bo @ [0, 0, 1], [1, 0, 0]))
    ground = np.array([[0.5, -2.0, 0.0], [0.7, -2.2, 0.01], [2.0, 0.0, float(ramp_height(2.0, 0.0))]])
    st = projection_stats(ground, (1.0, -1.0))
    chk(f"projection: ground at z 0 and a ramp point on the surface {st.get('ground')} {st.get('ramp')}",
        st['ground']['abs_max'] <= 0.01 and st['ramp']['surface_residual']['abs_max'] < 1e-9)
    ms = mark_summary([-3.0, -3.0, 0.0], [2.14, 2.2, 0.0], (-3.0, 1.5), tops=[0.25, 0.5, np.nan])
    chk(f"mark_summary counts per geometry and lists phantoms {ms['labels']}",
        ms['labels'] == {'box_obstacle_1': 2, 'phantom': 1} and len(ms['phantoms']) == 1
        and ms['objects']['box_obstacle_1']['voxel_top_max_m'] == 0.5)
    print(f'\n{n - fail} passed, {fail} FAILED')
    return 1 if fail else 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog='c2nav43_perception.py')
    sub = ap.add_subparsers(dest='mode', required=True)
    s = sub.add_parser('sensors')
    s.add_argument('--out', required=True)
    s.add_argument('--duration', type=float, default=8.0)
    s.add_argument('--pose', default='', choices=[''] + list(POSES))
    s.add_argument('--cloud-topic', default='/camera/points')
    s.add_argument('--tag', default='', help='suffix for the output file name')
    c = sub.add_parser('capture')
    c.add_argument('--out', required=True)
    c.add_argument('--lidar-params', required=True)
    c.add_argument('--fused-params', required=True)
    c.add_argument('--poses', default='')
    c.add_argument('--cloud-topic', default='/camera/depth/points')
    r = sub.add_parser('record')
    r.add_argument('--out', required=True)
    r.add_argument('--duration', type=float, default=0.0)
    h = sub.add_parser('rates')
    h.add_argument('--out', required=True)
    h.add_argument('--duration', type=float, default=10.0)
    h.add_argument('--tag', default='')
    h.add_argument('topics', nargs='+')
    sub.add_parser('selftest')
    args = ap.parse_args(argv)
    if args.mode == 'selftest':
        return selftest()
    return {'sensors': sensor_checks_mode, 'capture': capture_mode, 'record': record_mode,
            'rates': rates_mode}[args.mode](args)


if __name__ == '__main__':
    sys.exit(main())
