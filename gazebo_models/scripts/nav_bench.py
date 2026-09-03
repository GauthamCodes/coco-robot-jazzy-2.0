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

"""
nav_bench.py
============
C2-NAV.0 movement-quality benchmark: drive a fixed tour of Nav2 legs and
record what every stage of the command chain actually did.

`nav_round_trip.py` answers "did the leg succeed, and how close did it
come to an obstacle". That is the right question for a mission and the
wrong one for *movement quality*, which is about the metre before the
goal, not the goal. This records the whole chain instead:

    controller_server -> /cmd_vel_nav -> velocity_smoother
      -> /cmd_vel_smoothed -> collision_monitor -> /cmd_vel
      -> cmd_vel_relay -> /diff_drive_controller/cmd_vel -> wheels

plus DWB's own `/evaluation`, which carries the per-trajectory critic
scores and — the part that matters — the name of the critic that threw
`IllegalTrajectoryException` for every REJECTED trajectory. Counting
those by critic is the difference between "DWB looks cautious near
walls" and "BaseObstacle rejected 94 % of the sample set at t=31.2 s".

Ground truth (`/model/coco/odometry`) is read for EVALUATION ONLY and is
never published anywhere Nav2 can see it.

The tour is a chain: each leg starts wherever the previous one stopped,
so nothing is ever teleported and odom stays continuous. A failed leg is
recorded and the next one is attempted from wherever the robot actually
is, which is also how a mission would experience it.

Poses are WORLD coordinates; the map frame is anchored at the spawn, so
map = world + (2.0, 0).

Usage (with the sim and nav.launch.py already up):
  ros2 run gazebo_models nav_bench.py --tag baselineA --repeats 3
  ros2 run gazebo_models nav_bench.py --tag baselineB --repeats 3 \
      --out /tmp/navbench
"""
import argparse
import bisect
import csv
import json
import math
import os
import sys
import threading
import time
from collections import Counter, defaultdict

from action_msgs.msg import GoalStatus

from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                               TwistStamped)

from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.msg import CollisionMonitorState

from nav_msgs.msg import OccupancyGrid, Odometry, Path

import numpy as np

from rcl_interfaces.msg import Log

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

from sensor_msgs.msg import LaserScan

WORLD_TO_MAP_X = 2.0
WORLD_TO_MAP_Y = 0.0

# From nav2_params.yaml. Recorded here so the report can state the
# thresholds it is classifying against instead of implying them.
ROBOT_RADIUS = 0.20
INFLATION_RADIUS = 0.50
CONTROLLER_FREQUENCY = 10.0
# goal_checker.xy_goal_tolerance. NOT FollowPath.xy_goal_tolerance, which
# is 0.05 and is a different thing: it is the window RotateToGoal uses.
# The two disagreeing by 5x is itself a finding; see docs/RESULTS.md.
GOAL_XY_TOLERANCE = 0.25

# A "stop" is slower than this for at least STOP_MIN_S of sim time. 0.02
# m/s is 1/15th of max_vel_x and below anything the smoother emits on
# purpose; 0.4 s is four controller periods, so a single late command
# cannot manufacture one.
STOP_V = 0.02
STOP_MIN_S = 0.4

# A reversal is a sign change in COMMANDED linear velocity where both
# sides exceed this. Well above the 0.001 min_x_velocity_threshold that
# controller_server itself zeroes below.
REVERSAL_V = 0.03

# An oscillation event is a sign change in commanded angular velocity
# where both sides exceed this. 0.15 rad/s is 15 % of max_vel_theta.
OSC_W = 0.15

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}

# The tour. Each entry is (name, goal_world_x, goal_world_y, what it
# probes). Leg N starts where leg N-1 stopped; the first starts at the
# spawn. Clearances in the comments are measured from the shipped map by
# mapcheck, not from the world file's nominal geometry.
TOUR = [
    ('open_space',      -2.00, -2.20,
     'goal 1.15 m from anything: the control case'),
    ('wall_adjacent',   -2.00, -3.00,
     'goal 0.35 m from the south wall: inside inflation, outside inscribed'),
    ('wall_parallel',    0.50, -2.95,
     '2.5 m run held ~0.36 m off the south wall'),
    ('obstacle_corner',  0.30, -0.30,
     'round the box_obstacle_2 corner into open ground'),
    ('corridor_gate',   -2.60, -0.10,
     'through the Zone A gate: 1.30 m gap, 0.90 m non-inscribed'),
    ('enclosure_entry', -3.45,  2.95,
     'up the 0.63 m NW pinch (0.30 m free band) into the corner pocket'),
    ('enclosure_exit',  -2.00,  0.00,
     'back out of the pocket to home'),
]


def apply_goal_overrides(tour, specs):
    """TOUR with zero or more goals replaced, from `NAME:X,Y` strings.

    C2-NAV.7. Moving a scenario goal is a BENCHMARK change, not a tuning
    knob, so it is an explicit override rather than an edit: TOUR stays
    byte-identical to the C2-NAV.0 baseline that every earlier experiment
    ran against, and the goal that actually ran is written into each leg
    record as `goal_world`. Only the position moves -- `send_leg` sends a
    shared `orientation.w = 1.0` for every leg, so there is no per-leg
    yaw to disturb.

    Returns the new tour, or None if a spec is malformed or names a
    scenario that does not exist (checked before anything is launched).
    """
    if not specs:
        return list(tour)
    moved = {}
    for spec in specs:
        name, sep, xy = spec.partition(':')
        parts = xy.split(',')
        if not sep or len(parts) != 2:
            print(f'[nav_bench] malformed --goal {spec!r}, want NAME:X,Y')
            return None
        try:
            moved[name] = (float(parts[0]), float(parts[1]))
        except ValueError:
            print(f'[nav_bench] non-numeric --goal {spec!r}')
            return None
    unknown = sorted(set(moved) - {t[0] for t in tour})
    if unknown:
        print(f'[nav_bench] unknown scenario in --goal: {unknown}')
        return None
    out = []
    for (n, x, y, p) in tour:
        if n in moved:
            gx, gy = moved[n]
            print(f'[nav_bench] GOAL OVERRIDE {n}: ({x}, {y}) -> ({gx}, {gy})',
                  flush=True)
            out.append((n, gx, gy, p + ' [GOAL OVERRIDDEN]'))
        else:
            out.append((n, x, y, p))
    return out


def apply_waypoint_insert(tour, specs):
    """Insert zero or more intermediate legs immediately before a named
    scenario, from `BEFORE:X,Y` strings.

    C2-NAV.10. C2-NAV.9's offline corridor reconstruction found a
    PolygonStop-free route from every observed `corridor_gate` exit to
    the current `enclosure_entry` goal with a 326 mm bottleneck -- 76 mm
    of margin over PolygonStop's 250 mm -- but found nothing in the local
    cost field that would make DWB prefer it over the 257-260 mm route
    past `box_obstacle_1`'s SW corner that deadlocked C2-NAV.8's r1 tour
    for 269.5 s: `local_costmap.cost_scaling_factor = 65.0` reaches cost
    0 at 291 mm, so BaseObstacle cannot tell 257 mm from 326 mm apart.
    This tests whether an explicit global-plan waypoint, not a parameter
    change, fixes the selection.

    Like `apply_goal_overrides` and `apply_leg_timeouts`, this is a
    default-off, benchmark-level route change: TOUR is never edited, and
    every earlier C2-NAV.0 ... C2-NAV.9 command reproduces unchanged
    without `--waypoint`. The inserted leg is a plain scenario entry --
    it gets its own `NavigateToPose` call through the normal per-leg loop
    in `main()`, so nothing else in this file treats it specially, and it
    chains exactly like any other leg: whatever follows it starts from
    wherever it actually stopped, not from the inserted point itself.

    Returns the new tour, or None if a spec is malformed, names a
    scenario that does not exist, or collides with an existing name --
    checked before anything is launched.
    """
    if not specs:
        return list(tour)
    names = [t[0] for t in tour]
    inserts = {}
    for spec in specs:
        before, sep, xy = spec.partition(':')
        parts = xy.split(',')
        if not sep or len(parts) != 2:
            print(f'[nav_bench] malformed --waypoint {spec!r}, '
                  f'want BEFORE:X,Y')
            return None
        try:
            wx, wy = float(parts[0]), float(parts[1])
        except ValueError:
            print(f'[nav_bench] non-numeric --waypoint {spec!r}')
            return None
        if before not in names:
            print(f'[nav_bench] unknown scenario in --waypoint: {before!r}')
            return None
        if before in inserts:
            print(f'[nav_bench] duplicate --waypoint target: {before!r}')
            return None
        inserts[before] = (wx, wy)
    out = []
    for (n, x, y, p) in tour:
        if n in inserts:
            wx, wy = inserts[n]
            wp_name = f'{n}_waypoint'
            if wp_name in names:
                print(f'[nav_bench] waypoint leg name collides with an '
                      f'existing scenario: {wp_name!r}')
                return None
            print(f'[nav_bench] WAYPOINT INSERT before {n}: ({wx}, {wy})',
                  flush=True)
            out.append((wp_name, wx, wy,
                        f'C2-NAV.10 corridor-aligned intermediate waypoint '
                        f'into {n} [WAYPOINT INSERTED]'))
        out.append((n, x, y, p))
    return out


def apply_through_poses(tour, specs):
    """Map {scenario_name: [(x, y), ...]} of intermediate poses to send
    ahead of that scenario's own goal in ONE `NavigateThroughPoses`
    request, from zero or more `BEFORE:X,Y` strings.

    C2-NAV.11. C2-NAV.10's `apply_waypoint_insert` (above) tests a
    waypoint as a SEPARATE `NavigateToPose` leg; that experiment found
    the wide-corridor preference it buys does not survive the re-plan
    boundary between the inserted leg and the one after it. This is the
    other shape of intervention the C2-NAV.10 write-up's "Exact next
    experiment" section names: keep the waypoint's coordinates
    unchanged, but present it to Nav2 as part of ONE continuous
    multi-pose request for the named scenario's own goal, so there is no
    re-plan boundary for DWB's local sampling to reset at.

    Unlike `apply_waypoint_insert`, this does NOT add a leg to `tour` --
    the named scenario keeps its single entry, and the run loop in
    `main()` decides at dispatch time whether to call `send_leg` (no
    through-poses attached) or `send_multi_leg` (this scenario's
    intermediate poses, then its own goal, all in one action request).
    That means, unlike C2-NAV.10's insertion, there is no `--only`
    ordering hazard here: this mapping is consulted by NAME after
    filtering, not spliced into the list filtering itself walks.

    Returns {name: [(x, y), ...]}, possibly with empty lists for
    scenarios not named in `specs`, or None if a spec is malformed or
    names a scenario that does not exist -- checked before anything is
    launched.
    """
    names = [t[0] for t in tour]
    out = {n: [] for n in names}
    if not specs:
        return out
    for spec in specs:
        before, sep, xy = spec.partition(':')
        parts = xy.split(',')
        if not sep or len(parts) != 2:
            print(f'[nav_bench] malformed --through-pose {spec!r}, '
                  f'want BEFORE:X,Y')
            return None
        try:
            wx, wy = float(parts[0]), float(parts[1])
        except ValueError:
            print(f'[nav_bench] non-numeric --through-pose {spec!r}')
            return None
        if before not in out:
            print(f'[nav_bench] unknown scenario in --through-pose: '
                  f'{before!r}')
            return None
        print(f'[nav_bench] THROUGH-POSE for {before}: ({wx}, {wy}) '
              f'[continuous multi-pose request]', flush=True)
        out[before].append((wx, wy))
    return out


def apply_leg_timeouts(tour, default_s, specs):
    """Per-leg wall-clock caps, from zero or more `NAME:SECONDS` strings.

    C2-NAV.8. A benchmark timeout is a MEASUREMENT WINDOW, not a
    navigation parameter -- but a single `--timeout` forces one window on
    every leg, and the two constraints here disagree. The six ordinary
    legs must stay at C2-NAV.5's 75 s or their results are not comparable
    to its committed 18/21; `enclosure_entry` at the C2-NAV.7 goal ran
    116.56 / 150.68 / 150.01 s against a 150 s cap, so 75 s would score
    slow convergence as failure and 150 s cannot tell the two apart at
    all, two of the three having ended AT the cap.

    So the cap is per leg, default-off exactly as `--goal` is: absent the
    flag every leg gets `default_s` and every C2-NAV.0 ... C2-NAV.7
    command reproduces unchanged. The cap a leg actually ran under is
    written into its record as `timeout_s`, so a TIMEOUT can never be
    read against the wrong window.

    Returns {name: seconds}, or None if a spec is malformed, non-positive
    or names a scenario that does not exist -- checked before anything is
    launched.
    """
    out = {t[0]: float(default_s) for t in tour}
    if not specs:
        return out
    for spec in specs:
        name, sep, secs = spec.partition(':')
        if not sep:
            print(f'[nav_bench] malformed --leg-timeout {spec!r}, '
                  f'want NAME:SECONDS')
            return None
        try:
            val = float(secs)
        except ValueError:
            print(f'[nav_bench] non-numeric --leg-timeout {spec!r}')
            return None
        if val <= 0.0:
            print(f'[nav_bench] non-positive --leg-timeout {spec!r}')
            return None
        if name not in out:
            print(f'[nav_bench] unknown scenario in --leg-timeout: {name!r}')
            return None
        print(f'[nav_bench] LEG TIMEOUT {name}: {default_s} -> {val} s',
              flush=True)
        out[name] = val
    return out


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_norm(a):
    return math.atan2(math.sin(a), math.cos(a))


class Series:
    """Timestamped scalar/tuple samples with (wall, sim) clocks."""

    def __init__(self):
        self.t = []          # sim seconds
        self.tw = []         # wall seconds (monotonic)
        self.v = []

    def add(self, tsim, twall, value):
        self.t.append(tsim)
        self.tw.append(twall)
        self.v.append(value)

    def window(self, t0, t1):
        i = bisect.bisect_left(self.t, t0)
        j = bisect.bisect_right(self.t, t1)
        return self.t[i:j], self.v[i:j]

    def rate(self, t0, t1):
        """Messages per SIM second over the window."""
        ts, _ = self.window(t0, t1)
        if len(ts) < 2 or (t1 - t0) <= 0:
            return None
        return len(ts) / (t1 - t0)


class NavBench(Node):
    """Records every stage of the command chain across a tour of legs."""

    def __init__(self):
        # use_sim_time as an OVERRIDE, not a later set_parameters: rclpy's
        # TimeSource binds the clock when the node is constructed, and a
        # node that reports wall time while everything else reports sim
        # time silently mis-scales every rate in the report.
        super().__init__('nav_bench',
                         parameter_overrides=[
                             Parameter('use_sim_time', value=True)])
        self._lock = threading.Lock()
        self._t0_wall = time.monotonic()
        self._sim_ok = False

        self.gt = Series()          # (x, y, yaw, v, w) world frame
        self.amcl = Series()        # (x, y, yaw) map frame
        self.nav = Series()         # /cmd_vel_nav      (v, w)
        self.smooth = Series()      # /cmd_vel_smoothed (v, w)
        self.out = Series()         # /cmd_vel          (v, w)
        self.wheel = Series()       # wheel topic       (v, w)
        self.cmstate = Series()     # (action_type, polygon_name)
        self.scanmin = Series()     # min range in the scan
        self.plans = Series()       # (n_poses, length_m)
        self.localplan = Series()   # n_poses
        self.evals = Series()       # dict summary per control cycle
        self.logs = []              # (tsim, level, name, msg)

        self.grid = None
        self.last_plan = None       # list of (x, y) in MAP frame
        # Ring of recent LOCAL costmaps, so the report can quote the cost
        # field the controller was actually looking at when it slowed
        # down rather than a grid captured seconds later.
        self.localmaps = []         # (tsim, OccupancyGrid), last 60
        # C2-NAV.11. `self.plans` (above) only keeps (n_poses, length) per
        # /plan message -- enough for a rate, not enough to answer "did
        # THIS specific plan already extend past the waypoint toward the
        # final goal". Ring of full /plan geometry, world frame, so a
        # snapshot from early in a leg (before the robot has physically
        # reached an intermediate pose) can be inspected directly as
        # continuity evidence.
        self.plan_snapshots = []    # (tsim, [(x, y), ...] world), last 200

        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(
            Odometry, '/model/coco/odometry', self._gt_cb, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._local_cb, latched)
        for topic, series in (('/cmd_vel_nav', self.nav),
                              ('/cmd_vel_smoothed', self.smooth),
                              ('/cmd_vel', self.out),
                              ('/diff_drive_controller/cmd_vel', self.wheel)):
            self.create_subscription(
                TwistStamped, topic,
                (lambda s: lambda m: self._twist_cb(s, m))(series), 20)
        self.create_subscription(
            CollisionMonitorState, '/collision_monitor_state',
            self._cm_cb, 10)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.create_subscription(Path, '/local_plan', self._lp_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, latched)
        self.create_subscription(Log, '/rosout', self._log_cb, 200)
        self._eval_ok = self._try_eval()

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # C2-NAV.11. Separate client, separate action server name --
        # `bt_navigator`'s `navigators` param registers both under the
        # SAME node, so this does not compete with `self._client` for
        # anything; the two are just different goal message shapes.
        self._ntp_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')

    # -- clocks ---------------------------------------------------------
    def now(self):
        """(sim seconds, wall seconds)."""
        return (self.get_clock().now().nanoseconds * 1e-9,
                time.monotonic() - self._t0_wall)

    # -- subscriptions --------------------------------------------------
    def _try_eval(self):
        """/evaluation is DWB's own scoring dump. It carries the rejecting
        critic's name for every illegal trajectory, which is the only
        direct evidence for 'which critic stopped the robot'. Optional:
        dwb_msgs is a Nav2 package but the topic only exists while DWB is
        the loaded controller."""
        try:
            from dwb_msgs.msg import LocalPlanEvaluation
        except ImportError:
            self.get_logger().warn('dwb_msgs unavailable: no critic evidence')
            return False
        self.create_subscription(
            LocalPlanEvaluation, '/evaluation', self._eval_cb, 10)
        return True

    def _gt_cb(self, m):
        ts, tw = self.now()
        p, o = m.pose.pose.position, m.pose.pose.orientation
        t = m.twist.twist
        with self._lock:
            self._sim_ok = True
            self.gt.add(ts, tw, (p.x, p.y, yaw_of(o), t.linear.x, t.angular.z))

    def _amcl_cb(self, m):
        ts, tw = self.now()
        p, o = m.pose.pose.position, m.pose.pose.orientation
        with self._lock:
            self.amcl.add(ts, tw, (p.x, p.y, yaw_of(o)))

    def _local_cb(self, m):
        ts, _ = self.now()
        with self._lock:
            self.localmaps.append((ts, m))
            if len(self.localmaps) > 60:
                self.localmaps.pop(0)

    def _twist_cb(self, series, m):
        ts, tw = self.now()
        with self._lock:
            series.add(ts, tw, (m.twist.linear.x, m.twist.angular.z))

    def _cm_cb(self, m):
        ts, tw = self.now()
        with self._lock:
            self.cmstate.add(ts, tw, (int(m.action_type), str(m.polygon_name)))

    def _scan_cb(self, m):
        ts, tw = self.now()
        good = [r for r in m.ranges
                if m.range_min <= r <= m.range_max and math.isfinite(r)]
        with self._lock:
            self.scanmin.add(ts, tw, min(good) if good else float('inf'))

    def _plan_cb(self, m):
        ts, tw = self.now()
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        length = sum(math.dist(pts[i], pts[i + 1])
                     for i in range(len(pts) - 1)) if len(pts) > 1 else 0.0
        with self._lock:
            self.last_plan = pts
            self.plans.add(ts, tw, (len(pts), length))
            self.plan_snapshots.append((ts, pts))   # map frame, like last_plan
            if len(self.plan_snapshots) > 200:
                self.plan_snapshots.pop(0)

    def _lp_cb(self, m):
        ts, tw = self.now()
        with self._lock:
            self.localplan.add(ts, tw, len(m.poses))

    def _map_cb(self, m):
        with self._lock:
            self.grid = m

    def _log_cb(self, m):
        if m.level < 30:                 # WARN and above only
            return
        ts, _ = self.now()
        with self._lock:
            self.logs.append((ts, int(m.level), m.name, m.msg))

    def _eval_cb(self, m):
        """Summarise in the callback: 20 vx x 40 vtheta is 800 trajectories
        per cycle at 10 Hz, and storing them raw is hundreds of MB."""
        ts, tw = self.now()
        n_total = len(m.twists)
        illegal = Counter()
        n_illegal = 0
        for score in m.twists:
            if score.total < 0.0:
                n_illegal += 1
                for cs in score.scores:
                    if cs.raw_score < 0.0:
                        illegal[cs.name] += 1
                        break
        best = None
        if 0 <= m.best_index < n_total:
            b = m.twists[m.best_index]
            best = {'total': float(b.total),
                    'vx': float(b.traj.velocity.x),
                    'wz': float(b.traj.velocity.theta),
                    'critics': {cs.name: float(cs.raw_score) * float(cs.scale)
                                for cs in b.scores}}
        with self._lock:
            self.evals.add(ts, tw, {'n': n_total, 'n_illegal': n_illegal,
                                    'illegal': dict(illegal), 'best': best})

    # -- map clearance --------------------------------------------------
    def occupied_cells(self):
        """Centres of occupied STATIC map cells, in map frame, as an
        (N, 2) array so clearance is a vectorised distance rather than a
        3000 x 3700 Python loop per leg."""
        g = self.grid
        if g is None:
            return np.zeros((0, 2))
        info = g.info
        res = info.resolution
        data = np.asarray(g.data, dtype=np.int16).reshape(
            info.height, info.width)
        ys, xs = np.nonzero(data > 65)
        return np.stack([
            info.origin.position.x + xs * res + res / 2.0,
            info.origin.position.y + ys * res + res / 2.0], axis=1)

    # -- goals ----------------------------------------------------------
    def send_leg(self, wx, wy, timeout_s):
        """Drive to a world pose. Returns (status_name, t0, t1) in sim s."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wx + WORLD_TO_MAP_X
        goal.pose.pose.position.y = wy + WORLD_TO_MAP_Y
        goal.pose.pose.orientation.w = 1.0

        if not self._client.wait_for_server(timeout_sec=15.0):
            return ('NO_SERVER', None, None)

        t0 = self.now()[0]
        fut = self._client.send_goal_async(goal)
        deadline = time.monotonic() + 20.0
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not fut.done():
            return ('NO_ACK', t0, self.now()[0])
        handle = fut.result()
        if not handle.accepted:
            return ('REJECTED', t0, self.now()[0])

        res_fut = handle.get_result_async()
        wall_deadline = time.monotonic() + timeout_s
        while not res_fut.done() and time.monotonic() < wall_deadline:
            time.sleep(0.05)
        if not res_fut.done():
            handle.cancel_goal_async()
            time.sleep(2.0)
            return ('TIMEOUT', t0, self.now()[0])
        status = res_fut.result().status
        return (STATUS_NAMES.get(status, f'STATUS_{status}'), t0, self.now()[0])

    def send_multi_leg(self, world_poses, timeout_s):
        """Drive through an ORDERED list of world poses in ONE
        `NavigateThroughPoses` request -- no leg boundary between them, so
        unlike a chain of `send_leg` calls there is no independent re-plan
        when the robot passes an intermediate pose.

        C2-NAV.11. `world_poses` is `[(x, y), ...]`; the last entry is the
        scenario's own goal, everything before it is a `--through-pose`.

        Returns `(status_name, t0, t1, early_plan)` in sim s, where
        `early_plan` is `(ts, [(x, y) map frame, ...])` for the FIRST
        `/plan` message received after the goal was accepted -- this is
        the continuity evidence the experiment exists to produce, not a
        report statistic: a single message computed before the robot has
        had time to reach even the first intermediate pose, whose points
        already run toward the LAST requested pose, is what a genuinely
        continuous plan looks like. C2-NAV.10's mechanism cannot produce
        this by construction -- its second leg's /plan does not exist
        until the first leg's `NavigateToPose` call has already
        completed. `early_plan` is None if no `/plan` arrived within 8
        real seconds of acceptance -- reported honestly, not treated as
        empty-equals-continuous.
        """
        goal = NavigateThroughPoses.Goal()
        for (wx, wy) in world_poses:
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = wx + WORLD_TO_MAP_X
            p.pose.position.y = wy + WORLD_TO_MAP_Y
            p.pose.orientation.w = 1.0
            goal.poses.append(p)

        if not self._ntp_client.wait_for_server(timeout_sec=15.0):
            return ('NO_SERVER', None, None, None)

        with self._lock:
            n_before = len(self.plan_snapshots)
        t0 = self.now()[0]
        fut = self._ntp_client.send_goal_async(goal)
        deadline = time.monotonic() + 20.0
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not fut.done():
            return ('NO_ACK', t0, self.now()[0], None)
        handle = fut.result()
        if not handle.accepted:
            return ('REJECTED', t0, self.now()[0], None)

        early_plan = None
        cap_deadline = time.monotonic() + 8.0
        while time.monotonic() < cap_deadline:
            with self._lock:
                if len(self.plan_snapshots) > n_before:
                    early_plan = self.plan_snapshots[n_before]
                    break
            time.sleep(0.05)

        res_fut = handle.get_result_async()
        wall_deadline = time.monotonic() + timeout_s
        while not res_fut.done() and time.monotonic() < wall_deadline:
            time.sleep(0.05)
        if not res_fut.done():
            handle.cancel_goal_async()
            time.sleep(2.0)
            return ('TIMEOUT', t0, self.now()[0], early_plan)
        status = res_fut.result().status
        return (STATUS_NAMES.get(status, f'STATUS_{status}'), t0,
                self.now()[0], early_plan)


def costmap_cross_section(grid, yaw, half_width=1.0):
    """Cost across the robot, perpendicular to its heading.

    The local costmap is `rolling_window: true`, so the robot is at the
    centre cell by construction and no TF lookup is needed. Its frame is
    `odom`, whose axes are parallel to the world's (odom is anchored at
    the spawn, yaw 0), so the world heading indexes it directly.

    Returns (offsets, costs) where offset 0 is the robot centre, negative
    is to starboard. Cost 100 in an OccupancyGrid is Nav2's LETHAL/
    INSCRIBED band; -1 is unknown.
    """
    info = grid.info
    res = info.resolution
    data = np.asarray(grid.data, dtype=np.int16).reshape(
        info.height, info.width)
    cx, cy = info.width // 2, info.height // 2
    ux, uy = -math.sin(yaw), math.cos(yaw)          # left-normal
    offs, costs = [], []
    n = int(half_width / res)
    for k in range(-n, n + 1):
        d = k * res
        gx = int(round(cx + ux * d / res))
        gy = int(round(cy + uy * d / res))
        if 0 <= gx < info.width and 0 <= gy < info.height:
            offs.append(round(d, 3))
            costs.append(int(data[gy, gx]))
    return offs, costs


def summarise(node, name, probe, goal_w, status, t0, t1, occupied):
    """Everything section 4 of the brief asks for, per leg."""
    with node._lock:
        gt_t, gt_v = node.gt.window(t0, t1)
        nav_t, nav_v = node.nav.window(t0, t1)
        sm_t, sm_v = node.smooth.window(t0, t1)
        out_t, out_v = node.out.window(t0, t1)
        wh_t, wh_v = node.wheel.window(t0, t1)
        cm_t, cm_v = node.cmstate.window(t0, t1)
        sc_t, sc_v = node.scanmin.window(t0, t1)
        pl_t, pl_v = node.plans.window(t0, t1)
        lp_t, _ = node.localplan.window(t0, t1)
        ev_t, ev_v = node.evals.window(t0, t1)
        logs = [rec for rec in node.logs if t0 <= rec[0] <= t1]
        plan = list(node.last_plan) if node.last_plan else None
        wallpairs = [(t, w) for t, w in zip(node.gt.t, node.gt.tw)
                     if t0 <= t <= t1]
        localmaps = list(node.localmaps)

    dur = t1 - t0
    r = {'scenario': name, 'probes': probe, 'status': status,
         'goal_world': list(goal_w), 'duration_sim_s': round(dur, 2)}
    # Real-time factor over the leg. Every rate below is per SIM second;
    # this is what converts them to wall rates, and a low RTF is itself a
    # candidate explanation for a controller missing its period.
    if len(wallpairs) > 1:
        dw = wallpairs[-1][1] - wallpairs[0][1]
        ds = wallpairs[-1][0] - wallpairs[0][0]
        r['rtf'] = round(ds / dw, 3) if dw > 1e-6 else None
        r['duration_wall_s'] = round(dw, 2)

    # --- path -------------------------------------------------------
    xy = [(v[0], v[1]) for v in gt_v]
    r['start_world'] = [round(xy[0][0], 3), round(xy[0][1], 3)] if xy else None
    r['end_world'] = [round(xy[-1][0], 3), round(xy[-1][1], 3)] if xy else None
    driven = sum(math.dist(xy[i], xy[i + 1]) for i in range(len(xy) - 1)) \
        if len(xy) > 1 else 0.0
    r['path_len_m'] = round(driven, 3)
    if xy:
        r['straight_line_m'] = round(math.dist(xy[0], goal_w), 3)
        r['final_goal_err_m'] = round(math.dist(xy[-1], goal_w), 3)
        r['path_efficiency'] = (round(r['straight_line_m'] / driven, 3)
                                if driven > 1e-6 else None)

    # --- clearance --------------------------------------------------
    # Distance from the DRIVEN path to the nearest occupied static cell.
    # Cell centres, so a robot exactly on a wall face reads ~0.025 m, not
    # 0. The number to compare against is robot_radius = 0.20.
    if len(occupied) and xy:
        track = np.asarray(xy) + np.array([WORLD_TO_MAP_X, WORLD_TO_MAP_Y])
        d = np.sqrt(((track[:, None, :] - occupied[None, :, :]) ** 2
                     ).sum(-1)).min(axis=1)
        r['min_clearance_m'] = round(float(d.min()), 3)
        r['med_clearance_m'] = round(float(np.median(d)), 3)
        r['frac_track_inside_inflation'] = round(
            float((d < INFLATION_RADIUS).mean()), 3)
    r['min_scan_range_m'] = round(min(sc_v), 3) if sc_v else None

    # --- TRANSIT vs TERMINAL -----------------------------------------
    # Split the leg where the robot first enters the goal checker's
    # xy_goal_tolerance. Everything after that is settling on the goal
    # YAW, which is a different behaviour with a different cause, and
    # averaging the two together hides both: a leg that drives perfectly
    # and then spins for ten seconds reports the same mean velocity as
    # one that crawls the whole way.
    t_reach = None
    for t, v in zip(gt_t, gt_v):
        if math.dist((v[0], v[1]), goal_w) <= GOAL_XY_TOLERANCE:
            t_reach = t
            break
    r['goal_xy_tolerance'] = GOAL_XY_TOLERANCE
    if t_reach is not None:
        r['t_transit_s'] = round(t_reach - t0, 2)
        r['t_terminal_s'] = round(t1 - t_reach, 2)
        r['terminal_frac_of_leg'] = round((t1 - t_reach) / dur, 3) \
            if dur > 0 else None
        tr_v = [v for t, v in zip(gt_t, gt_v) if t <= t_reach]
        te_v = [v for t, v in zip(gt_t, gt_v) if t > t_reach]
        if tr_v:
            xy_tr = [(v[0], v[1]) for v in tr_v]
            d_tr = sum(math.dist(xy_tr[i], xy_tr[i + 1])
                       for i in range(len(xy_tr) - 1))
            r['transit_len_m'] = round(d_tr, 3)
            r['transit_speed_mean'] = round(
                d_tr / (t_reach - t0), 3) if t_reach > t0 else None
            r['transit_v_med'] = round(
                sorted(abs(v[3]) for v in tr_v)[len(tr_v) // 2], 4)
        if te_v:
            r['terminal_yaw_travel_rad'] = round(
                sum(abs(ang_norm(te_v[i + 1][2] - te_v[i][2]))
                    for i in range(len(te_v) - 1)), 3)
            r['terminal_v_med'] = round(
                sorted(abs(v[3]) for v in te_v)[len(te_v) // 2], 4)
        # DWB rejections, split the same way.
        for label, lo, hi in (('transit', t0, t_reach),
                              ('terminal', t_reach, t1)):
            sub = [v for t, v in zip(ev_t, ev_v) if lo <= t <= hi]
            tot = sum(e['n'] for e in sub)
            ill = sum(e['n_illegal'] for e in sub)
            by = Counter()
            for e in sub:
                by.update(e['illegal'])
            r[f'dwb_illegal_frac_{label}'] = (round(ill / tot, 4)
                                              if tot else None)
            r[f'dwb_illegal_by_critic_{label}'] = dict(by.most_common(5))
    else:
        r['t_transit_s'] = None
        r['t_terminal_s'] = 0.0
        r['note'] = 'never reached goal xy tolerance'

    # --- velocities: commanded (controller) vs actual (ground truth) --
    def stats(vals, key):
        if not vals:
            return {}
        a = sorted(abs(x) for x in vals)
        return {f'{key}_mean': round(sum(a) / len(a), 4),
                f'{key}_med': round(a[len(a) // 2], 4),
                f'{key}_p95': round(a[int(0.95 * (len(a) - 1))], 4),
                f'{key}_max': round(a[-1], 4)}

    r.update(stats([v[3] for v in gt_v], 'v_actual'))
    r.update(stats([v[4] for v in gt_v], 'w_actual'))
    r.update(stats([v[0] for v in nav_v], 'v_cmd'))
    r.update(stats([v[1] for v in nav_v], 'w_cmd'))
    r.update(stats([v[0] for v in wh_v], 'v_wheel'))
    # How much of the leg was spent crawling, as commanded and as driven.
    if nav_v:
        r['frac_cmd_below_0.05'] = round(
            sum(1 for v in nav_v if abs(v[0]) < 0.05) / len(nav_v), 3)
    if gt_v:
        r['frac_actual_below_0.05'] = round(
            sum(1 for v in gt_v if abs(v[3]) < 0.05) / len(gt_v), 3)

    # --- stops, reversals, oscillation ------------------------------
    stops, run_start = 0, None
    for t, v in zip(gt_t, gt_v):
        if abs(v[3]) < STOP_V:
            run_start = t if run_start is None else run_start
        else:
            if run_start is not None and (t - run_start) >= STOP_MIN_S:
                stops += 1
            run_start = None
    if run_start is not None and gt_t and (gt_t[-1] - run_start) >= STOP_MIN_S:
        stops += 1
    r['n_stops'] = stops

    def sign_flips(vals, thresh):
        n, last = 0, 0
        for x in vals:
            s = (1 if x > thresh else (-1 if x < -thresh else 0))
            if s and last and s != last:
                n += 1
            if s:
                last = s
        return n

    r['n_reversals_cmd'] = sign_flips([v[0] for v in nav_v], REVERSAL_V)
    r['n_osc_cmd'] = sign_flips([v[1] for v in nav_v], OSC_W)
    r['osc_per_sec'] = round(r['n_osc_cmd'] / dur, 3) if dur > 0 else None

    # --- command chain rates (per SIM second) ------------------------
    for key, s in (('cmd_vel_nav', node.nav), ('cmd_vel_smoothed', node.smooth),
                   ('cmd_vel', node.out),
                   ('diff_drive_controller/cmd_vel', node.wheel)):
        hz = s.rate(t0, t1)
        r[f'hz_{key}'] = round(hz, 2) if hz else None
    r['hz_controller_expected'] = CONTROLLER_FREQUENCY

    # --- planner ----------------------------------------------------
    r['n_plans'] = len(pl_t)
    r['replans_per_sec'] = round(len(pl_t) / dur, 3) if dur > 0 else None
    r['plan_len_m_first'] = round(pl_v[0][1], 3) if pl_v else None
    r['plan_len_m_last'] = round(pl_v[-1][1], 3) if pl_v else None
    r['n_local_plans'] = len(lp_t)

    # cross-track: ground-truth position against the last global plan
    if plan and xy:
        errs = []
        for (x, y) in xy:
            mx, my = x + WORLD_TO_MAP_X, y + WORLD_TO_MAP_Y
            errs.append(min(math.dist((mx, my), p) for p in plan))
        errs.sort()
        r['xtrack_med_m'] = round(errs[len(errs) // 2], 3)
        r['xtrack_max_m'] = round(errs[-1], 3)

    # --- collision monitor ------------------------------------------
    # TIME-weighted, not message-weighted: the monitor publishes its
    # state on change, so a leg can contain four messages and counting
    # them makes a 0.2 s blip look like a quarter of the run.
    names = {0: 'DO_NOTHING', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH',
             4: 'LIMIT'}
    if cm_v:
        held = defaultdict(float)
        polyt = defaultdict(float)
        for i, (t, v) in enumerate(zip(cm_t, cm_v)):
            end = cm_t[i + 1] if i + 1 < len(cm_t) else t1
            held[v[0]] += max(0.0, end - t)
            if v[1]:
                polyt[v[1]] += max(0.0, end - t)
        span = sum(held.values()) or 1.0
        r['cm_action_frac'] = {names.get(k, str(k)): round(s / span, 3)
                               for k, s in sorted(held.items())}
        r['cm_polygon_secs'] = {k: round(s, 2) for k, s in polyt.items()}
        r['cm_gated_frac'] = round(
            sum(s for k, s in held.items() if k != 0) / span, 3)
        r['cm_msgs'] = len(cm_v)
    else:
        r['cm_action_frac'] = None

    # --- DWB critic evidence ----------------------------------------
    if ev_v:
        tot = sum(e['n'] for e in ev_v)
        ill = sum(e['n_illegal'] for e in ev_v)
        by = Counter()
        for e in ev_v:
            by.update(e['illegal'])
        r['dwb_cycles'] = len(ev_v)
        r['dwb_hz'] = round(len(ev_v) / dur, 2) if dur > 0 else None
        r['dwb_traj_per_cycle'] = round(tot / len(ev_v), 1)
        r['dwb_illegal_frac'] = round(ill / tot, 4) if tot else None
        r['dwb_illegal_by_critic'] = {k: v for k, v in by.most_common()}
        # Worst cycle: the moment DWB had least to choose from.
        worst = max(ev_v, key=lambda e: (e['n_illegal'] / e['n']) if e['n'] else 0)
        r['dwb_worst_cycle_illegal_frac'] = round(
            worst['n_illegal'] / worst['n'], 4) if worst['n'] else None
        # Mean scaled contribution of each critic to the CHOSEN trajectory:
        # this is what actually decided the command.
        crit = defaultdict(list)
        for e in ev_v:
            if e['best']:
                for k, v in e['best']['critics'].items():
                    crit[k].append(v)
        r['dwb_best_critic_mean'] = {
            k: round(sum(v) / len(v), 2)
            for k, v in sorted(crit.items(),
                               key=lambda kv: -sum(kv[1]) / len(kv[1]))}
        chosen = [e['best']['vx'] for e in ev_v if e['best']]
        if chosen:
            r['dwb_best_vx_mean'] = round(sum(chosen) / len(chosen), 4)
            r['dwb_best_vx_zero_frac'] = round(
                sum(1 for v in chosen if abs(v) < 1e-6) / len(chosen), 3)
    else:
        r['dwb_cycles'] = 0

    # --- the worst moment, and the whole chain at it -----------------
    # "Identify the exact moment the robot slows/stops" (brief S5). The
    # anchor is the longest run of commanded crawl, because that is the
    # symptom being investigated; everything else is read AT that time so
    # the causes cannot be assembled from different instants.
    # Restricted to the TRANSIT phase: a crawl at the goal is the
    # terminal rotation and is reported separately. The wall-stall
    # question is only about crawls that happen on the way.
    if nav_t:
        t_cut = t_reach if t_reach is not None else t1
        worst_start, worst_len, run_t = None, 0.0, None
        for t, v in zip(nav_t, nav_v):
            if t > t_cut:
                break
            if abs(v[0]) < 0.05:
                run_t = t if run_t is None else run_t
                if (t - run_t) > worst_len:
                    worst_len, worst_start = t - run_t, run_t
            else:
                run_t = None
        if worst_start is not None and worst_len >= 0.3:
            tw_ = worst_start + worst_len / 2.0
            snap = {'t_rel_s': round(worst_start - t0, 2),
                    'crawl_len_s': round(worst_len, 2)}

            def at(ts, vs):
                i = bisect.bisect_right(ts, tw_) - 1
                return vs[i] if i >= 0 else None

            g = at(gt_t, gt_v)
            if g:
                snap['pose_world'] = [round(g[0], 3), round(g[1], 3),
                                      round(g[2], 3)]
                snap['v_actual'] = round(g[3], 4)
                snap['w_actual'] = round(g[4], 4)
                # Distance to goal separates "stalled on the way" from
                # "arrived and settling", which look identical in v alone.
                snap['dist_to_goal_m'] = round(
                    math.dist((g[0], g[1]), goal_w), 3)
            n_ = at(nav_t, nav_v)
            snap['w_cmd_nav'] = round(n_[1], 4) if n_ else None
            for key, (ts, vs) in (('v_cmd_nav', (nav_t, nav_v)),
                                  ('v_smoothed', (sm_t, sm_v)),
                                  ('v_cmdvel', (out_t, out_v)),
                                  ('v_wheel', (wh_t, wh_v))):
                a = at(ts, vs)
                snap[key] = round(a[0], 4) if a else None
            c = at(cm_t, cm_v)
            snap['collision_monitor'] = (
                {0: 'DO_NOTHING', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH',
                 4: 'LIMIT'}.get(c[0], str(c[0])), c[1]) if c else None
            snap['scan_min_m'] = round(at(sc_t, sc_v), 3) \
                if at(sc_t, sc_v) is not None else None
            e = at(ev_t, ev_v)
            if e:
                snap['dwb_n_traj'] = e['n']
                snap['dwb_n_illegal'] = e['n_illegal']
                snap['dwb_illegal_frac'] = (round(e['n_illegal'] / e['n'], 3)
                                            if e['n'] else None)
                snap['dwb_illegal_by_critic'] = e['illegal']
                if e['best']:
                    snap['dwb_chosen_vx'] = round(e['best']['vx'], 4)
                    snap['dwb_chosen_wz'] = round(e['best']['wz'], 4)
                    snap['dwb_chosen_critics'] = {
                        k: round(v, 2) for k, v in
                        sorted(e['best']['critics'].items(),
                               key=lambda kv: -kv[1])}
            # Was there a global path at all, and was it fresh?
            prior = [t for t in pl_t if t <= tw_]
            snap['plan_age_s'] = (round(tw_ - prior[-1], 2) if prior
                                  else None)
            snap['had_plan'] = bool(plan)
            # The local costmap the controller was reading, across the
            # robot. This is the free-corridor-width measurement.
            if localmaps and g:
                lm = min(localmaps, key=lambda p: abs(p[0] - tw_))
                if abs(lm[0] - tw_) < 3.0:
                    offs, costs = costmap_cross_section(lm[1], g[2])
                    snap['costmap_age_s'] = round(abs(lm[0] - tw_), 2)
                    snap['costmap_offsets'] = offs
                    snap['costmap_costs'] = costs
                    free = [o for o, c in zip(offs, costs) if 0 <= c < 99]
                    snap['free_band_m'] = (round(max(free) - min(free), 2)
                                           if free else 0.0)
                    mid = costs[len(costs) // 2]
                    snap['cost_at_robot'] = mid
            r['worst_crawl'] = snap

    # --- what Nav2 complained about ---------------------------------
    msgs = Counter()
    for (_, lvl, nm, msg) in logs:
        key = msg.split('(')[0].strip()[:90]
        msgs[f'{nm}: {key}'] += 1
    r['warnings'] = dict(msgs.most_common(12))
    # The three events that name a mechanism directly, pulled out of the
    # free-text pile so they can be counted across runs.
    r['n_progress_failures'] = sum(
        1 for (_, _, _, m) in logs if 'Failed to make progress' in m)
    r['n_loop_rate_misses'] = sum(
        1 for (_, _, _, m) in logs if 'missed its desired rate' in m)
    r['n_stale_cmd_drops'] = sum(
        1 for (_, _, _, m) in logs if 'Ignoring the received message' in m)
    if r['n_loop_rate_misses']:
        rates = []
        for (_, _, _, m) in logs:
            if 'Current loop rate is' in m:
                try:
                    rates.append(float(m.split('Current loop rate is')[1]
                                       .split('Hz')[0].strip()))
                except (ValueError, IndexError):
                    pass
        if rates:
            r['loop_rate_min_hz'] = round(min(rates), 2)
    return r


def write_trace(node, path, t0, t1):
    """10 Hz resampled trace of the whole chain, for pinpointing the
    moment a leg diverges. One row per 0.1 sim-second."""
    with node._lock:
        gt = list(zip(*node.gt.window(t0, t1)))
        series = {
            'nav': dict(zip(*[list(x) for x in node.nav.window(t0, t1)])),
            'smooth': dict(zip(*[list(x) for x in node.smooth.window(t0, t1)])),
            'out': dict(zip(*[list(x) for x in node.out.window(t0, t1)])),
            'wheel': dict(zip(*[list(x) for x in node.wheel.window(t0, t1)])),
        }
        gt_t, gt_v = node.gt.window(t0, t1)
        cm_t, cm_v = node.cmstate.window(t0, t1)
        sc_t, sc_v = node.scanmin.window(t0, t1)
        ev_t, ev_v = node.evals.window(t0, t1)

    def last_at(ts, vs, t):
        i = bisect.bisect_right(ts, t) - 1
        return vs[i] if i >= 0 else None

    nav_t, nav_v = list(series['nav'].keys()), list(series['nav'].values())
    sm_t, sm_v = list(series['smooth'].keys()), list(series['smooth'].values())
    ot, ov = list(series['out'].keys()), list(series['out'].values())
    wt, wv = list(series['wheel'].keys()), list(series['wheel'].values())

    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_rel', 'x', 'y', 'yaw', 'v_act', 'w_act',
                    'v_nav', 'w_nav', 'v_smoothed', 'v_cmdvel', 'v_wheel',
                    'cm_action', 'cm_polygon', 'scan_min',
                    'dwb_n', 'dwb_illegal', 'dwb_best_vx', 'dwb_best_total'])
        t = t0
        while t <= t1:
            g = last_at(gt_t, gt_v, t)
            n = last_at(nav_t, nav_v, t)
            s = last_at(sm_t, sm_v, t)
            o = last_at(ot, ov, t)
            wl = last_at(wt, wv, t)
            c = last_at(cm_t, cm_v, t)
            sc = last_at(sc_t, sc_v, t)
            e = last_at(ev_t, ev_v, t)
            w.writerow([
                round(t - t0, 2),
                round(g[0], 4) if g else '', round(g[1], 4) if g else '',
                round(g[2], 4) if g else '', round(g[3], 4) if g else '',
                round(g[4], 4) if g else '',
                round(n[0], 4) if n else '', round(n[1], 4) if n else '',
                round(s[0], 4) if s else '', round(o[0], 4) if o else '',
                round(wl[0], 4) if wl else '',
                c[0] if c else '', c[1] if c else '',
                round(sc, 3) if sc is not None else '',
                e['n'] if e else '', e['n_illegal'] if e else '',
                (round(e['best']['vx'], 4) if e and e['best'] else ''),
                (round(e['best']['total'], 2) if e and e['best'] else ''),
            ])
            t += 0.1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--tag', default='run', help='label for this batch')
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--timeout', type=float, default=90.0,
                    help='per-leg wall-clock cap, seconds')
    ap.add_argument('--out', default='/tmp/navbench')
    ap.add_argument('--only', default=None,
                    help='comma-separated scenario names to run')
    # C2-NAV.7. A scenario's goal is a WORLD position in TOUR above, and
    # moving one is a benchmark change, not a tuning knob -- so it is an
    # explicit override rather than an edit. TOUR stays byte-identical to
    # the C2-NAV.0 baseline every earlier experiment ran against, the
    # override is visible on the command line, and the goal that actually
    # ran is written into each leg record as `goal_world`, so a result
    # cannot be silently attributed to the wrong scenario.
    ap.add_argument('--goal', action='append', default=None,
                    metavar='NAME:X,Y',
                    help='override one scenario goal, world coords; '
                         'repeatable. Default: the committed TOUR.')
    # C2-NAV.8. A per-leg measurement window, for the reason in
    # apply_leg_timeouts above. Default-off: without it every leg gets
    # --timeout, as every earlier experiment ran.
    ap.add_argument('--leg-timeout', action='append', default=None,
                    metavar='NAME:SECONDS',
                    help='override one scenario wall-clock cap; '
                         'repeatable. Default: --timeout for every leg.')
    # C2-NAV.10. A route-level insertion, not a tuning knob or a goal
    # move: the named scenario's own goal is untouched, an extra leg
    # with its own NavigateToPose call runs immediately before it.
    # Default-off, exactly as --goal and --leg-timeout are: absent the
    # flag every C2-NAV.0 ... C2-NAV.9 command reproduces unchanged.
    ap.add_argument('--waypoint', action='append', default=None,
                    metavar='BEFORE:X,Y',
                    help='insert one intermediate leg immediately before '
                         'a named scenario, world coords; repeatable. '
                         'Default: no insertion.')
    # C2-NAV.11. Unlike --waypoint (a separate NavigateToPose leg), this
    # attaches one or more intermediate poses to a named scenario's OWN
    # goal so both are sent in a single NavigateThroughPoses request --
    # no re-plan boundary between them. Default-off, exactly like every
    # other override above: absent the flag every C2-NAV.0 ... C2-NAV.10
    # command reproduces unchanged (send_leg / NavigateToPose for every
    # leg).
    ap.add_argument('--through-pose', action='append', default=None,
                    metavar='BEFORE:X,Y',
                    help='attach one intermediate pose to a named '
                         "scenario's own goal as part of ONE continuous "
                         'NavigateThroughPoses request, world coords; '
                         'repeatable per scenario. Default: none.')
    args, rest = ap.parse_known_args(argv if argv is not None else sys.argv[1:])

    rclpy.init(args=rest)
    node = NavBench()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    os.makedirs(args.out, exist_ok=True)
    tracedir = os.path.join(args.out, f'{args.tag}_traces')
    os.makedirs(tracedir, exist_ok=True)

    # Wait for ground truth and the map before starting anything.
    for _ in range(200):
        with node._lock:
            if node._sim_ok and node.grid is not None:
                break
        time.sleep(0.1)
    occupied = node.occupied_cells()
    print(f'[nav_bench] map: {len(occupied)} occupied cells; '
          f'evaluation topic: {"yes" if node._eval_ok else "NO"}')

    tour = apply_goal_overrides(TOUR, args.goal)
    if tour is None:
        return 2
    # --only filters by ORIGINAL scenario name, before --waypoint inserts
    # anything: filtering after insertion would silently drop an inserted
    # leg whenever its own generated name (e.g. "enclosure_entry_waypoint")
    # is not itself listed in --only, even though the scenario it leads
    # into is. Filter first, then thread each surviving scenario through
    # its waypoint, so `--only enclosure_entry --waypoint enclosure_entry:
    # X,Y` keeps the leg that makes the waypoint apply at all.
    if args.only:
        want = set(args.only.split(','))
        tour = [t for t in tour if t[0] in want]
    tour = apply_waypoint_insert(tour, args.waypoint)
    if tour is None:
        return 2
    # C2-NAV.11. Consulted by name, after --only and --waypoint have both
    # already settled the tour's shape -- no ordering hazard vs those two,
    # since this never changes which scenarios exist, only how ONE of
    # them is driven.
    through = apply_through_poses(tour, args.through_pose)
    if through is None:
        return 2
    timeouts = apply_leg_timeouts(tour, args.timeout, args.leg_timeout)
    if timeouts is None:
        return 2

    results = []
    for rep in range(args.repeats):
        for (name, gx, gy, probe) in tour:
            leg_to = timeouts[name]
            via = through.get(name, [])
            if via:
                print(f'[nav_bench] rep {rep} leg {name} -> world '
                      f'{via} -> ({gx}, {gy}) cap {leg_to}s '
                      f'[NavigateThroughPoses, 1 request]', flush=True)
                status, t0, t1, early_plan = node.send_multi_leg(
                    via + [(gx, gy)], leg_to)
            else:
                print(f'[nav_bench] rep {rep} leg {name} -> world '
                      f'({gx}, {gy}) cap {leg_to}s', flush=True)
                status, t0, t1 = node.send_leg(gx, gy, leg_to)
                early_plan = None
            if t0 is None:
                print(f'[nav_bench]   {status}')
                results.append({'scenario': name, 'rep': rep,
                                'status': status, 'timeout_s': leg_to})
                continue
            time.sleep(1.0)             # let trailing messages land
            t1 = node.now()[0]
            rec = summarise(node, name, probe, (gx, gy), status, t0, t1,
                            occupied)
            rec['rep'] = rep
            rec['tag'] = args.tag
            rec['timeout_s'] = leg_to
            if via:
                rec['through_poses_world'] = via
                rec['action'] = 'NavigateThroughPoses'
                if early_plan is not None:
                    ts, pts = early_plan
                    rec['early_plan_ts_sim_s'] = round(ts, 3)
                    rec['early_plan_ts_offset_from_t0_s'] = round(ts - t0, 3)
                    rec['early_plan_n_poses'] = len(pts)
                    rec['early_plan_endpoint_map'] = (
                        [round(pts[-1][0], 3), round(pts[-1][1], 3)]
                        if pts else None)
                    # Distance from the plan's own last point to the FINAL
                    # goal (world frame): small means this one /plan
                    # message, captured before the robot could plausibly
                    # have reached even the first through-pose, already
                    # runs to the final goal -- the continuity proof.
                    if pts:
                        endpoint_world = (pts[-1][0] - WORLD_TO_MAP_X,
                                          pts[-1][1] - WORLD_TO_MAP_Y)
                        rec['early_plan_endpoint_to_final_goal_m'] = round(
                            math.dist(endpoint_world, (gx, gy)), 3)
                else:
                    rec['early_plan_ts_sim_s'] = None
                    rec['note_early_plan'] = (
                        'no /plan captured within 8s of goal acceptance')
            else:
                rec['action'] = 'NavigateToPose'
            results.append(rec)
            write_trace(node, os.path.join(
                tracedir, f'{name}_rep{rep}.csv'), t0, t1)
            print(f'[nav_bench]   {status} t={rec["duration_sim_s"]}s '
                  f'len={rec.get("path_len_m")}m '
                  f'clear={rec.get("min_clearance_m")}m '
                  f'stops={rec.get("n_stops")} '
                  f'v_cmd_med={rec.get("v_cmd_med")} '
                  f'illegal={rec.get("dwb_illegal_frac")}', flush=True)

    out = os.path.join(args.out, f'{args.tag}.json')
    with open(out, 'w') as f:
        json.dump({'tag': args.tag,
                   'robot_radius': ROBOT_RADIUS,
                   'inflation_radius': INFLATION_RADIUS,
                   'controller_frequency': CONTROLLER_FREQUENCY,
                   'legs': results}, f, indent=1)
    print(f'[nav_bench] wrote {out}')

    ex.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
