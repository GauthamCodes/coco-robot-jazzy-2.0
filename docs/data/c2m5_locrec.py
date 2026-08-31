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
c2m5_locrec — the C2-M5.0 localization recorder.

**This is instrumentation, not a robot capability.** It subscribes and
writes a CSV. It publishes nothing, calls no service, and is not part of
any launch file. C2-M5.0's rule is OBSERVE -> CLASSIFY -> DEFINE -> only
then RECOVER, and this file is the whole of "observe".

Why a recorder at all
---------------------
The four recorded nav-home legs (RESULTS.md, "C2-M1.5 runtime integrity")
failed twice, by what look like two different mechanisms, and neither was
measured while it happened — both were reconstructed afterwards from
``~/.ros/log``. Reconstruction gave the *outcome* of each run and almost
none of the *signals*: no covariance trace, no map->odom history, no
command rate, no collision-monitor state. A threshold cannot be
calibrated against a post-mortem, so this records the signals live.

The ground-truth boundary, which is the point
---------------------------------------------
Gazebo's ``/model/coco/odometry`` is the model's true world pose. It is
recorded, in columns that all begin ``gt_``, and it exists **only to
score the other columns offline**. Nothing computed from a ``gt_`` column
may enter a deployable health signal — that is C2-M5's stated rule and
the reason the prefix is uniform enough to grep for:

    csvcut -c "$(head -1 run.csv | tr ',' '\\n' | grep -v '^gt_')"

Everything else here is available to the robot: the map it was given,
its own laser, its own TF tree, its own topics.

The candidate signal, and why it is not covariance
--------------------------------------------------
AMCL's covariance is the spread of its own particle set. It answers "how
much do my hypotheses disagree", which is not the same question as "is my
hypothesis right". The M6 run-15 failure family is a *confidently wrong*
pose: the filter collapses onto a wrong mode and reports a small
covariance while doing it. So this records covariance — it may yet be
informative — and alongside it computes the quantity AMCL's own sensor
model is built on and never publishes:

  ``lik_mean_d``  the mean distance, in metres, from a laser endpoint to
                  the nearest occupied cell of the map, with the endpoint
                  placed by the CURRENT map->laser transform.

That is the likelihood field ``nav2_amcl`` scores particles against
(``laser_model_type: likelihood_field``), evaluated once at the pose
AMCL actually published. It needs the map, the scan and TF, all of which
the robot has. A pose that is confidently wrong puts the endpoints in
the wrong place and this number rises; a pose that is uncertain but
correct leaves it low. Whether it separates the recorded failures is the
measurement, not an assumption — see RESULTS.md.

**Its known blind spot is stated up front.** The raised platform and the
ramp are not in ``coco_world.pgm`` — the map is a 2D slice of the flat
world — so any sample taken on the platform scores badly for a reason
that has nothing to do with localization health. The column is only
interpretable on the flat, which is where RETURN_HOME runs. The recorder
writes the mission state beside every row so that filter is possible.

Real-time factor
----------------
``rtf`` is d(sim)/d(wall) over a one-second window. It is here because
"the control loop missed its desired rate of 10 Hz, current 4.8077 Hz"
is an un-isolated confound in one of the two recorded failures, and that
message is measured by nav2 in ROS time. Without RTF beside it there is
no way to tell a loaded machine from a genuinely slow controller.

Rates
-----
Every ``hz_`` column counts messages that arrived in the last second of
**simulation** time, which is the unit nav2's own rate complaint uses.
The four command topics are recorded separately because they are four
different stages of one chain and the interesting question is where a
command stops:

    controller_server --/cmd_vel_nav--> velocity_smoother
      --/cmd_vel_smoothed--> collision_monitor --/cmd_vel-->
      cmd_vel_relay --/cmd_vel_nav--> cmd_vel_arbiter
      --/diff_drive_controller/cmd_vel--> DiffDriveController

Note that ``/cmd_vel_nav`` appears twice in that chain. Whether it really
does at runtime is checked by ``--topology``, which prints the publisher
and subscriber list for each stage and exits.

**Subscribe to the wheel topic as TwistStamped.**
``/diff_drive_controller/cmd_vel`` carries both types; the arbiter
publishes ``TwistStamped`` and a ``Twist`` subscriber sees nothing and
raises nothing. It cost C2-M3.1 a whole run and it is in CLAUDE.md's
trap table.

Usage
-----
    # alongside a running mission stack, before /mission/start
    python3 c2m5_locrec.py --out c2m5_run.csv --tag healthy

    # what is actually wired to what, then exit
    python3 c2m5_locrec.py --topology

Stops on Ctrl-C, on ``--duration``, or when ``/mission/state`` reaches a
terminal state and ``--stop-on-terminal`` is set.
"""

import argparse
import csv
import math
import os
import sys
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import tf2_ros

try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:                                    # pragma: no cover
    CollisionMonitorState = None

# GoalStatus codes, spelled out so the CSV is readable without the enum.
GOAL_STATUS = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
               4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
# CollisionMonitorState.action_type, likewise.
CM_ACTION = {0: 'DO_NOTHING', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH',
             4: 'LIMIT'}

TERMINAL_STATES = ('COMPLETE', 'ABORT')

# The chain, in order, as topic -> the stage that publishes it.
COMMAND_CHAIN = [
    ('/cmd_vel_nav', 'controller_server, and cmd_vel_relay'),
    ('/cmd_vel_smoothed', 'velocity_smoother'),
    ('/cmd_vel', 'collision_monitor'),
    ('/diff_drive_controller/cmd_vel', 'cmd_vel_arbiter'),
]

# Beams per scan used for the likelihood field. nav2_amcl's own
# `max_beams` is 60; matching it keeps the number comparable to what the
# filter itself scores and keeps the cost per sample trivial.
LIK_BEAMS = 60
# An endpoint this close to an occupied cell counts as "explained".
# 0.10 m is two map cells at the map's 0.05 m resolution.
LIK_NEAR_M = 0.10


def yaw_of(q):
    """Yaw from a quaternion, in radians."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def state_of(line):
    """The bare state label out of a /mission/state key=value line."""
    for token in (line or '').split():
        if token.startswith('state='):
            return token[len('state='):] or '--'
    return (line or '--').strip() or '--'


class LikelihoodField:
    """Distance-to-nearest-obstacle over the occupancy map.

    This is the deployable half of the recorder: a map the robot was
    given, and nothing else. ``distance(xs, ys)`` returns metres to the
    nearest occupied cell for world points, with points outside the map
    returned as NaN rather than clamped — a beam that leaves the map is
    not evidence of a good pose or a bad one, and averaging a made-up
    number in would be the same mistake as inventing a threshold.
    """

    def __init__(self, map_yaml):
        import yaml
        from PIL import Image
        from scipy.ndimage import distance_transform_edt

        with open(map_yaml) as fh:
            meta = yaml.safe_load(fh)
        image = meta['image']
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(map_yaml), image)
        self.res = float(meta['resolution'])
        self.origin = (float(meta['origin'][0]), float(meta['origin'][1]))
        occupied_thresh = float(meta.get('occupied_thresh', 0.65))
        negate = int(meta.get('negate', 0))

        pix = np.asarray(Image.open(image).convert('L'), dtype=np.float64)
        # map_server's convention: darker is more occupied, unless negate.
        occ = pix / 255.0 if negate else (255.0 - pix) / 255.0
        occupied = occ > occupied_thresh
        # Row 0 of the image is the TOP, i.e. the highest y. Flip so that
        # index 0 is the map's y origin and the arithmetic below is the
        # plain one.
        self.occupied = np.flipud(occupied)
        self.h, self.w = self.occupied.shape
        # Distance, in cells, from every cell to the nearest occupied one.
        self.dist = distance_transform_edt(~self.occupied) * self.res
        self.n_occupied = int(self.occupied.sum())

    def distance(self, xs, ys):
        """Metres to the nearest occupied cell, NaN outside the map."""
        j = np.floor((np.asarray(xs) - self.origin[0]) / self.res)
        i = np.floor((np.asarray(ys) - self.origin[1]) / self.res)
        inside = (i >= 0) & (i < self.h) & (j >= 0) & (j < self.w)
        out = np.full(np.shape(xs), np.nan, dtype=np.float64)
        if inside.any():
            out[inside] = self.dist[i[inside].astype(int),
                                    j[inside].astype(int)]
        return out


class RateWindow:
    """Messages per second of SIMULATION time, over a sliding window."""

    def __init__(self, window=1.0):
        self.window = window
        self._t = deque()

    def mark(self, t_sim):
        self._t.append(t_sim)

    def hz(self, t_sim):
        while self._t and t_sim - self._t[0] > self.window:
            self._t.popleft()
        if len(self._t) < 2:
            return float(len(self._t)) / self.window
        span = self._t[-1] - self._t[0]
        return (len(self._t) - 1) / span if span > 1e-9 else float('nan')


COLUMNS = [
    # bookkeeping
    't_wall', 't_sim', 'rtf', 'state', 'tag',
    # AMCL: the pose nav2 steers by, and its own opinion of itself
    'amcl_x', 'amcl_y', 'amcl_yaw',
    'amcl_cxx', 'amcl_cyy', 'amcl_caa', 'amcl_age', 'amcl_n',
    # the correction AMCL applies to odometry
    'mo_x', 'mo_y', 'mo_yaw', 'mo_age', 'mo_step',
    # wheel odometry: dead reckoning, no map
    'odom_x', 'odom_y', 'odom_yaw', 'odom_vx', 'odom_wz', 'odom_age',
    # the deployable candidate: scan against map, under the AMCL pose
    'lik_mean_d', 'lik_p90_d', 'lik_frac_near', 'lik_n', 'scan_age',
    # the command chain, stage by stage
    'hz_cmd_nav', 'hz_cmd_smoothed', 'hz_cmd_out', 'hz_cmd_wheels',
    'wheel_vx', 'wheel_wz', 'hz_scan', 'hz_amcl', 'hz_odom',
    # what the safety layer is doing
    'cm_action', 'cm_polygon', 'hz_cm',
    # what nav2 thinks about the goal
    'nav_status', 'plan_n', 'plan_len', 'goal_x', 'goal_y',
    # GROUND TRUTH — offline scoring only, never a health input
    'gt_x', 'gt_y', 'gt_yaw', 'gt_age',
]


class LocRecorder(Node):
    """Subscribes to the localization stack and writes one CSV row per tick."""

    def __init__(self, args):
        # use_sim_time is forced ON, not left to the launch environment.
        # Measured the hard way in the first recording: without it the
        # node clock is system time while every message stamp is
        # simulation time, so every *_age column came out as the Unix
        # epoch, every hz_ column was a wall-clock rate wearing a
        # simulation-time label, and `rtf` was d(wall)/d(wall) = 1.000
        # exactly — a number that looks like a healthy simulator and is
        # in fact a tautology.
        overrides = [Parameter('use_sim_time', Parameter.Type.BOOL,
                               not args.wall_clock)]
        super().__init__('c2m5_locrec', parameter_overrides=overrides)
        self.args = args
        self.tag = args.tag
        self.t0_wall = time.monotonic()
        self.t0_sim = None
        self.stop = False

        self.field = None
        if args.map:
            self.field = LikelihoodField(args.map)
            self.get_logger().info(
                f'likelihood field: {self.field.w}x{self.field.h} cells, '
                f'{self.field.n_occupied} occupied, res {self.field.res}')

        # ── latest sample of everything ──────────────────────────────────
        self.amcl = None          # (x, y, yaw, cxx, cyy, caa, stamp)
        self.amcl_n = 0
        self.odom = None          # (x, y, yaw, vx, wz, stamp)
        self.gt = None            # (x, y, yaw, stamp)
        self.scan = None
        self.state = '--'
        self.cm = ('--', '--')
        self.nav_status = '--'
        self.plan = None          # (n, length, gx, gy)
        self.plan_n_total = 0
        self.wheel = (float('nan'), float('nan'))
        self._mo_prev = None

        # ── rates ────────────────────────────────────────────────────────
        self.rate = {k: RateWindow() for k in (
            'cmd_nav', 'cmd_smoothed', 'cmd_out', 'cmd_wheels',
            'scan', 'amcl', 'odom', 'cm')}
        self._clock_hist = deque()

        # ── QoS ──────────────────────────────────────────────────────────
        # AMCL latches its pose TRANSIENT_LOCAL, and with update_min_a 0.2
        # a stationary robot emits no new one; a VOLATILE subscriber that
        # starts late would therefore see nothing at all.
        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # BEST_EFFORT on the subscriber matches a BEST_EFFORT *or* a
        # RELIABLE publisher, so it is the safe choice for sensor streams
        # whose publisher QoS is the bridge's business, not ours.
        sensor = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, latched)
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self._on_odom, 10)
        self.create_subscription(
            Odometry, '/model/coco/odometry', self._on_gt, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, sensor)
        self.create_subscription(
            String, '/mission/state', self._on_state, 10)
        self.create_subscription(Path, '/plan', self._on_plan, 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._on_nav_status, 10)
        if CollisionMonitorState is not None:
            self.create_subscription(
                CollisionMonitorState, '/collision_monitor_state',
                self._on_cm, 10)
        else:
            self.get_logger().warn(
                'nav2_msgs/CollisionMonitorState unavailable; cm_* stay --')

        # TwistStamped everywhere: the wheel topic carries both types and
        # a Twist subscriber is silently blind. CLAUDE.md, trap table.
        for topic, key in (('/cmd_vel_nav', 'cmd_nav'),
                           ('/cmd_vel_smoothed', 'cmd_smoothed'),
                           ('/cmd_vel', 'cmd_out'),
                           ('/diff_drive_controller/cmd_vel', 'cmd_wheels')):
            self.create_subscription(
                TwistStamped, topic,
                (lambda msg, k=key: self._on_cmd(k, msg)), 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── the file ─────────────────────────────────────────────────────
        self.fh = open(args.out, 'w', newline='')
        self.csv = csv.writer(self.fh)
        self.csv.writerow(COLUMNS)
        self.rows = 0
        self.events = []

        # The tick runs on a STEADY clock, for mission_executive's reason:
        # a node-clock timer under use_sim_time stops firing the moment
        # /clock stops, and a recorder that stops recording is exactly
        # what you did not want when the simulator died. Sampling on the
        # steady clock is also what makes `rtf` measurable at all.
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / args.hz, self._tick, clock=self._steady)
        self.get_logger().info(
            f'recording to {args.out} at {args.hz} Hz, tag={self.tag}, '
            f'use_sim_time={not args.wall_clock}')

    # ── time ─────────────────────────────────────────────────────────────
    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def stamp_of(msg):
        s = msg.header.stamp
        return s.sec + s.nanosec * 1e-9

    def age(self, stamp, t_sim):
        return float('nan') if stamp is None else t_sim - stamp

    # ── callbacks ────────────────────────────────────────────────────────
    def _on_amcl(self, msg):
        p = msg.pose.pose
        c = msg.pose.covariance
        self.amcl = (p.position.x, p.position.y, yaw_of(p.orientation),
                     c[0], c[7], c[35], self.stamp_of(msg))
        self.amcl_n += 1
        self.rate['amcl'].mark(self.sim_now())

    def _on_odom(self, msg):
        p = msg.pose.pose
        t = msg.twist.twist
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation),
                     t.linear.x, t.angular.z, self.stamp_of(msg))
        self.rate['odom'].mark(self.sim_now())

    def _on_gt(self, msg):
        p = msg.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation),
                   self.stamp_of(msg))

    def _on_scan(self, msg):
        self.scan = msg
        self.rate['scan'].mark(self.sim_now())

    def _on_state(self, msg):
        # /mission/state is a whole key=value line, not a bare label:
        #   state=RETURN_HOME prev=DESCEND event=run elapsed=12.3 ...
        # Reading it raw makes every 2 Hz republication look like a
        # transition (elapsed= changes each time), floods the event log,
        # and means --stop-on-terminal never matches COMPLETE.
        new = state_of(msg.data)
        if new != self.state:
            self.note(f'state {self.state} -> {new}')
            self.state = new
            if self.args.stop_on_terminal and new in TERMINAL_STATES:
                self.note(f'terminal state {new}; stopping')
                self.stop = True

    def _on_cm(self, msg):
        new = (CM_ACTION.get(msg.action_type, str(msg.action_type)),
               msg.polygon_name or '--')
        if new != self.cm:
            self.note(f'collision_monitor {self.cm} -> {new}')
            self.cm = new
        self.rate['cm'].mark(self.sim_now())

    def _on_nav_status(self, msg):
        if not msg.status_list:
            return
        code = msg.status_list[-1].status
        new = GOAL_STATUS.get(code, str(code))
        if new != self.nav_status:
            self.note(f'navigate_to_pose {self.nav_status} -> {new}')
            self.nav_status = new

    def _on_plan(self, msg):
        n = len(msg.poses)
        length = 0.0
        for a, b in zip(msg.poses, msg.poses[1:]):
            length += math.hypot(b.pose.position.x - a.pose.position.x,
                                 b.pose.position.y - a.pose.position.y)
        if n:
            end = msg.poses[-1].pose.position
            self.plan = (n, length, end.x, end.y)
        else:
            self.plan = (0, 0.0, float('nan'), float('nan'))
        self.plan_n_total += 1

    def _on_cmd(self, key, msg):
        self.rate[key].mark(self.sim_now())
        if key == 'cmd_wheels':
            self.wheel = (msg.twist.linear.x, msg.twist.angular.z)

    def note(self, text):
        t = self.sim_now()
        line = f'{t - (self.t0_sim or t):8.2f}  {self.state:<18} {text}'
        self.events.append(line)
        self.get_logger().info(text)

    # ── the likelihood field, evaluated at the CURRENT map->laser TF ─────
    def likelihood(self):
        """(mean_d, p90_d, frac_near, n, scan_age) — map + scan + TF only."""
        nan4 = (float('nan'),) * 3 + (0, float('nan'))
        if self.field is None or self.scan is None:
            return nan4
        scan = self.scan
        frame = scan.header.frame_id or 'base_footprint'
        try:
            # Latest available, not the scan's own stamp: AMCL republishes
            # map->odom on its own schedule and asking for an exact match
            # fails far more often than it helps. The staleness that
            # introduces is bounded by the transform_tolerance (0.5 s) and
            # is reported as scan_age beside it.
            tf = self.tf_buffer.lookup_transform(
                'map', frame, rclpy.time.Time())
        except Exception:
            return nan4
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        th = yaw_of(tf.transform.rotation)

        n = len(scan.ranges)
        if n == 0:
            return nan4
        step = max(1, n // LIK_BEAMS)
        idx = np.arange(0, n, step)
        r = np.asarray(scan.ranges, dtype=np.float64)[idx]
        a = scan.angle_min + idx * scan.angle_increment
        good = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
        if not good.any():
            return nan4
        r, a = r[good], a[good]
        xs = tx + r * np.cos(th + a)
        ys = ty + r * np.sin(th + a)
        d = self.field.distance(xs, ys)
        d = d[np.isfinite(d)]
        if d.size == 0:
            return nan4
        return (float(d.mean()), float(np.percentile(d, 90)),
                float((d <= LIK_NEAR_M).mean()), int(d.size),
                self.sim_now() - self.stamp_of(scan))

    # ── map -> odom, the correction AMCL is applying ─────────────────────
    def map_odom(self, t_sim):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time())
        except Exception:
            return (float('nan'),) * 5
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_of(tf.transform.rotation)
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        step = float('nan')
        if self._mo_prev is not None:
            step = math.hypot(x - self._mo_prev[0], y - self._mo_prev[1])
        self._mo_prev = (x, y)
        return x, y, yaw, t_sim - stamp, step

    # ── the tick ─────────────────────────────────────────────────────────
    def _tick(self):
        t_wall = time.monotonic() - self.t0_wall
        t_sim_abs = self.sim_now()
        if self.t0_sim is None:
            self.t0_sim = t_sim_abs
        t_sim = t_sim_abs - self.t0_sim

        # RTF over a one-second wall window.
        self._clock_hist.append((t_wall, t_sim))
        while len(self._clock_hist) > 2 and t_wall - self._clock_hist[0][0] > 1.0:
            self._clock_hist.popleft()
        rtf = float('nan')
        if len(self._clock_hist) >= 2:
            dw = t_wall - self._clock_hist[0][0]
            ds = t_sim - self._clock_hist[0][1]
            if dw > 1e-6:
                rtf = ds / dw

        amcl = self.amcl or (float('nan'),) * 7
        odom = self.odom or (float('nan'),) * 6
        gt = self.gt or (float('nan'),) * 4
        mo = self.map_odom(t_sim_abs)
        lik = self.likelihood()
        plan = self.plan or (0, float('nan'), float('nan'), float('nan'))

        self.csv.writerow([
            f'{t_wall:.3f}', f'{t_sim:.3f}', f'{rtf:.3f}', self.state, self.tag,
            f'{amcl[0]:.4f}', f'{amcl[1]:.4f}', f'{amcl[2]:.4f}',
            f'{amcl[3]:.6f}', f'{amcl[4]:.6f}', f'{amcl[5]:.6f}',
            f'{self.age(amcl[6], t_sim_abs):.3f}', self.amcl_n,
            f'{mo[0]:.4f}', f'{mo[1]:.4f}', f'{mo[2]:.4f}',
            f'{mo[3]:.3f}', f'{mo[4]:.5f}',
            f'{odom[0]:.4f}', f'{odom[1]:.4f}', f'{odom[2]:.4f}',
            f'{odom[3]:.4f}', f'{odom[4]:.4f}',
            f'{self.age(odom[5], t_sim_abs):.3f}',
            f'{lik[0]:.4f}', f'{lik[1]:.4f}', f'{lik[2]:.4f}', lik[3],
            f'{lik[4]:.3f}',
            f'{self.rate["cmd_nav"].hz(t_sim_abs):.2f}',
            f'{self.rate["cmd_smoothed"].hz(t_sim_abs):.2f}',
            f'{self.rate["cmd_out"].hz(t_sim_abs):.2f}',
            f'{self.rate["cmd_wheels"].hz(t_sim_abs):.2f}',
            f'{self.wheel[0]:.4f}', f'{self.wheel[1]:.4f}',
            f'{self.rate["scan"].hz(t_sim_abs):.2f}',
            f'{self.rate["amcl"].hz(t_sim_abs):.2f}',
            f'{self.rate["odom"].hz(t_sim_abs):.2f}',
            self.cm[0], self.cm[1],
            f'{self.rate["cm"].hz(t_sim_abs):.2f}',
            self.nav_status, plan[0], f'{plan[1]:.3f}',
            f'{plan[2]:.4f}', f'{plan[3]:.4f}',
            f'{gt[0]:.4f}', f'{gt[1]:.4f}', f'{gt[2]:.4f}',
            f'{self.age(gt[3], t_sim_abs):.3f}',
        ])
        self.rows += 1
        if self.rows % 50 == 0:
            self.fh.flush()
        if self.args.duration and t_wall >= self.args.duration:
            self.note(f'duration {self.args.duration}s reached; stopping')
            self.stop = True

    def close(self):
        self.fh.flush()
        self.fh.close()
        if self.args.events:
            with open(self.args.events, 'w') as fh:
                fh.write('\n'.join(self.events) + '\n')
        print(f'\n{self.rows} rows -> {self.args.out}', file=sys.stderr)
        if self.args.events:
            print(f'{len(self.events)} events -> {self.args.events}',
                  file=sys.stderr)


def show_topology(node):
    """Print who publishes and who subscribes to each stage of the chain.

    Read from the graph, not from the launch files, because a remapping
    applied three includes deep is exactly the thing a launch file does
    not tell you.
    """
    print('\ncommand chain, as the running graph has it:')
    for topic, expected in COMMAND_CHAIN:
        pubs = node.get_publishers_info_by_topic(topic)
        subs = node.get_subscriptions_info_by_topic(topic)
        print(f'\n  {topic}   (expected publisher: {expected})')
        for p in pubs:
            print(f'    pub  {p.node_name:<28} {p.topic_type}')
        for s in subs:
            print(f'    sub  {s.node_name:<28} {s.topic_type}')
        if not pubs:
            print('    pub  (none)')
        if not subs:
            print('    sub  (none)')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument('--out', default='c2m5_run.csv')
    ap.add_argument('--events', default=None,
                    help='write the transition log here as well')
    ap.add_argument('--tag', default='--',
                    help='free text copied into every row (e.g. healthy)')
    ap.add_argument('--hz', type=float, default=10.0)
    ap.add_argument('--duration', type=float, default=0.0,
                    help='wall seconds; 0 means run until interrupted')
    ap.add_argument('--map', default=None,
                    help='map yaml for the likelihood field. Without it '
                         'the lik_* columns stay NaN.')
    ap.add_argument('--stop-on-terminal', action='store_true',
                    help='stop when /mission/state reaches COMPLETE/ABORT')
    ap.add_argument('--topology', action='store_true',
                    help='print the command chain from the live graph, exit')
    ap.add_argument('--wall-clock', action='store_true',
                    help='do NOT use simulation time. Only for a run with '
                         'no /clock at all; every age and rate is then in '
                         'wall seconds and rtf is meaningless by '
                         'construction.')
    args = ap.parse_args(argv)

    rclpy.init()
    if args.topology:
        node = rclpy.create_node('c2m5_topology')
        # The graph is discovered asynchronously; without a pause the
        # answer is "nothing is running", which is a lie about the graph
        # rather than a fact about the robot.
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        show_topology(node)
        node.destroy_node()
        rclpy.shutdown()
        return 0

    node = LocRecorder(args)
    try:
        while rclpy.ok() and not node.stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
