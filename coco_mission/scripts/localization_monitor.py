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
The ROS face of the localization health signal.

Imports :mod:`localization_health` and never the reverse — the rule stays
pure so the tests can drive a whole excursion in a loop with no ROS, and
this file holds every part that needs a clock, a transform or a topic.
Same split as ``mission_states.py`` / ``mission_executive.py``.

**This node publishes and does not drive.** It creates no publisher on
any ``cmd_vel`` topic, so ``cmd_vel_arbiter`` remains the sole publisher
to the controller exactly as CLAUDE.md §4 requires. It also takes no
recovery action: it says whether the scan agrees with the map, and the
mission executive decides what that is worth. C2-M5.0 measured the
signal, C2-M5.1 acts on it, and keeping the two in different processes is
what lets Experiment 1 run the monitor over a healthy mission with
nothing wired to its output.

What it computes, and what it refuses to
========================================
Once per tick it fills a :class:`localization_health.Observation` from
the map it was given, its own laser, its own TF tree and its own topics,
and hands it to :func:`localization_health.classify`. **No field comes
from the simulator.** There is no ``/model/coco/odometry`` subscription
here and there is not meant to be: ground truth is how the offline
recorder *scores* this signal, never an input to it.

The verdict is then held to :data:`~localization_health.DEGRADED_HOLD_S`
before it is allowed to mean anything, because C2-M5.0 measured that the
healthy run's own worst scan-vs-map samples reach 0.31 m and a
single-sample trigger would have fired on a mission that went home to
0.078 m.

Interfaces
==========

Subscribes
----------
``/scan`` (``sensor_msgs/LaserScan``, **BEST_EFFORT**)
    The reliability is not optional. Camera and lidar topics in this
    project are BEST_EFFORT and a RELIABLE subscriber never matches,
    leaving the node **silently blind** — CLAUDE.md's trap table names
    this one, and a health monitor that has gone blind reporting UNKNOWN
    for a whole mission is exactly the failure it exists to catch.

``/map`` (``nav_msgs/OccupancyGrid``, **TRANSIENT_LOCAL**)
    The map the robot was given. Latched, because ``map_server``
    publishes it once: a VOLATILE subscriber that starts late sees
    nothing at all and the field is never built. Taking it off the topic
    rather than off a YAML path means the monitor scores against the same
    grid AMCL is scoring against, with no second copy to drift.

``/amcl_pose`` (``geometry_msgs/PoseWithCovarianceStamped``, latched)
    Where AMCL thinks the robot is, and its covariance. The covariance is
    **published in the status line and never consulted** — see the
    module docstring of ``localization_health`` for the 24.5 s and the
    wrong-way dip that decided that.

TF
    ``map -> <scan frame>`` to place the endpoints, ``map -> odom`` for
    the freshness check. Latest-available rather than stamp-matched, for
    the reason the recorder gives: AMCL republishes on its own schedule
    and an exact-time lookup fails far more often than it helps.

Publishes
---------
``/localization/health`` (``std_msgs/String``, 10 Hz)
    A ``key=value`` status line, the shape every status topic in this
    project uses, so ``mission_states.parse_kv`` reads it with no new
    parser::

        verdict=CONSISTENT reason=OK degraded=0 healthy=1 held=7.30
        d=0.052 near=0.881 beams=57 mapped=1 sigma=0.376 mo_age=-0.44

    ``degraded`` and ``healthy`` are the **latched** answers — the ones
    that have survived their persistence window — and they are what the
    executive reads. The raw numbers beside them are what a human reads
    when deciding whether the latched answer was right.

Parameters
----------
``rate`` (double, 10.0)
    Publish rate. Matches ``c2m5_locrec.py``'s sampling so a live figure
    and a recorded CSV mean the same thing.
``degraded_hold`` / ``healthy_hold`` (double)
    Persistence windows. Default to the values named in
    ``localization_health`` with the runs that justify them.
``lik_mean_d_max`` / ``lik_frac_near_min`` (double)
    The thresholds. Exposed because a future world needs re-measuring,
    **not** because they are meant to be tuned to make a run pass.
"""

import math
import os
import sys
from dataclasses import replace

from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from sensor_msgs.msg import LaserScan

from std_msgs.msg import String

import tf2_ros

# Same shim mission_executive uses: a script run out of lib/<pkg> gets its
# own directory on sys.path, so the pure module beside it imports by name.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import localization_health as lh  # noqa: E402

from coco_config.robot import (  # noqa: E402
    SENSOR_TOPICS,
    SPAWN_XY,
    is_best_effort,
)

# The map frame is anchored at the spawn pose, so a map-frame x is this
# much less than a world-frame x. mission_states.WORLD_TO_MAP_X is the
# same constant in the other direction; both derive from SPAWN_XY rather
# than repeating -2.0, because the trap that cost C2-M5.0 time was
# subtracting the two frames raw and reading 2.2 m of error on a mission
# that finished 0.078 m from home.
MAP_TO_WORLD_X = SPAWN_XY[0]


def yaw_of(q):
    """Yaw from a quaternion, in radians."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def format_status(verdict, degraded, healthy, held, obs, fix=None, now=None):
    """The status line, in the project's usual ``key=value`` shape."""
    def num(value, fmt='.3f'):
        return '--' if value is None else format(value, fmt)

    if fix is None or now is None:
        fix_fields = ['fix_x=--', 'fix_y=--', 'fix_yaw=--', 'fix_age=--']
    else:
        fx, fy, fyaw, ft = fix
        fix_fields = [f'fix_x={fx:.3f}', f'fix_y={fy:.3f}',
                      f'fix_yaw={fyaw:.3f}', f'fix_age={now - ft:.2f}']

    return ' '.join([
        f'verdict={verdict.verdict}',
        f'reason={verdict.reason}',
        f'degraded={1 if degraded else 0}',
        f'healthy={1 if healthy else 0}',
        f'held={held:.2f}',
        f'd={num(obs.lik_mean_d)}',
        f'near={num(obs.lik_frac_near)}',
        f'beams={obs.lik_beams}',
        f'mapped={1 if obs.on_mapped_ground else 0}',
        f'sigma={num(obs.cov_sigma_xy)}',
        f'mo_age={num(obs.map_odom_age, "+.2f")}',
    ] + fix_fields)


class LocalizationMonitor(Node):
    """Score the scan against the map and publish a held verdict."""

    def __init__(self):
        super().__init__('localization_monitor')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('degraded_hold', lh.DEGRADED_HOLD_S)
        self.declare_parameter('healthy_hold', lh.HEALTHY_HOLD_S)
        self.declare_parameter('lik_mean_d_max', lh.LIK_MEAN_D_MAX)
        self.declare_parameter('lik_frac_near_min',
                               lh.LIK_FRAC_NEAR_DISABLED)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('status_topic', '/localization/health')

        # Built by REPLACING fields on the shipped thresholds, never by
        # naming a subset. Experiment 2 caught the bug that motivates
        # this: the node listed only the two parameters it exposes, so
        # `max_amcl_age` silently fell back to the dataclass default of
        # 5.0 s and the C2-M5.1 decision to switch that check off never
        # reached the running node. The mission degraded on POSE_STALE at
        # startup, spun, and recovered from a fault that did not exist.
        # A partial constructor call is a silent way to ignore a measured
        # decision, so there is now exactly one place the defaults live.
        self.thresholds = replace(
            lh.C2M51_THRESHOLDS,
            lik_mean_d_max=float(
                self.get_parameter('lik_mean_d_max').value),
            lik_frac_near_min=float(
                self.get_parameter('lik_frac_near_min').value))
        self.degraded = lh.Persistence(
            float(self.get_parameter('degraded_hold').value))
        self.healthy = lh.Persistence(
            float(self.get_parameter('healthy_hold').value))

        self.field = None
        self.scan = None
        self.amcl = None            # (x, y, yaw, sigma_xy, stamp)
        # (x, y, yaw, when) — the last pose AMCL published while this
        # monitor was calling it healthy. Published in the status line so
        # a recovery has a verified place to re-seed the filter from.
        self.fix = None

        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        scan_topic = self.get_parameter('scan_topic').value
        # coco_config is the source of truth and says /scan is
        # BEST_EFFORT. It is consulted rather than assumed — but the
        # fallback is BEST_EFFORT too, and that asymmetry is deliberate.
        # `is_best_effort` returns False for a topic it does not know, so
        # deriving RELIABLE from it would make a renamed scan topic
        # silently blind — the exact trap CLAUDE.md's table names. A
        # BEST_EFFORT subscriber matches a BEST_EFFORT *or* a RELIABLE
        # publisher, so it is the safe direction to be wrong in.
        reliable_scan = (scan_topic in SENSOR_TOPICS
                         and not is_best_effort(scan_topic))
        sensor = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=(QoSReliabilityPolicy.RELIABLE if reliable_scan
                         else QoSReliabilityPolicy.BEST_EFFORT),
            durability=QoSDurabilityPolicy.VOLATILE)

        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, latched)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, sensor)

        self.status = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        rate = float(self.get_parameter('rate').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'localization_monitor: lik_mean_d_max='
            f'{self.thresholds.lik_mean_d_max:.3f} '
            f'degraded_hold={self.degraded.hold:.1f}s '
            f'healthy_hold={self.healthy.hold:.1f}s. '
            f'Publishes only; drives nothing.')

    # ── inputs ───────────────────────────────────────────────────────────
    def _on_map(self, msg):
        if self.field is not None:
            return
        self.field = lh.LikelihoodField.from_occupancy_grid(
            msg.data, msg.info.width, msg.info.height,
            msg.info.resolution,
            (msg.info.origin.position.x, msg.info.origin.position.y))
        self.get_logger().info(
            f'likelihood field: {self.field.w}x{self.field.h} cells, '
            f'{self.field.n_occupied} occupied, res {self.field.res}')

    def _on_scan(self, msg):
        self.scan = msg

    def _on_amcl(self, msg):
        cov = msg.pose.covariance
        self.amcl = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     yaw_of(msg.pose.pose.orientation),
                     math.sqrt(max(0.0, cov[0]) + max(0.0, cov[7])),
                     self._stamp(msg.header.stamp))

    # ── clock ────────────────────────────────────────────────────────────
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    # ── the tick ─────────────────────────────────────────────────────────
    def _observe(self):
        """Fill an Observation from deployable inputs only."""
        now = self.now()
        amcl_age = None
        sigma = None
        world_x = world_y = None
        if self.amcl is not None:
            x, y, _yaw, sigma, stamp = self.amcl
            amcl_age = now - stamp
            world_x = x + MAP_TO_WORLD_X
            # The map frame is only translated in x, so a map y IS a
            # world y. Named rather than left implicit, because the whole
            # frame confusion C2-M5.0 warns about is an x-offset that
            # nobody applied.
            world_y = y

        map_odom_age = None
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time())
            map_odom_age = now - self._stamp(tf.header.stamp)
        except Exception:
            pass

        mean_d = frac_near = None
        beams = 0
        if self.field is not None and self.scan is not None:
            scan = self.scan
            frame = scan.header.frame_id or 'base_footprint'
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', frame, rclpy.time.Time())
                mean_d, frac_near, beams = lh.score_scan(
                    self.field, scan.ranges, scan.angle_min,
                    scan.angle_increment, scan.range_min, scan.range_max,
                    tf.transform.translation.x, tf.transform.translation.y,
                    yaw_of(tf.transform.rotation))
            except Exception:
                pass

        return lh.Observation(
            lik_mean_d=mean_d, lik_frac_near=frac_near, lik_beams=beams,
            cov_sigma_xy=sigma, amcl_age=amcl_age,
            map_odom_age=map_odom_age,
            on_mapped_ground=lh.on_mapped_ground(world_x, world_y))

    def _tick(self):
        now = self.now()
        obs = self._observe()
        verdict = lh.classify(obs, self.thresholds)

        # Only INCONSISTENT and STALE count as evidence of a fault.
        # UNKNOWN is not bad news and must not become bad news by
        # persisting: off the mapped ground the metric is meaningless,
        # and a third of the mission happens there.
        bad = verdict.verdict in (lh.INCONSISTENT, lh.STALE)
        degraded = self.degraded.update(now, bad)
        healthy = self.healthy.update(now, verdict.verdict == lh.CONSISTENT)

        # The two latches are MUTUALLY EXCLUSIVE, and degraded wins.
        #
        # Experiment 2 measured why. The windows are deliberately
        # asymmetric — 2.0 s to declare a degradation, 3.0 s to declare
        # health — so on a robot that has been healthy for a whole
        # mission, `degraded` latches a full second before `healthy`
        # finishes draining. For that second both flags read 1. The
        # executive enters RELOCALIZE, immediately sees healthy=1 left
        # over from BEFORE the fault, and resumes 0.1 s later without the
        # spin having turned the robot at all. Measured: RELOCALIZE
        # entered at t+0.0 and exited at t+0.1, then failed again.
        #
        # Clearing `healthy` here means the resume gate can only be
        # satisfied by CONSISTENT samples earned after the degradation
        # was declared — which is what "post-recovery health is verified"
        # has to mean for the verification to be worth anything.
        if degraded:
            self.healthy.reset()
            healthy = False

        # The last pose AMCL published while the monitor was calling it
        # healthy, kept so a recovery has somewhere to re-seed from.
        #
        # Experiment 2 measured why this is worth keeping. Recovering by
        # `/reinitialize_global_localization` spreads the particles over
        # the whole map, and on this map — a largely rectangular room
        # whose 2D slice is highly self-similar — a 360 degree scan from
        # one standing position does not disambiguate it. AMCL converged
        # to world (2.60, -0.64), which is inside the wedge footprint;
        # the planner then reported "Start occupied" and "no valid path
        # found", so a pose the health monitor was willing to call
        # consistent was one Nav2 could not plan from at all.
        #
        # This is a fix the robot measured for itself and has already
        # verified: it is where AMCL said it was while the scan agreed
        # with the map. It is NOT ground truth, and it is stale by
        # design — the point is that it is stale by only a few seconds.
        if healthy and self.amcl is not None:
            x, y, yaw, _sigma, _stamp = self.amcl
            self.fix = (x, y, yaw, now)

        held = (self.degraded.held_for(now) if degraded or bad
                else self.healthy.held_for(now))

        msg = String()
        msg.data = format_status(verdict, degraded, healthy, held, obs,
                                 self.fix, now)
        self.status.publish(msg)


def main():
    rclpy.init()
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGTERM from a launch teardown arrives as ExternalShutdown, and
        # letting it propagate prints a 15-line rclpy traceback over the
        # end of every run. ros_clean's sweep is a normal way for this
        # node to die, not a fault worth a stack trace.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
