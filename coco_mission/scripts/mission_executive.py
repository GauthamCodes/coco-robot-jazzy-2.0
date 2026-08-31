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
mission_executive — the ROS face of the fetch mission's state machine.

All of the mission's logic lives in ``mission_states``, which is pure
Python and knows nothing about ROS. This file is the adapter: it turns
subscriptions into an :class:`~mission_states.Observation`, hands that to
:class:`~mission_states.MissionMachine`, and performs the single request
the machine asks for. It is deliberately thin, and everything in it is
plumbing — if you are looking for why a state advances, it is not here.

That split is not a style preference. C2-M2.1 shipped a thoroughly
tested pure estimator behind an untested ROS adapter and three defects
lived in the gap, one of which stopped the node constructing at all. So
this node has its own tests that construct it for real.

What it replaces
----------------
``gazebo_models/scripts/traverse_demo.py``, which runs the same mission
as a blocking script. That script is kept, unchanged, because it is the
harness the M4/M5/M6 numbers were measured with and those measurements
have to stay reproducible. **Do not run both at once**: they would both
publish ``/mission/mode`` and ``/mission/state``, and the arbiter latches
the last mode it saw. ``mission.launch.py`` starts this node;
``executive:=false`` turns it off for a traverse_demo run.

Interfaces
----------
in   /model/coco/odometry     nav_msgs/Odometry            world pose
in   /amcl_pose               geometry_msgs/PoseWithCovarianceStamped
in   /ramp/status             std_msgs/String
in   /approach/status         std_msgs/String
in   /grasp/status            std_msgs/String
in   /perception/status       std_msgs/String
in   /cmd_vel_arbiter/status  std_msgs/String
in   /mission/target_colour   std_msgs/String
out  /mission/mode            std_msgs/String   (2 Hz, and on change)
out  /mission/state           std_msgs/String   (2 Hz, and on change)
out  /mission/target_colour   std_msgs/String   only when the colour came
     from a parameter or the CLI, exactly as traverse_demo does — with
     the panel up, the panel is the publisher and this stays silent.
srv  /mission/start           std_srvs/Trigger
srv  /mission/abort           std_srvs/Trigger
act  navigate_to_pose         nav2_msgs/NavigateToPose
cli  /ramp/climb /ramp/descend /ramp/stop /approach/run /approach/stop
     /grasp/stow /grasp/pick /grasp/place   (all std_srvs/Trigger)

**It publishes no velocity.** ``cmd_vel_arbiter`` remains the sole
publisher to ``/diff_drive_controller/cmd_vel``; this node only selects
which source the arbiter forwards. A test asserts the package contains
no publisher to the controller topic.

Two clocks, on purpose
----------------------
State timeouts are measured on the **node clock**, which is simulation
time under ``use_sim_time``. That is the right unit for "how long did the
robot take", and it is what makes a timeout mean the same thing in a run
that renders and one that does not.

The tick timer itself runs on a **steady clock**. Under ``use_sim_time``
a node-clock timer stops firing the moment ``/clock`` stops — which is
exactly the situation (a dead simulator, an orphaned bridge publishing a
stale clock) where the executive most needs to notice. The machine's
``CLOCK_STALLED`` check compares the two and aborts if simulation time
stands still while wall time does not. cmd_vel_arbiter's watchdog runs on
a steady clock for the same reason.

Usage
-----
  ros2 run coco_mission mission_executive.py --colour blue --autostart
  ros2 service call /mission/start std_srvs/srv/Trigger   # if not autostart
  ros2 service call /mission/abort std_srvs/srv/Trigger   # operator stop
  ros2 topic echo /mission/state
"""

import argparse
import math
import sys
import time

from action_msgs.msg import GoalStatus

from coco_config.robot import TARGET_COLOURS

from geometry_msgs.msg import PoseWithCovarianceStamped

import mission_states as ms

from nav2_msgs.action import NavigateToPose, Spin

from nav_msgs.msg import Odometry

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger

# How often the executive looks at the world and decides. 10 Hz is the
# rate the status topics it reads are published at or above (5 Hz for the
# three servers), so it never decides on the same sample twice by
# accident and never waits a noticeable time to notice a change.
TICK_HZ = 10.0

# How often /mission/mode and /mission/state are re-asserted. The mode
# MUST be re-asserted: if it lapses, the arbiter keeps its last value,
# and a mode set once at a step boundary is a mode nothing refreshes if
# the executive dies. 2 Hz matches traverse_demo and the web panel.
PUBLISH_HZ = 2.0

# How long a service or action server may be missing before the request
# fails as unavailable. traverse_demo used 20 s for the same wait.
SERVICE_WAIT = 20.0

# C2-M5.1, for the recovery re-seed. Both from nav2_params.yaml rather
# than chosen here: the robot cannot have travelled further than
# `max_vel_x` for as long as the last verified fix has been stale, and
# below the controller's own `xy_goal_tolerance` there is no point being
# more precise than the stack's idea of having arrived.
NAV_MAX_VEL_X = 0.3
NAV_XY_GOAL_TOLERANCE = 0.25
# The heading is re-seeded too, and its spread is amcl.update_min_a —
# the rotation AMCL itself treats as the smallest worth a filter update.
AMCL_SEED_YAW_SIGMA = 0.2

# The status topics the machine reads, and the key each is stored under.
STATUS_TOPICS = (
    ('ramp', '/ramp/status'),
    ('approach', '/approach/status'),
    ('grasp', '/grasp/status'),
    ('perception', '/perception/status'),
    ('arbiter', '/cmd_vel_arbiter/status'),
    # C2-M5.1. localization_monitor publishes the same key=value shape as
    # every other status topic here, which is why it needed no new
    # parser and no new subscription pattern — only a row in this table.
    ('localization', '/localization/health'),
)

# Everything the executive can ask a subsystem to do, and the services
# RECOVERY/ABORT use to stop them. /grasp has no stop service — an arm
# trajectory in flight is finished by move_group, not interrupted here.
SERVICES = (
    '/ramp/climb', '/ramp/descend', '/ramp/stop',
    '/approach/run', '/approach/stop',
    '/grasp/stow', '/grasp/pick', '/grasp/place',
)
STOP_SERVICES = ('/ramp/stop', '/approach/stop')


def _finite(value):
    """A float, or None if it is NaN — the "no threshold" spelling."""
    value = float(value)
    return None if math.isnan(value) else value


def yaw_of(orientation):
    """Yaw from a geometry_msgs/Quaternion, in radians."""
    x, y, z, w = (orientation.x, orientation.y,
                  orientation.z, orientation.w)
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


class MissionExecutive(Node):
    """Drives MissionMachine against the live ROS graph."""

    def __init__(self, colour=None, lane=None, do_grasp=None,
                 autostart=None):
        super().__init__('mission_executive')

        self.declare_parameter('target_colour', '')
        self.declare_parameter('lane', float('nan'))
        self.declare_parameter('do_grasp', True)
        self.declare_parameter('autostart', False)
        self.declare_parameter('service_wait', SERVICE_WAIT)
        self.declare_parameter('stall_limit', 10.0)
        self.declare_parameter('exit_on_finish', False)
        self.declare_parameter('xy_tolerance', ms.GOAL_XY_TOLERANCE)
        # NaN means "do not gate on heading", which is the default. See
        # mission_states.GOAL_YAW_TOLERANCE: the obvious 0.25 rad aborts
        # missions that complete, and no threshold has been measured.
        self.declare_parameter('yaw_tolerance', float('nan'))
        self.declare_parameter('lane_tolerance', ms.LANE_TOLERANCE)
        # C2-M5.1. False leaves /localization/health published and unread:
        # the executive never fails a leg on it and never relocalizes.
        # That is the C2-M5.1 false-positive experiment, and it is also
        # how a pre-C2-M5.1 mission is reproduced exactly.
        self.declare_parameter('localization_recovery', True)

        # The CLI wins over the parameter, because `ros2 run ... --colour
        # blue` is the headless form and the parameter is the launch-file
        # form; a run that passes both meant the one it typed.
        param_colour = str(self.get_parameter('target_colour').value or '')
        self.colour = (colour or param_colour).strip().lower() or None
        # A colour this node was TOLD is one it must also announce: the
        # approach and grasp servers take the choice off the topic and
        # two of the three refuse to start without it. With the panel up
        # the panel is the publisher and this stays quiet, exactly as
        # traverse_demo._assert_colour does.
        self.announce = self.colour is not None

        param_lane = float(self.get_parameter('lane').value)
        if lane is None and not math.isnan(param_lane):
            lane = param_lane
        if do_grasp is None:
            do_grasp = bool(self.get_parameter('do_grasp').value)
        if autostart is None:
            autostart = bool(self.get_parameter('autostart').value)

        self.service_wait = float(self.get_parameter('service_wait').value)
        self.exit_on_finish = bool(
            self.get_parameter('exit_on_finish').value)

        self.plan = ms.MissionPlan(
            self.colour, lane=lane, do_grasp=do_grasp,
            xy_tolerance=float(self.get_parameter('xy_tolerance').value),
            yaw_tolerance=_finite(self.get_parameter('yaw_tolerance').value),
            lane_tolerance=float(
                self.get_parameter('lane_tolerance').value),
            localization_recovery=bool(
                self.get_parameter('localization_recovery').value))
        self.machine = ms.MissionMachine(
            self.plan,
            stall_limit=float(self.get_parameter('stall_limit').value))

        self.started = bool(autostart)
        self.abort_requested = False
        self.finished_at = None

        # ── inputs ───────────────────────────────────────────────────────
        self._pose = None
        self._pose_stamp = None
        self._localized = False
        self._views = {key: ms.WorkerView() for key, _ in STATUS_TOPICS}

        self.create_subscription(
            Odometry, '/model/coco/odometry', self._on_odom, 10)
        for key, topic in STATUS_TOPICS:
            self.create_subscription(
                String, topic,
                (lambda msg, k=key: self._on_status(k, msg)), 10)
        self.create_subscription(
            String, '/mission/target_colour', self._on_colour, 10)
        # AMCL latches its pose TRANSIENT_LOCAL and, with update_min_a
        # 0.2 rad, a stationary robot produces no new one. A VOLATILE
        # subscriber started after AMCL would therefore never see a pose
        # at all and LOCALIZE would time out on a healthy stack.
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, latched)

        # ── outputs ──────────────────────────────────────────────────────
        self._mode_pub = self.create_publisher(String, '/mission/mode', 10)
        self._state_pub = self.create_publisher(String, '/mission/state', 10)
        self._colour_pub = self.create_publisher(
            String, '/mission/target_colour', 10)

        # ── the subsystems ───────────────────────────────────────────────
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # C2-M5.1. nav2's behavior_server already serves this and already
        # publishes to /cmd_vel_nav; sending it a goal adds a CLIENT, not
        # a publisher. Verified on the live graph in C2-M5.0's
        # c2m5_topology.txt, which lists behavior_server among
        # /cmd_vel_nav's publishers before any of this existed.
        self._spin = ActionClient(self, Spin, 'spin')
        # AMCL's own global-relocalization service — std_srvs/Empty, so it
        # cannot go in `service_clients`, which is a Trigger table. Kept
        # separate rather than generalising that table for one caller.
        self._relocalize = self.create_client(
            Empty, '/reinitialize_global_localization')
        # The same topic RViz's "2D Pose Estimate" writes. Publishing a
        # POSE is not publishing a velocity: the arbiter invariant is
        # about /diff_drive_controller/cmd_vel and this touches nothing
        # on that path.
        self._initialpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        # NOT `self._clients`: rclpy's Node keeps its own list of service
        # clients under exactly that name, and shadowing it makes the
        # executor iterate this dict's KEYS instead of the clients. The
        # symptom is an AttributeError from deep inside rclpy's wait set
        # on the first spin, with nothing in this file in the traceback.
        self.service_clients = {
            name: self.create_client(Trigger, name) for name in SERVICES}

        self.create_service(Trigger, '/mission/start', self._on_start)
        self.create_service(Trigger, '/mission/abort', self._on_abort)

        # ── request bookkeeping ──────────────────────────────────────────
        self._token = None
        self._status = None
        self._request = None
        self._deadline = None
        self._sent = False
        self._goal_handle = None
        self._stop_pending = 0
        # Every side effect this node has performed, in order. Kept for
        # the log and for the tests: "what did the executive actually
        # ask the robot to do" should not require reading a rosbag.
        self._issued = []

        self._mode = 'idle'
        self._last_published = None

        # The tick runs on a STEADY clock — see the module docstring. A
        # node-clock timer under use_sim_time stops with /clock, and a
        # stopped executive cannot report that the clock stopped.
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / TICK_HZ, self._tick, clock=self._steady)
        self.create_timer(1.0 / PUBLISH_HZ, self._assert_outputs,
                          clock=self._steady)

        self.get_logger().info(
            f'mission_executive up: colour={self.colour or "--"} '
            f'lane={self.plan.lane:+.2f} '
            f'grasp={"yes" if do_grasp else "no (traverse only)"} '
            f'autostart={self.started}')
        if not self.started:
            self.get_logger().info(
                'waiting for /mission/start — '
                'ros2 service call /mission/start std_srvs/srv/Trigger')

    # ── clocks ───────────────────────────────────────────────────────────
    def now(self):
        """Mission time in seconds: the node clock, so sim time in sim."""
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def wall():
        """Steady wall seconds, never affected by /clock."""
        return time.monotonic()

    # ── subscriptions ────────────────────────────────────────────────────
    def _on_odom(self, msg):
        pose = msg.pose.pose
        self._pose = (pose.position.x, pose.position.y,
                      yaw_of(pose.orientation))
        self._pose_stamp = self.now()

    def _on_status(self, key, msg):
        self._views[key] = ms.WorkerView.from_line(msg.data, self.now())

    def _on_amcl(self, msg):
        # Presence only. Whether the estimate is any GOOD is C2-M5's
        # question and its threshold has never been calibrated; asserting
        # one here would be inventing a number.
        self._localized = True

    def _on_colour(self, msg):
        colour = (msg.data or '').strip().lower()
        if colour not in TARGET_COLOURS or colour == self.colour:
            return
        if self.machine.state != ms.IDLE:
            self.get_logger().warn(
                f'ignoring a colour change to {colour} mid-mission '
                f'(state {self.machine.state}); the lane is already set')
            return
        self.colour = colour
        self.plan = ms.MissionPlan(
            colour, do_grasp=self.plan.do_grasp,
            xy_tolerance=self.plan.xy_tolerance,
            yaw_tolerance=self.plan.yaw_tolerance,
            lane_tolerance=self.plan.lane_tolerance)
        self.machine.plan = self.plan
        self.get_logger().info(
            f'target colour {colour}, lane {self.plan.lane:+.2f}')

    # ── operator services ────────────────────────────────────────────────
    def _on_start(self, request, response):
        if self.colour is None:
            response.success = False
            response.message = (
                'no target colour. Pick one on the panel, publish '
                '/mission/target_colour, or pass --colour.')
            return response
        if self.machine.state in ms.TERMINAL_STATES:
            response.success = False
            response.message = (
                f'mission already finished in {self.machine.state}; '
                f'restart the node for another run')
            return response
        self.started = True
        response.success = True
        response.message = f'starting the fetch for {self.colour}'
        self.get_logger().info(response.message)
        return response

    def _on_abort(self, request, response):
        self.abort_requested = True
        response.success = True
        response.message = 'aborting; stopping every driver'
        self.get_logger().warn(response.message)
        return response

    # ── the loop ─────────────────────────────────────────────────────────
    def observe(self):
        """Snapshot the world for the machine. Pure read, no side effects."""
        return ms.Observation(
            ros_now=self.now(),
            wall_now=self.wall(),
            started=self.started,
            abort_requested=self.abort_requested,
            colour=self.colour,
            pose=self._pose,
            pose_stamp=self._pose_stamp,
            localized=self._localized,
            ramp=self._views['ramp'],
            approach=self._views['approach'],
            grasp=self._views['grasp'],
            perception=self._views['perception'],
            arbiter=self._views['arbiter'],
            localization=self._views['localization'],
            request_token=self._token,
            request_status=self._status)

    def _tick(self):
        self._pump()
        directive = self.machine.update(self.observe())
        self._mode = directive.mode

        for event in directive.events:
            self._log_event(event)

        if (directive.request is not None
                and directive.request.token != self._token):
            self._begin(directive.request)

        if directive.events:
            self._assert_outputs(event='enter')
        if directive.terminal and self.finished_at is None:
            self.finished_at = self.wall()
            self._report()

    def _log_event(self, event):
        if (event.previous == ms.ALIGN_FOR_CLIMB
                and self.machine.align_yaw is not None):
            # Reported because it is not gated. It is the number a
            # calibrated heading check would need, and the only place it
            # is measured on a real run.
            gate = self.plan.yaw_tolerance
            self.get_logger().info(
                f'pre-climb heading {self.machine.align_yaw:+.3f} rad '
                f'({math.degrees(self.machine.align_yaw):+.1f} deg); '
                f'gate {"off" if gate is None else f"{gate:.2f} rad"}')

        line = (f'{event.previous} -> {event.state}'
                f'{"" if not event.reason else f" [{event.reason}]"}'
                f'{"" if not event.detail else f": {event.detail}"}')
        if event.state in (ms.RECOVERY, ms.ABORT):
            self.get_logger().error(line)
        else:
            self.get_logger().info(line)

    def _report(self):
        machine = self.machine
        self.get_logger().info(
            f'MISSION {machine.state}: result={machine.result or "--"} '
            f'reason={machine.reason or "--"} '
            f'attempts={machine.attempts or "{}"}')

    # ── requests ─────────────────────────────────────────────────────────
    def _begin(self, request):
        """Take ownership of a new request and try to send it."""
        # A goal from the state we just left must not be allowed to steer
        # the robot into the next one. STOP_ALL cancels it properly; this
        # covers the ordinary case of moving on.
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._token = request.token
        self._status = ms.PENDING
        self._deadline = self.wall() + self.service_wait
        self._sent = False
        self._request = request
        self._issued.append((request.kind, request.payload))
        self._pump()

    def requests_issued(self):
        """(kind, payload) for every request performed, in order."""
        return list(self._issued)

    def _pump(self):
        """Send, or keep waiting for, the outstanding request."""
        if (self._token is None or self._request is None or self._sent
                or self._status != ms.PENDING):
            return
        request = self._request
        if request.kind == ms.NAV_GOAL:
            ready = self._nav.server_is_ready()
        elif request.kind == ms.RELOCALIZE_GOAL:
            ready = self._spin.server_is_ready()
        elif request.kind == ms.CALL_SERVICE:
            ready = self.service_clients[request.payload].service_is_ready()
        else:
            ready = True                       # STOP_ALL sends what it can

        if not ready:
            if self.wall() > self._deadline:
                self._status = ms.UNAVAILABLE
                self.get_logger().error(
                    f'{request.payload or request.kind} did not appear in '
                    f'{self.service_wait:.0f}s')
            return

        self._sent = True
        if request.kind == ms.NAV_GOAL:
            self._send_nav(request)
        elif request.kind == ms.RELOCALIZE_GOAL:
            self._send_spin(request)
        elif request.kind == ms.CALL_SERVICE:
            self._call(request, request.payload)
        else:
            self._stop_all(request)

    def _finish(self, token, status):
        """Record a reply, unless the state that asked has moved on."""
        if token != self._token:
            return
        self._status = status

    def _send_nav(self, request):
        wx, wy = request.payload
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = wx + ms.WORLD_TO_MAP_X
        goal.pose.pose.position.y = wy
        goal.pose.pose.orientation.w = 1.0
        token = request.token
        self.get_logger().info(
            f'NavigateToPose -> world ({wx:.2f}, {wy:.2f})')

        def on_sent(future):
            handle = future.result()
            if handle is None or not handle.accepted:
                self._finish(token, ms.REJECTED)
                return
            if token != self._token:
                handle.cancel_goal_async()
                return
            self._goal_handle = handle
            self._finish(token, ms.ACCEPTED)
            handle.get_result_async().add_done_callback(
                lambda result: on_result(result))

        def on_result(future):
            if token != self._token:
                return
            self._goal_handle = None
            status = future.result().status
            self._finish(token, {
                GoalStatus.STATUS_SUCCEEDED: ms.SUCCEEDED,
                GoalStatus.STATUS_CANCELED: ms.CANCELED,
            }.get(status, ms.ABORTED))

        self._nav.send_goal_async(goal).add_done_callback(on_sent)

    def _reseed_amcl(self):
        """Give AMCL somewhere to put its particles before the spin.

        **Preferred: the last fix the health monitor verified.** The
        monitor publishes ``fix_x/fix_y/fix_yaw/fix_age`` — the pose AMCL
        was reporting while the scan still agreed with the map. Seeding
        there is not cheating and is not ground truth: it is the robot's
        own estimate from a few seconds ago, at a moment something
        independent had checked it.

        **Fallback: /reinitialize_global_localization**, AMCL's uniform
        reset. It is the fallback and not the default because Experiment
        2 measured what it does on this map: the particles spread over a
        largely rectangular room whose 2D slice is highly self-similar, a
        360 degree scan from a standing robot does not disambiguate it,
        and AMCL converged to world (2.60, -0.64) — inside the wedge
        footprint. The health monitor was satisfied and the planner was
        not: "Start occupied", then "no valid path found" from (4.60,
        -0.64) to (0.00, 0.00), and the mission aborted with a pose it
        had just declared healthy.

        The seed spread is derived, not chosen. The robot cannot have
        travelled further than ``max_vel_x`` for as long as the fix has
        been stale, and below Nav2's own ``xy_goal_tolerance`` there is
        no point being more precise than the stack's idea of "arrived".
        Both numbers come from ``nav2_params.yaml``.
        """
        fields = self.machine and self._views['localization'].fields or {}

        def value(key):
            raw = fields.get(key, '--')
            try:
                return float(raw)
            except (TypeError, ValueError):
                return None

        fx, fy, fyaw, age = (value('fix_x'), value('fix_y'),
                             value('fix_yaw'), value('fix_age'))
        if None not in (fx, fy, fyaw, age):
            sigma = max(NAV_XY_GOAL_TOLERANCE, NAV_MAX_VEL_X * age)
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = fx
            msg.pose.pose.position.y = fy
            msg.pose.pose.orientation.z = math.sin(fyaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(fyaw / 2.0)
            cov = [0.0] * 36
            cov[0] = cov[7] = sigma ** 2
            cov[35] = AMCL_SEED_YAW_SIGMA ** 2
            msg.pose.covariance = cov
            self._initialpose.publish(msg)
            self.get_logger().warn(
                f'localization recovery: re-seeding AMCL at the last '
                f'verified fix ({fx:.2f}, {fy:.2f}), {age:.1f} s old, '
                f'sigma {sigma:.2f} m')
            return

        if self._relocalize.service_is_ready():
            self._relocalize.call_async(Empty.Request())
            self.get_logger().error(
                'localization recovery: no verified fix to re-seed from, '
                'falling back to global relocalization — C2-M5.1 measured '
                'this converging to an unplannable pose on this map')
            return

        self.get_logger().error(
            'localization recovery: no verified fix and no global '
            'relocalization service; spinning without resetting the '
            'filter, which does not recover a confidently-wrong pose')

    def _send_spin(self, request):
        """The localization recovery: reset the filter, then re-observe.

        Two existing interfaces, in this order, and the order is the
        whole point:

        1. ``/reinitialize_global_localization`` — AMCL's own service,
           the one RViz's "Global Localization" button calls. It spreads
           the particles over the map's free space.
        2. ``nav2_msgs/action/Spin`` — served by ``behavior_server``,
           which is already a publisher on ``/cmd_vel_nav``.

        **Experiment 2 measured why step 1 has to be there.** The spin on
        its own recovered nothing: ``nav2_params.yaml`` sets
        ``recovery_alpha_fast`` and ``recovery_alpha_slow`` to 0.0, so
        AMCL's augmented-MCL random-particle injection is off and the
        filter cannot leave a mode it is confident in — which is exactly
        the class-A failure being injected. The spin ran a full 9.1 s,
        the scan agreed for long enough to satisfy the resume gate, and
        disagreed again 6.0 s after the mission resumed. Turning gives
        the filter new data; only the reset gives it somewhere else to
        put its particles.

        Deliberately the same shape as ``_send_nav``: the goal handle is
        stored in the same slot, so ``_begin``'s cancel-on-move-on and
        ``_stop_all``'s cancel both already cover it. A recovery motion
        that could outlive the state that asked for it would be a new way
        to leave the robot driving, and reusing the existing handle is
        what stops that being possible.

        Neither step publishes a velocity. The reset is a service call;
        the turn comes out of a node that was already on the wheel path.
        """
        self._reseed_amcl()
        goal = Spin.Goal()
        goal.target_yaw = float(request.payload)
        token = request.token
        self.get_logger().warn(
            f'localization recovery: spinning {goal.target_yaw:+.2f} rad '
            f'to re-observe')

        def on_sent(future):
            handle = future.result()
            if handle is None or not handle.accepted:
                self._finish(token, ms.REJECTED)
                return
            if token != self._token:
                handle.cancel_goal_async()
                return
            self._goal_handle = handle
            self._finish(token, ms.ACCEPTED)
            handle.get_result_async().add_done_callback(on_result)

        def on_result(future):
            if token != self._token:
                return
            self._goal_handle = None
            status = future.result().status
            self._finish(token, {
                GoalStatus.STATUS_SUCCEEDED: ms.SUCCEEDED,
                GoalStatus.STATUS_CANCELED: ms.CANCELED,
            }.get(status, ms.ABORTED))

        self._spin.send_goal_async(goal).add_done_callback(on_sent)

    def _call(self, request, name):
        token = request.token
        self.get_logger().info(f'calling {name}')

        def on_reply(future):
            reply = future.result()
            self._finish(token,
                         ms.ACCEPTED if reply is not None and reply.success
                         else ms.REJECTED)
            if reply is not None and not reply.success:
                self.get_logger().error(f'{name} refused: {reply.message}')

        self.service_clients[name].call_async(
            Trigger.Request()).add_done_callback(on_reply)

    def _stop_all(self, request):
        """Cancel the nav goal and tell every driver to stop.

        ``/ramp/stop`` had no caller anywhere in the tree before
        traverse_demo added one, and the reason is worth repeating: a
        failed step that just stops asserting a mode leaves the last
        twist to age out against the arbiter's watchdog rather than being
        cancelled. On the platform that is the difference between
        stopping and coasting off a 0.65 m edge.
        """
        token = request.token
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        pending = [name for name in STOP_SERVICES
                   if self.service_clients[name].service_is_ready()]
        self._stop_pending = len(pending)
        if not pending:
            self._finish(token, ms.SUCCEEDED)
            return

        def on_reply(future):
            self._stop_pending -= 1
            if self._stop_pending <= 0:
                self._finish(token, ms.SUCCEEDED)

        for name in pending:
            self.service_clients[name].call_async(
                Trigger.Request()).add_done_callback(on_reply)

    # ── outputs ──────────────────────────────────────────────────────────
    def _assert_outputs(self, event='run'):
        """Re-assert the mode, the state and (if ours) the colour.

        The mode is re-asserted rather than published once at a step
        boundary because the arbiter latches it: a mode published once is
        a mode nothing refreshes if this node dies mid-leg, and the
        robot would keep driving on the last one it saw.
        """
        self._mode_pub.publish(String(data=self._mode))
        line = self.machine.status_line(self.now(), event=event)
        self._state_pub.publish(String(data=line))
        self._last_published = line
        if self.announce and self.colour:
            self._colour_pub.publish(String(data=self.colour))

    # ── shutdown ─────────────────────────────────────────────────────────
    def should_exit(self):
        """True once a finished mission has had time to be seen."""
        if not self.exit_on_finish or self.finished_at is None:
            return False
        return (self.wall() - self.finished_at) > 1.0

    def exit_code(self):
        """0 only for a mission that completed what it was asked to do."""
        return 0 if self.machine.state == ms.COMPLETE else 1


def parse_args(argv):
    """CLI, matching traverse_demo's so the two are interchangeable."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--colour', choices=TARGET_COLOURS, default=None,
                        help='which target to fetch; picks its lane. Omit '
                             'to wait for /mission/target_colour.')
    parser.add_argument('--lane', type=float, default=None,
                        help='override the lane with a bare world y')
    parser.add_argument('--no-grasp', action='store_true',
                        help='traverse only — reproduces the M4/M5 run')
    parser.add_argument('--autostart', action='store_true',
                        help='start as soon as the inputs are there, '
                             'instead of waiting for /mission/start')
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    """Run the executive until the mission finishes or the node is killed."""
    argv = sys.argv[1:] if argv is None else argv
    args = parse_args(argv)
    rclpy.init(args=sys.argv)
    node = MissionExecutive(
        colour=args.colour,
        lane=args.lane,
        do_grasp=False if args.no_grasp else None,
        autostart=True if args.autostart else None)
    rc = 1
    try:
        while rclpy.ok() and not node.should_exit():
            rclpy.spin_once(node, timeout_sec=0.1)
        rc = node.exit_code()
    except (KeyboardInterrupt, ExternalShutdownException):
        rc = node.exit_code()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
