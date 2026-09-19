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
platform_server — the COCO web platform's single ROS 2 node.

It serves the browser UI and speaks the versioned WebSocket protocol in
``protocol.py``.

    ros2 run coco_web platform_server
    ros2 launch coco_web platform.launch.py     # with the arbiter

Then open http://localhost:8080.

What this replaces, and why
---------------------------
The previous panel was ``rosbridge_websocket`` plus a static HTML file.
rosbridge is a *generic* ROS-over-WebSocket bridge: any browser tab could
publish any topic, call any service, and enumerate the graph via rosapi.
Nothing but the HTML's good manners stopped a tab from publishing
``/diff_drive_controller/cmd_vel`` and becoming a second wheel publisher,
which is the exact failure C2-NAV.41 measured reaching the wheels 21.31 %
of the time.

This server is the opposite shape. The browser names *intents*, never
topics; the intent-to-topic mapping is a fixed allowlist checked against
``safety.WHEEL_TOPICS`` at construction. It is the same door the terminal
uses -- ``/mission/start`` is the same ``std_srvs/Trigger`` -- with a
button on it.

Threading
---------
rclpy spins in its own thread on a SingleThreadedExecutor; tornado owns
the main thread. The two meet at exactly two places, both guarded by
``self._lock``: the node writes its latest observations into a snapshot
dict, and the tornado PeriodicCallback reads it to build telemetry. ROS
publishing from the tornado thread is safe (rclpy publishers hold the
GIL-protected middleware handle), and every publish goes through
``_publish_*`` helpers so there is one place to audit.

Camera and depth
----------------
Images do NOT go through this WebSocket. P0.1 keeps ``web_video_server``'s
MJPEG endpoint and sends the browser its *metadata* -- URL, topic,
encoding -- so the ``<img>`` tag streams binary straight from a server
built for it. Forcing JPEG through JSON as base64 would inflate every
frame by a third and put image data on the same socket as the STOP
button. Depth is off by default and advertised only when asked for; a
binary WebSocket transport and WebRTC are P0.2/P0.3 decisions, recorded
in docs/ROADMAP.md.
"""

import json
import os
import threading
import time

from coco_web import protocol, safety
from coco_web import session as session_mod
from coco_web import telemetry as tele

from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                               TwistStamped)

from nav_msgs.msg import Odometry, Path

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from sensor_msgs.msg import LaserScan

from std_msgs.msg import String

from std_srvs.srv import Trigger

import tornado.ioloop
import tornado.web
import tornado.websocket

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#: How often telemetry is pushed to every connected client. 10 Hz is the
#: rate the old panel's readouts effectively ran at and is comfortably
#: below the 20 Hz the arbiter publishes its status at.
TELEMETRY_HZ = 10.0

#: A component is considered down if nothing has been heard from it for
#: this long. Three seconds is the old panel's "offline" timeout, kept so
#: the new UI does not report a different truth from the one operators
#: are used to.
STALE_AFTER_S = 3.0

#: The joystick's watchdog. The browser sends `drive` at ~10 Hz while the
#: stick is held; if frames stop arriving -- a dropped socket, a closed
#: laptop lid -- the server publishes one zero and stops. The arbiter's
#: own 0.3 s source timeout is the backstop, not the plan.
DRIVE_TIMEOUT_S = 0.5


def _sensor_qos(depth=5):
    """
    BEST_EFFORT QoS for sensor streams.

    Camera and LiDAR topics are BEST_EFFORT. A RELIABLE subscriber never
    matches one, receives nothing, and raises nothing -- the node goes
    silently blind. That trap has been paid for in this repo already.
    """
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth)


def _latched_qos():
    """TRANSIENT_LOCAL QoS, for /map and other latched one-shot topics."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1)


class CocoWebNode(Node):
    """The ROS half of the platform: observes telemetry, publishes intents."""

    def __init__(self):
        """Create publishers and subscriptions, refusing unsafe topics."""
        super().__init__('coco_web_platform')

        self.declare_parameter('http_port', 8080)
        self.declare_parameter('video_port', 8081)
        self.declare_parameter('bind', '0.0.0.0')
        self.declare_parameter('web_root', '')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('annotated_topic', '/perception/annotated')
        self.declare_parameter('depth_topic', '')
        self.declare_parameter('teleop_topic', safety.TELEOP_TOPIC)

        self.http_port = int(self.get_parameter('http_port').value)
        self.video_port = int(self.get_parameter('video_port').value)
        self.bind = str(self.get_parameter('bind').value)

        # ── the safety boundary ────────────────────────────────────────
        # Every topic this node is about to publish, including the one that
        # came from a parameter. A launch file that points the web layer at
        # the wheel topic fails here, loudly, at startup -- not at 20 Hz
        # with the arbiter quietly warning into a log nobody is reading.
        teleop_topic = str(self.get_parameter('teleop_topic').value)
        publish_topics = [topic for topic, _ in
                          safety.PUBLISH_ALLOWLIST.values()]
        publish_topics.append(teleop_topic)
        safety.assert_publish_safe(publish_topics)
        self._teleop_topic = teleop_topic

        self._lock = threading.Lock()
        self._snap = {
            'pose': None, 'velocity': None, 'scan': None, 'path': None,
            'arbiter': '', 'arbiter_t': 0.0,
            'mission': '', 'mission_t': 0.0,
            'perception': '', 'perception_t': 0.0,
            'odom_t': 0.0, 'clock_t': 0.0, 'nav_t': 0.0, 'map': None,
        }
        self._last_drive = 0.0
        self._drive_zeroed = True

        self.colours = self._load_colours()
        self.arm_limits, self.grip_limits = self._load_joint_limits()

        # ── publishers: exactly the allowlist, nothing else ────────────
        self._pub_drive = self.create_publisher(
            TwistStamped, self._teleop_topic, 10)
        self._pub_mode = self.create_publisher(String, '/mission/mode', 10)
        self._pub_colour = self.create_publisher(
            String, '/mission/target_colour', 10)
        self._pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._pub_arm = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self._pub_grip = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self._srv_start = self.create_client(Trigger, '/mission/start')
        self._srv_abort = self.create_client(Trigger, '/mission/abort')

        # ── subscriptions: telemetry only ──────────────────────────────
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, _sensor_qos())
        self.create_subscription(Path, '/plan', self._on_path, 10)
        self.create_subscription(
            String, '/cmd_vel_arbiter/status', self._on_arbiter, 10)
        self.create_subscription(
            String, '/mission/state', self._on_mission, 10)
        self.create_subscription(
            String, '/perception/status', self._on_perception, 10)

        # The drive watchdog runs on the steady clock: a paused simulator
        # must not freeze the thing that stops the robot.
        self.create_timer(0.1, self._drive_watchdog)

        self.get_logger().info(
            f'COCO platform on http://{self.bind}:{self.http_port} '
            f'(protocol {protocol.PROTOCOL_VERSION}); '
            f'drive -> {self._teleop_topic} (arbiter input)')

    # ── configuration sourced from the robot, not re-typed ─────────────
    def _load_colours(self):
        """
        Target colours, from coco_config if importable.

        coco_config is the mission's own colour table. Falling back to the
        protocol's copy keeps the server runnable in a stripped image, and
        a unit test asserts the two agree so the fallback cannot drift.
        """
        try:
            from coco_config.robot import TARGET_COLOURS
            return tuple(TARGET_COLOURS)
        except ImportError:
            self.get_logger().warn(
                'coco_config unavailable; using the protocol fallback '
                'colour list')
            return tuple(protocol.FALLBACK_COLOURS)

    def _load_joint_limits(self):
        """
        Arm and gripper limits, from coco_config.joint_limits.

        Deliberately NOT re-typed here. The old panel hard-coded
        -3.84..1.0 in its HTML, which is the same drift
        teleop_arm_node's comment records having already happened once:
        the numbers are a property of the URDF, and coco_config's tests
        check them against it.
        """
        try:
            from coco_config.joint_limits import ARM_LIMITS
            arm = {
                'shoulder': tuple(ARM_LIMITS['m_link1_Revolute-6']),
                'elbow': tuple(ARM_LIMITS['m_link2_Revolute-7']),
            }
            grip = tuple(ARM_LIMITS['m_link3_Revolute-8'])
            return arm, grip
        except (ImportError, KeyError):
            self.get_logger().warn(
                'coco_config.joint_limits unavailable; arm control '
                'disabled rather than guessed')
            return None, None

    # ── subscription callbacks ─────────────────────────────────────────
    def _on_odom(self, msg):
        """Record wheel odometry: pose, velocity, and a robot liveness mark."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        twist = msg.twist.twist
        with self._lock:
            self._snap['pose'] = tele.pose_payload(
                (pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w))
            self._snap['velocity'] = tele.velocity_payload(
                twist.linear.x, twist.angular.z)
            self._snap['odom_t'] = time.monotonic()

    def _on_amcl(self, msg):
        """Record the AMCL pose, which supersedes odometry for the map view."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        with self._lock:
            self._snap['pose'] = tele.pose_payload(
                (pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w))
            self._snap['nav_t'] = time.monotonic()

    def _on_scan(self, msg):
        """Downsample the LiDAR once, here, rather than per connected client."""
        payload = tele.downsample_scan(
            list(msg.ranges), msg.angle_min, msg.angle_increment,
            msg.range_max)
        with self._lock:
            self._snap['scan'] = payload

    def _on_path(self, msg):
        """Reduce the current Nav2 plan to a drawable polyline."""
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self._lock:
            self._snap['path'] = tele.path_payload(points)
            self._snap['nav_t'] = time.monotonic()

    def _on_arbiter(self, msg):
        """Keep the arbiter's own status line raw; it is parsed later."""
        with self._lock:
            self._snap['arbiter'] = msg.data
            self._snap['arbiter_t'] = time.monotonic()

    def _on_mission(self, msg):
        """Keep the mission executive's status line."""
        with self._lock:
            self._snap['mission'] = msg.data
            self._snap['mission_t'] = time.monotonic()

    def _on_perception(self, msg):
        """Keep the target finder's status line."""
        with self._lock:
            self._snap['perception'] = msg.data
            self._snap['perception_t'] = time.monotonic()

    # ── command publishing: the only paths out ─────────────────────────
    def publish_drive(self, linear, angular):
        """Publish one teleop velocity. Already clamped by protocol.decode."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self._pub_drive.publish(msg)
        self._last_drive = time.monotonic()
        self._drive_zeroed = (linear == 0.0 and angular == 0.0)

    def publish_stop(self):
        """
        Publish an explicit zero. Not the same as ceasing to publish.

        A client that stops sending has merely gone quiet; a client that
        sends stop has decided. Both end with the robot stationary, but
        only this one does so immediately.
        """
        self.publish_drive(0.0, 0.0)
        self._drive_zeroed = True

    def _drive_watchdog(self):
        """
        Zero the stick if drive frames stop arriving.

        The browser sends `drive` continuously while the stick is held.
        Silence means the client is gone or the socket stalled, and a held
        velocity must not outlive the client that asked for it.
        """
        if self._drive_zeroed:
            return
        if time.monotonic() - self._last_drive > DRIVE_TIMEOUT_S:
            self.get_logger().warn(
                'drive frames stopped arriving; zeroing the stick')
            self.publish_stop()

    def publish_mode(self, ui_mode):
        """Publish a drive mode in the arbiter's spelling."""
        arbiter_mode = protocol.mode_to_arbiter(ui_mode)
        if arbiter_mode is None:
            return False
        self._pub_mode.publish(String(data=arbiter_mode))
        return True

    def publish_colour(self, colour):
        """Publish the fetch target colour."""
        self._pub_colour.publish(String(data=colour))
        return True

    def publish_goal(self, x, y):
        """Publish a map-frame Nav2 goal with identity orientation."""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self._pub_goal.publish(msg)
        return True

    def publish_arm(self, shoulder, elbow):
        """Publish an arm trajectory, clamped to the URDF's own limits."""
        if self.arm_limits is None:
            return False
        low_s, high_s = self.arm_limits['shoulder']
        low_e, high_e = self.arm_limits['elbow']
        point = JointTrajectoryPoint()
        point.positions = [
            max(low_s, min(high_s, float(shoulder))),
            max(low_e, min(high_e, float(elbow))),
        ]
        point.time_from_start.sec = 1
        msg = JointTrajectory()
        msg.joint_names = ['m_link1_Revolute-6', 'm_link2_Revolute-7']
        msg.points = [point]
        self._pub_arm.publish(msg)
        return True

    def publish_gripper(self, grip):
        """Publish a gripper trajectory; the second joint mirrors the first."""
        if self.grip_limits is None:
            return False
        low, high = self.grip_limits
        value = max(low, min(high, float(grip)))
        point = JointTrajectoryPoint()
        point.positions = [value, -value]
        point.time_from_start.sec = 1
        msg = JointTrajectory()
        msg.joint_names = ['m_link3_Revolute-8', 'm_link3_Revolute-9']
        msg.points = [point]
        self._pub_grip.publish(msg)
        return True

    def call_mission(self, action):
        """
        Call /mission/start or /mission/abort. Returns (ok, message).

        Non-blocking by design: this runs on the tornado thread, and
        waiting on a ROS future there would stall every other client's
        telemetry. The service is fire-and-check-availability; the real
        answer arrives on /mission/state, which the UI is already showing.
        """
        client = self._srv_start if action == 'start' else self._srv_abort
        name = '/mission/start' if action == 'start' else '/mission/abort'
        if not client.service_is_ready():
            return False, f'{name} is not available (is the mission '\
                          f'executive running?)'
        client.call_async(Trigger.Request())
        return True, f'{name} called'

    # ── the observation the session model consumes ─────────────────────
    def snapshot(self):
        """Copy the latest observations consistently, with staleness flags."""
        now = time.monotonic()
        with self._lock:
            snap = dict(self._snap)
        fresh = {
            'robot': now - snap['odom_t'] < STALE_AFTER_S,
            'arbiter': now - snap['arbiter_t'] < STALE_AFTER_S,
            'mission': now - snap['mission_t'] < STALE_AFTER_S,
            'perception': now - snap['perception_t'] < STALE_AFTER_S,
            'navigation': now - snap['nav_t'] < STALE_AFTER_S,
        }
        return snap, fresh

    def simulator_up(self):
        """
        Whether Gazebo is driving the clock.

        ``use_sim_time`` being set is not evidence; a node can be told to
        use sim time and then wait forever for a /clock that never comes,
        which looks exactly like a hung mission. So this asks whether
        anything is actually publishing /clock.
        """
        return self.count_publishers('/clock') > 0

    def camera_streams(self):
        """MJPEG stream metadata for the browser. See the module docstring."""
        video = self.video_port
        camera = str(self.get_parameter('camera_topic').value)
        annotated = str(self.get_parameter('annotated_topic').value)
        depth = str(self.get_parameter('depth_topic').value)
        streams = {
            'camera': _stream(video, camera),
            'annotated': _stream(video, annotated),
        }
        # Depth stays off unless a topic was configured: C2-NAV.43 left
        # depth fusion a candidate, OFF by default, and the browser must
        # not be the thing that quietly turns it on.
        streams['depth'] = _stream(video, depth) if depth else None
        return streams


def _stream(port, topic):
    """One MJPEG stream descriptor, as web_video_server serves it."""
    return {
        'topic': topic,
        'encoding': 'mjpeg',
        'path': f'/stream?topic={topic}&type=mjpeg&quality=60',
        'port': port,
    }


class Platform:
    """Glue: owns the node, the session registry and the client set."""

    def __init__(self, node):
        """Create the sole session and prepare for clients."""
        self.node = node
        self.registry = session_mod.SessionRegistry()
        self.session = self.registry.sole()
        self.clients = set()
        self.seq = 0

    def refresh(self):
        """Re-observe every component and return the telemetry payload."""
        snap, fresh = self.node.snapshot()
        sess = self.session
        sess.observe(session_mod.ROS, True, 'rclpy spinning')
        sess.observe(session_mod.SIMULATOR, self.node.simulator_up(),
                     '/clock publisher present')
        sess.observe(session_mod.ROBOT, fresh['robot'],
                     '/diff_drive_controller/odom')
        sess.observe(session_mod.ARBITER, fresh['arbiter'],
                     '/cmd_vel_arbiter/status')
        sess.observe(session_mod.MISSION, fresh['mission'], '/mission/state')
        sess.observe(session_mod.PERCEPTION, fresh['perception'],
                     '/perception/status')
        sess.observe(session_mod.NAVIGATION, fresh['navigation'],
                     '/plan or /amcl_pose')

        arbiter = tele.parse_arbiter_status(
            snap['arbiter'] if fresh['arbiter'] else '')
        mission = tele.parse_mission_state(
            snap['mission'] if fresh['mission'] else '')
        perception = tele.parse_perception_status(
            snap['perception'] if fresh['perception'] else '')
        sess.active_mission = mission['state'] or ''
        if mission['colour']:
            sess.target_colour = mission['colour']

        self.seq += 1
        return protocol.telemetry(
            seq=self.seq,
            stamp=time.time(),
            robot={
                'pose': snap['pose'],
                'velocity': snap['velocity'],
                'online': fresh['robot'],
            },
            mission=mission,
            nav={
                'online': fresh['navigation'],
                'path': snap['path'] or [],
                'active_source': arbiter['active'],
            },
            sensors={
                'lidar': snap['scan'],
                'perception': perception,
                'streams': self.node.camera_streams(),
            },
            platform={
                'session': sess.as_dict(),
                'arbiter': arbiter,
            })

    def broadcast(self, frame):
        """Send one frame to every connected client, dropping dead sockets."""
        payload = protocol.encode(frame)
        for client in list(self.clients):
            try:
                client.write_message(payload)
            except tornado.websocket.WebSocketClosedError:
                self.clients.discard(client)


class ControlSocket(tornado.websocket.WebSocketHandler):
    """One browser connection, speaking protocol.PROTOCOL_VERSION."""

    def initialize(self, platform):
        """Receive the shared Platform from the Application's route table."""
        self.platform = platform
        self.client_id = None

    def check_origin(self, origin):
        """
        Accept any origin.

        P0.1 is a single-user LOCAL appliance with no authentication and
        no secrets to steal, and locking the origin down here would break
        the documented "open it from your phone on the same LAN" workflow
        without protecting anything. This is a deliberate, recorded
        decision, and it is the first thing that must change before the
        platform is exposed beyond localhost -- see the security boundary
        section of docs/WEB_API.md and the P1.0 row of docs/ROADMAP.md.
        """
        return True

    def open(self):       # noqa: A003 - tornado names this handler hook
        """Register the client and send the welcome frame."""
        self.client_id = f'{id(self):x}'
        self.platform.clients.add(self)
        self.platform.session.attach(self.client_id)
        node = self.platform.node
        limits = {
            'linear': safety.MAX_LINEAR,
            'angular': safety.MAX_ANGULAR,
            'colours': list(node.colours),
            'modes': list(protocol.UI_MODES),
            'arm': ({k: list(v) for k, v in node.arm_limits.items()}
                    if node.arm_limits else None),
            'gripper': list(node.grip_limits) if node.grip_limits else None,
        }
        self.write_message(protocol.encode(protocol.welcome(
            self.platform.session.as_dict(),
            node.camera_streams(),
            limits)))

    def on_close(self):
        """
        Deregister, and stop the robot if that was the last client.

        Closing the tab is not a reason to keep driving. The node's own
        watchdog would catch it within DRIVE_TIMEOUT_S anyway; this makes
        it immediate.
        """
        self.platform.clients.discard(self)
        if self.client_id:
            remaining = self.platform.session.detach(self.client_id)
            if remaining == 0:
                self.platform.node.publish_stop()

    def on_message(self, message):
        """Validate one client frame and act on it."""
        node = self.platform.node
        try:
            frame = protocol.decode(message, colours=node.colours)
        except protocol.ProtocolError as exc:
            self.write_message(protocol.encode(
                protocol.error(exc.code, str(exc), exc.frame_id)))
            return

        kind = frame['type']
        frame_id = frame.get('id')
        try:
            ok, detail = self._dispatch(kind, frame, node)
        except Exception as exc:       # noqa: BLE001 - see below
            # One client's bad frame must not take down the server for
            # every other client, so the dispatch boundary is broad on
            # purpose. The error is logged with its type, not swallowed.
            node.get_logger().error(
                f'{kind} frame failed: {type(exc).__name__}: {exc}')
            self.write_message(protocol.encode(protocol.error(
                'command_failed', f'{kind} failed: {exc}', frame_id)))
            return

        if ok:
            self.write_message(protocol.encode(protocol.ack(frame_id, kind)))
        else:
            self.write_message(protocol.encode(protocol.error(
                'refused', detail or f'{kind} refused', frame_id)))

    def _dispatch(self, kind, frame, node):
        """Map a validated frame onto the allowlisted ROS action."""
        if kind == 'hello':
            return True, ''
        if kind == 'ping':
            self.write_message(protocol.encode(protocol.pong(frame['t'])))
            return True, ''
        if kind == 'drive':
            node.publish_drive(frame['linear'], frame['angular'])
            return True, ''
        if kind == 'stop':
            node.publish_stop()
            return True, ''
        if kind == 'set_mode':
            return node.publish_mode(frame['mode']), 'unknown mode'
        if kind == 'select_target':
            node.publish_colour(frame['colour'])
            self.platform.session.target_colour = frame['colour']
            return True, ''
        if kind == 'mission':
            return node.call_mission(frame['action'])
        if kind == 'nav_goal':
            return node.publish_goal(frame['x'], frame['y']), ''
        if kind == 'set_arm':
            return (node.publish_arm(frame['shoulder'], frame['elbow']),
                    'arm limits unavailable (coco_config not importable)')
        if kind == 'set_gripper':
            return (node.publish_gripper(frame['grip']),
                    'gripper limits unavailable (coco_config not importable)')
        if kind == 'subscribe':
            # Streams are advertised, not gated: P0.1 sends every stream to
            # every client. Accepting the frame keeps the client's code the
            # same when P0.2 makes it selective.
            return True, ''
        return False, f'unhandled frame type {kind!r}'


class HealthHandler(tornado.web.RequestHandler):
    """``GET /healthz`` -- 200 only when the session is READY."""

    def initialize(self, platform):
        """Receive the shared Platform."""
        self.platform = platform

    def get(self):
        """Report component health, with an honest HTTP status."""
        self.platform.refresh()
        body, status = self.platform.session.health()
        body['protocol'] = protocol.PROTOCOL_VERSION
        self.set_status(status)
        self.set_header('Content-Type', 'application/json')
        self.write(json.dumps(body, indent=1))

    def compute_etag(self):
        """No ETag: health must never be served from a cache."""
        return None


class SessionHandler(tornado.web.RequestHandler):
    """``GET /api/session`` -- the session document, for scripts and tests."""

    def initialize(self, platform):
        """Receive the shared Platform."""
        self.platform = platform

    def get(self):
        """Return the current session as JSON."""
        self.platform.refresh()
        self.set_header('Content-Type', 'application/json')
        self.write(json.dumps(self.platform.session.as_dict(), indent=1))

    def compute_etag(self):
        """No ETag: the session changes continuously."""
        return None


def make_app(platform, web_root):
    """Build the tornado Application: API first, static files last."""
    return tornado.web.Application([
        (r'/ws', ControlSocket, {'platform': platform}),
        (r'/healthz', HealthHandler, {'platform': platform}),
        (r'/api/session', SessionHandler, {'platform': platform}),
        (r'/(.*)', tornado.web.StaticFileHandler,
         {'path': web_root, 'default_filename': 'index.html'}),
    ])


def _default_web_root():
    """Find the installed web/ directory, or the source one."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('coco_web'), 'web')
    except Exception:                                  # noqa: BLE001
        return os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), 'web')


def main(args=None):
    """Spin rclpy in a thread and run tornado on the main thread."""
    rclpy.init(args=args)
    node = CocoWebNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=_spin, args=(executor,), daemon=True, name='coco_web_rclpy')
    spin_thread.start()

    platform = Platform(node)
    web_root = str(node.get_parameter('web_root').value) or _default_web_root()
    app = make_app(platform, web_root)
    app.listen(node.http_port, address=node.bind)

    ticker = tornado.ioloop.PeriodicCallback(
        lambda: platform.broadcast(platform.refresh()),
        1000.0 / TELEMETRY_HZ)
    ticker.start()

    node.get_logger().info(f'serving {web_root}')
    try:
        tornado.ioloop.IOLoop.current().start()
    except KeyboardInterrupt:
        pass
    finally:
        ticker.stop()
        # Stop the robot on the way out. A web server exiting is not a
        # reason for the wheels to keep their last command until the
        # controller's own watchdog notices.
        try:
            node.publish_stop()
        except Exception:                              # noqa: BLE001
            pass
        executor.shutdown()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


def _spin(executor):
    """Spin the executor until the context goes down."""
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    main()
