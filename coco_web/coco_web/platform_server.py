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
P0.1 kept images off this socket entirely: ``web_video_server`` served
MJPEG on :8081 and the browser was sent its *metadata*. That is efficient
and it is out of band, which is exactly the problem -- a stream the
platform does not carry is a stream it cannot subscribe per client, cannot
rate-limit, and cannot count. "A slow browser must not grow ROS memory"
is unprovable about a socket this process does not own.

So P0.2 encodes JPEG here and sends **binary frames** (``binary.py``),
subject to the same subscription, rate and backpressure rules as every
other stream. MJPEG stays available behind ``video:=`` for one release,
the way the rosbridge panel was retired.

The cost is honest and written down: the frame is encoded ONCE for every
viewer, so quality and scale are shared and the most demanding subscriber
wins. Encoding per client would cost a JPEG per viewer to save bandwidth
nobody is short of on a LAN.

Depth remains OFF unless ``depth_topic`` is configured. It is a
visualisation -- a greyscale picture with the metre range it mapped --
and nothing here feeds a costmap. C2-NAV.43 left depth *fusion* a
candidate that is off, and the browser must not be what turns it on.
"""

import base64
import json
import os
import threading
import time

from coco_web import binary, imaging, metrics as metrics_mod
from coco_web import protocol, safety
from coco_web import session as session_mod
from coco_web import streams as streams_mod
from coco_web import telemetry as tele

from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                               TwistStamped)

from nav_msgs.msg import OccupancyGrid, Odometry, Path

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from sensor_msgs.msg import Image, LaserScan

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

#: WebSocket keepalive. A client whose network disappeared without
#: closing the socket looks connected forever, and here that is not
#: cosmetic: the last client disconnecting is what stops the robot, so a
#: ghost client keeps the platform believing someone is watching.
PING_INTERVAL_S = 10.0
PING_TIMEOUT_S = 30.0

#: How long a client keeps the stick after its last drive frame. Matches
#: DRIVE_TIMEOUT_S: the moment the server would zero a stale stick is
#: exactly the moment someone else may take it.
PILOT_TIMEOUT_S = DRIVE_TIMEOUT_S


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
        # The COCO-specific proof that the simulator up there is OURS.
        # See simulator_up() for why /clock alone is not enough.
        self.declare_parameter('sim_topic', '/model/coco/odometry')

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
            'grasp': '', 'grasp_t': 0.0,
            'colour': '', 'colour_t': 0.0,
            'camera': None, 'depth': None,
            'odom_t': 0.0, 'clock_t': 0.0, 'nav_t': 0.0, 'map': None,
        }
        self._last_drive = 0.0
        self._drive_zeroed = True
        # Which optional sensor streams clients are watching, and the
        # rclpy subscriptions currently open for them. The two are
        # reconciled on the rclpy thread (see _reconcile_streams): the
        # tornado thread only ever writes the demand.
        self._wanted_streams = set()
        self._open_streams = {}
        self._image_seq = {'camera': 0, 'depth': 0}
        self._image_config = {}
        self._depth_clip = self._load_depth_clip()
        self.metrics = metrics_mod.Metrics(streams_mod.STREAMS)

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
        # /map is latched TRANSIENT_LOCAL by nav2_map_server and published
        # once at activation, so a late-joining server needs the matching
        # durability or it waits forever for a message already sent.
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, _latched_qos())
        self.create_subscription(
            String, '/cmd_vel_arbiter/status', self._on_arbiter, 10)
        self.create_subscription(
            String, '/mission/state', self._on_mission, 10)
        self.create_subscription(
            String, '/perception/status', self._on_perception, 10)
        self.create_subscription(
            String, '/grasp/status', self._on_grasp, 10)
        # The AUTHORITATIVE target colour. /mission/state has never
        # carried one -- P0.1 looked for a `colour` field there that the
        # executive does not emit, so the colour on the wire was
        # permanently null and only the browser's own echo filled it in.
        self.create_subscription(
            String, '/mission/target_colour', self._on_colour, 10)

        # The drive watchdog runs on the steady clock: a paused simulator
        # must not freeze the thing that stops the robot.
        self.create_timer(0.1, self._drive_watchdog)
        # Optional sensor subscriptions are created and destroyed here,
        # on the rclpy thread. See want_streams for why not inline.
        self.create_timer(0.25, self._reconcile_streams)

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

    def _load_depth_clip(self):
        """
        Return the depth camera's near/far clip, for the greyscale ramp.

        Taken from coco_config rather than from the frame's own min and
        max: a per-frame range makes the picture rescale every time the
        robot moves, which reads as the depth changing when only the
        contrast did.
        """
        try:
            from coco_config.robot import CAMERA_DEPTH_CLIP
            return tuple(CAMERA_DEPTH_CLIP)
        except ImportError:
            return None

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

    def _on_map(self, msg):
        """
        Store the occupancy grid as a base64 frame, once per change.

        Kept out of the telemetry tick: the grid does not move, and
        re-sending 102 400 cells ten times a second to redraw a static
        picture is the kind of cost that only shows up on someone's
        phone. ``_map_seq`` lets the broadcast loop notice a new one.
        """
        cells = bytes((value & 0xFF) for value in msg.data)
        payload = protocol.map_frame(
            msg.info.width, msg.info.height, msg.info.resolution,
            msg.info.origin.position.x, msg.info.origin.position.y,
            base64.b64encode(cells).decode('ascii'))
        with self._lock:
            self._snap['map'] = payload
            self._snap['map_seq'] = self._snap.get('map_seq', 0) + 1

    def _on_arbiter(self, msg):
        """Keep the arbiter's own status line raw; it is parsed later."""
        with self._lock:
            self._snap['arbiter'] = msg.data
            self._snap['arbiter_t'] = time.monotonic()

    def _on_mission(self, msg):
        """Keep the mission executive's status line, and stamp its arrival."""
        with self._lock:
            changed = msg.data != self._snap['mission']
            self._snap['mission'] = msg.data
            self._snap['mission_t'] = time.monotonic()
        # Only a CHANGED line starts the latency clock. The executive
        # re-asserts the same line at 2 Hz, and measuring the delay to a
        # repeat would report the tick interval rather than how far
        # behind the UI is.
        if changed:
            self.metrics.mission_seen()

    def _on_perception(self, msg):
        """Keep the target finder's status line."""
        with self._lock:
            self._snap['perception'] = msg.data
            self._snap['perception_t'] = time.monotonic()

    def want_streams(self, wanted):
        """
        Record which optional sensor streams have a viewer.

        Called from the tornado thread, so it only writes a set. The
        rclpy objects are created and destroyed on the executor thread by
        ``_reconcile_streams`` -- building a subscription from another
        thread races the spin loop, and the resulting failures look like
        a sensor that intermittently does not exist.
        """
        with self._lock:
            self._wanted_streams = set(wanted)

    def _reconcile_streams(self):
        """
        Open or close the optional sensor subscriptions to match demand.

        Runs on the rclpy thread, where creating and destroying
        subscriptions is safe. The tornado thread only ever sets
        ``_wanted_streams``; doing the rclpy work there races the spin
        loop, and the resulting failures present as a sensor that
        intermittently does not exist.
        """
        with self._lock:
            wanted = set(self._wanted_streams)
        topics = {
            'camera': str(self.get_parameter('camera_topic').value),
            'depth': str(self.get_parameter('depth_topic').value),
        }
        for name in ('camera', 'depth'):
            topic = topics[name]
            open_now = name in self._open_streams
            # Depth with no topic configured stays closed however many
            # clients ask: C2-NAV.43 left depth OFF by default and the
            # browser must not be the thing that turns it on.
            should = name in wanted and bool(topic)
            if should and not open_now:
                self._open_streams[name] = self.create_subscription(
                    Image, topic,
                    (self._on_camera if name == 'camera'
                     else self._on_depth),
                    _sensor_qos(depth=1))
                self.get_logger().info(f'{name}: subscribed to {topic}')
            elif open_now and not should:
                self.destroy_subscription(self._open_streams.pop(name))
                with self._lock:
                    self._snap[name] = None
                self.get_logger().info(f'{name}: no viewers, unsubscribed')

    def _on_camera(self, msg):
        """Encode one colour frame as JPEG for the binary stream."""
        self._encode_image('camera', msg)

    def _on_depth(self, msg):
        """Encode one depth frame as a greyscale JPEG for visualisation."""
        self._encode_image('depth', msg)

    def _encode_image(self, name, msg):
        """
        Encode once, here, rather than per connected client.

        Two clients watching the same camera at the same rate should cost
        one JPEG, not two. Per-client rate and quality are honoured by
        the sender choosing whether to forward this frame -- which means
        quality is shared, and the fastest subscriber's setting wins.
        That is the trade: one encode for everyone, and it is written
        down rather than discovered.
        """
        try:
            if name == 'depth':
                jpeg, width, height, low, high = imaging.depth_jpeg(
                    msg.width, msg.height, msg.encoding, bytes(msg.data),
                    quality=self._image_quality(name),
                    scale=self._image_scale(name),
                    clip=self._depth_clip)
                extra = {'min_m': round(low, 3), 'max_m': round(high, 3),
                         'palette': 'grey_near_bright'}
            else:
                jpeg, width, height = imaging.colour_jpeg(
                    msg.width, msg.height, msg.encoding, bytes(msg.data),
                    quality=self._image_quality(name),
                    scale=self._image_scale(name))
                extra = None
        except imaging.ImageError as exc:
            # Rate-limited: a wrong encoding would otherwise log at the
            # sensor's own 15 Hz and bury everything else.
            self.get_logger().warn(
                f'{name}: {exc.code}: {exc}', throttle_duration_sec=5.0)
            return
        with self._lock:
            self._image_seq[name] += 1
            self._snap[name] = {
                'seq': self._image_seq[name],
                't': time.time(),
                'w': width, 'h': height, 'jpeg': jpeg, 'extra': extra,
                'quality': self._image_quality(name),
            }
        self.metrics.observed(name, len(msg.data))

    def _image_quality(self, name):
        """Return the quality every viewer of `name` shares."""
        return self._image_config.get(name, {}).get(
            'quality', streams_mod.LIMITS[name]['quality'])

    def _image_scale(self, name):
        """Return the scale every viewer of `name` shares."""
        return self._image_config.get(name, {}).get(
            'scale', streams_mod.LIMITS[name]['scale'])

    def set_image_config(self, config):
        """Record the encode settings the fastest subscriber asked for."""
        self._image_config = config

    def _on_grasp(self, msg):
        """Keep the grasp server's status line; it is parsed specially."""
        with self._lock:
            self._snap['grasp'] = msg.data
            self._snap['grasp_t'] = time.monotonic()

    def _on_colour(self, msg):
        """Record the mission's own target colour, not the browser's."""
        with self._lock:
            self._snap['colour'] = msg.data
            self._snap['colour_t'] = time.monotonic()

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
            'grasp': now - snap['grasp_t'] < STALE_AFTER_S,
            # The colour is latched, not periodic: the executive
            # publishes it once, on change. Staleness would report a
            # perfectly good colour as gone three seconds later.
            'colour': bool(snap['colour']),
        }
        return snap, fresh

    def simulator_up(self):
        """
        Whether COCO's own simulator is running.

        ``use_sim_time`` being set is not evidence; a node can be told to
        use sim time and then wait forever for a /clock that never comes,
        which looks exactly like a hung mission. So this asks whether
        something is actually publishing.

        But /clock ALONE is not evidence either, and this was measured,
        not guessed: with no COCO simulator running at all, a bare probe
        on this machine still found ``count_publishers('/clock') == 1``,
        because an unrelated project's Gazebo was up on the same ROS
        graph. Reporting "simulator: up" then is the precise failure
        TASK 10 warns about -- a green light for a simulator that cannot
        answer. So a COCO-specific topic has to be there too.

        ``/model/coco/odometry`` is the gz plugin's own output. It is the
        right probe because it appears well BEFORE the ros2_control stack
        finishes activating, which is exactly the "simulator up, robot not
        yet" state the session model wants to be able to report -- and it
        is what verify_all.sh already gates on for the same reason.
        """
        if self.count_publishers('/clock') <= 0:
            return False
        sim_topic = str(self.get_parameter('sim_topic').value)
        return self.count_publishers(sim_topic) > 0

    def world_geometry(self):
        """
        Build the world the browser draws, in MAP coordinates.

        Sent once in `welcome` so the page re-types no geometry -- the
        same rule the arm limits follow, and for the same reason: the old
        panel hard-coded numbers in HTML and they drifted from the URDF.

        The frame shift is the subtle part. coco_config's geometry is in
        WORLD coordinates; every pose the browser draws against
        (``/amcl_pose``, ``/plan``, ``/map``) is in the MAP frame, whose
        origin is the robot's spawn point. So the offset is exactly
        ``-SPAWN_XY[0]``, which is 2.0 -- and that is *derived* here
        rather than copied from ``mission_states.WORLD_TO_MAP_X``, with a
        test asserting the two agree. Drawing the ramp 2 m from where the
        robot climbs it is the kind of error that looks like a broken
        localisation system.
        """
        try:
            from coco_config.robot import (
                PLATFORM_LEN, RAMP_FOOT_X, RAMP_SUMMIT_X, RAMP_WIDTH,
                SPAWN_XY, TARGET_ROW_X, TARGETS,
            )
        except ImportError:
            self.get_logger().warn(
                'coco_config unavailable; the browser will draw a bare '
                'grid instead of the world')
            return None
        shift = -SPAWN_XY[0]
        return {
            'frame': 'map',
            'offset_x': shift,
            'ramp': {
                'x0': RAMP_FOOT_X + shift,
                'x1': RAMP_SUMMIT_X + shift,
                'width': RAMP_WIDTH,
            },
            'platform': {
                'x0': RAMP_SUMMIT_X + shift,
                'x1': RAMP_SUMMIT_X + PLATFORM_LEN + shift,
                'width': RAMP_WIDTH,
            },
            'home': {'x': SPAWN_XY[0] + shift, 'y': SPAWN_XY[1]},
            'targets': [
                {
                    'colour': target.colour,
                    'x': TARGET_ROW_X + shift,
                    'y': target.lane_y,
                    'diameter': target.diameter,
                }
                for target in TARGETS
            ],
        }

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
        self._map_sent = 0
        self._lidar_seq = 0
        self._image_sent = {'camera': 0, 'depth': 0}
        # Which client currently holds the stick, and when it last used
        # it. Two browsers on one robot is a supported situation -- the
        # phone in your hand and the laptop on the desk -- but two
        # joysticks fighting is not, and the arbiter cannot help: both
        # arrive on /cmd_vel_teleop as the same source.
        self._pilot = None
        self._pilot_at = 0.0

    def pilot_claim(self, client):
        """
        Give `client` the stick if nobody else is actively holding it.

        Control lapses after DRIVE_TIMEOUT_S of silence rather than being
        held until disconnect, so a tab left open in a background window
        does not lock out the person actually driving.
        """
        now = time.monotonic()
        if (self._pilot is not None and self._pilot is not client
                and now - self._pilot_at < PILOT_TIMEOUT_S):
            return False
        self._pilot = client
        self._pilot_at = now
        return True

    def pilot_release(self, client):
        """Drop the stick if `client` was holding it."""
        if self._pilot is client:
            self._pilot = None

    def pilot_clear(self):
        """Drop the stick whoever holds it. STOP releases for everyone."""
        self._pilot = None

    def pilot_id(self):
        """Return the id of the client holding the stick, or None."""
        return getattr(self._pilot, 'client_id', None)

    def map_payload(self):
        """Return the current occupancy-grid frame, or None."""
        snap, _fresh = self.node.snapshot()
        return snap.get('map')

    def demand(self):
        """
        Which optional sensor streams at least one client is watching.

        The camera and depth subscriptions are created only while someone
        is looking: decoding and re-encoding a 320x240 frame fifteen
        times a second for nobody is pure waste, and on this machine the
        simulator needs the CPU more than an unwatched tab does.
        """
        wanted = set()
        for client in self.clients:
            subscription = getattr(client, 'subscription', None)
            if subscription is None:
                continue
            for stream in streams_mod.BINARY_STREAMS:
                if subscription.wants(stream):
                    wanted.add(stream)
        return wanted

    def reconcile(self):
        """Tell the node which optional sensor streams are wanted now."""
        self.node.want_streams(self.demand())

    def tick(self):
        """
        Broadcast one telemetry frame, and the map when it has changed.

        The map rides the same loop rather than its own timer so there is
        one place where "what goes out, and how often" is decided.
        """
        metrics = self.node.metrics
        metrics.clients = len(self.clients)
        self.broadcast(self.refresh(), 'telemetry')
        metrics.mission_delivered()
        snap, _fresh = self.node.snapshot()
        seq = snap.get('map_seq', 0)
        if seq != self._map_sent and snap.get('map'):
            self._map_sent = seq
            self.broadcast(snap['map'], 'map')
        self.push_sensors(snap)
        metrics.sample()

    def push_sensors(self, snap):
        """
        Send the binary sensor frames every subscriber is due for.

        The frame is built once per stream and written to each client
        that wants it, is due for it, and is not already behind. A client
        that IS behind has the frame dropped and counted rather than
        queued -- see streams.py.
        """
        metrics = self.node.metrics
        self.node.set_image_config(self.encode_config())
        frames = {}
        lidar = snap.get('scan')
        if lidar:
            self._lidar_seq += 1
            frames['lidar'] = binary.lidar_frame(
                self._lidar_seq, time.time(), lidar)
        for name in ('camera', 'depth'):
            image = snap.get(name)
            if not image or image['seq'] == self._image_sent.get(name):
                continue
            self._image_sent[name] = image['seq']
            frames[name] = binary.image_frame(
                name, image['seq'], image['t'], image['w'], image['h'],
                image['jpeg'], quality=image['quality'],
                extra=image['extra'])
        for stream, blob in frames.items():
            for client in list(self.clients):
                try:
                    if client.send_binary(stream, blob):
                        metrics.sent(stream, len(blob))
                    metrics.buffered(client.buffered_bytes())
                except tornado.websocket.WebSocketClosedError:
                    self.clients.discard(client)
        for stream in streams_mod.BINARY_STREAMS + ('lidar',):
            metrics.dropped[stream] = sum(
                client.subscription.dropped.get(stream, 0)
                for client in self.clients)

    def encode_config(self):
        """
        Merge every viewer's settings into the one encode that happens.

        The frame is encoded once for everyone, so where clients disagree
        the HIGHER setting wins -- a viewer who asked for more detail
        gets it, and one who asked for less is merely sent a better
        picture than they needed. The alternative is encoding per client,
        which costs a JPEG per viewer to save bandwidth nobody is short
        of on a LAN.
        """
        merged = {}
        for client in self.clients:
            subscription = getattr(client, 'subscription', None)
            if subscription is None:
                continue
            for name in streams_mod.BINARY_STREAMS:
                if not subscription.wants(name):
                    continue
                config = subscription.config.get(name, {})
                target = merged.setdefault(name, {})
                for key in ('quality', 'scale'):
                    if key in config:
                        target[key] = max(target.get(key, 0), config[key])
        return merged

    def refresh(self):
        """Re-observe every component and return the telemetry payload."""
        snap, fresh = self.node.snapshot()
        sess = self.session
        sess.observe(session_mod.ROS, True, 'rclpy spinning')
        sess.observe(session_mod.SIMULATOR, self.node.simulator_up(),
                     '/clock and COCO sim odometry both present')
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
        # The colour the MISSION believes in, falling back to the one
        # this browser picked only while the topic is silent -- which is
        # the honest order: the executive is the authority, the
        # selection is a request.
        colour = (snap['colour'] if fresh['colour']
                  else sess.target_colour) or None
        mission = tele.parse_mission_state(
            snap['mission'] if fresh['mission'] else '',
            colour=colour, now=time.time())
        perception = tele.parse_perception_status(
            snap['perception'] if fresh['perception'] else '')
        grasp = tele.parse_grasp_status(
            snap['grasp'] if fresh['grasp'] else '')
        sess.active_mission = mission['state'] or ''
        sess.mission_running = bool(mission['active'])
        if colour:
            sess.target_colour = colour

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
                'grasp': grasp,
                'streams': self.node.camera_streams(),
            },
            platform={
                'session': sess.as_dict(),
                'arbiter': arbiter,
                'perf': self.node.metrics.as_dict(),
                'connection': sess.connection(),
                'pilot': self.pilot_id(),
            })

    def broadcast(self, frame, stream='telemetry'):
        """
        Send one frame to every client subscribed to `stream`.

        The frame is built once and filtered per client rather than
        rebuilt per client: the expensive part is observing the robot,
        not copying a dict, and doing it once keeps every client's view
        of the same tick consistent.
        """
        for client in list(self.clients):
            try:
                client.send_stream(frame, stream)
            except tornado.websocket.WebSocketClosedError:
                self.clients.discard(client)

    @staticmethod
    def filtered(frame, subscription):
        """
        Blank the telemetry sections this client did not subscribe to.

        Sections are set to None rather than removed. That is already
        their meaning before the first message arrives, so a client needs
        no new branch -- and a client that never sent `subscribe` gets
        the P0.1 default set, so it sees no change at all.
        """
        if subscription is None:
            return frame
        view = dict(frame)
        if not subscription.wants('mission'):
            view['mission'] = None
        if not subscription.wants('lidar') or not subscription.wants(
                'telemetry'):
            sensors = dict(view.get('sensors') or {})
            sensors['lidar'] = None
            view['sensors'] = sensors
        if not subscription.wants('path'):
            nav = dict(view.get('nav') or {})
            nav['path'] = []
            view['nav'] = nav
        return view


class ControlSocket(tornado.websocket.WebSocketHandler):
    """One browser connection, speaking protocol.PROTOCOL_VERSION."""

    def initialize(self, platform):
        """Receive the shared Platform from the Application's route table."""
        self.platform = platform
        self.client_id = None
        self.subscription = streams_mod.Subscription()

    # ── outbound ───────────────────────────────────────────────────────
    def buffered_bytes(self):
        """
        Return the bytes tornado is still holding, or 0 if unknown.

        Reached through two optional attributes because a closing or
        mocked connection has neither, and a backpressure check that
        raises is worse than one that assumes the socket is idle.
        """
        connection = getattr(self, 'ws_connection', None)
        stream = getattr(connection, 'stream', None)
        return getattr(stream, '_write_buffer_size', 0) or 0

    def send_stream(self, frame, stream):
        """
        Write one JSON frame if this client is subscribed to `stream`.

        Control and telemetry frames are never rate-limited or dropped;
        only sensor streams are. See streams.py for why.
        """
        if not self.subscription.wants(stream):
            return False
        payload = protocol.encode(
            self.platform.filtered(frame, self.subscription)
            if stream == 'telemetry' else frame)
        self.write_message(payload)
        self.subscription.mark_sent(stream)
        self.platform.node.metrics.sent(stream, len(payload))
        return True

    def send_binary(self, stream, blob):
        """
        Write one binary sensor frame, dropping it if the client is behind.

        Returns True when written. A drop is counted and reported to the
        client in the NEXT frame's header, so loss is visible rather than
        silent.
        """
        if not self.subscription.wants(stream):
            return False
        if not self.subscription.due(stream):
            return False
        if not self.subscription.ready(stream, self.buffered_bytes()):
            self.subscription.mark_dropped(stream)
            return False
        pending = self.write_message(blob, binary=True)
        self.subscription.mark_sent(stream, pending)
        return True

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
            limits,
            subscriptions=self.subscription.as_dict(),
            world=node.world_geometry())))
        # The map is broadcast only when it changes, so a client joining
        # after that would otherwise draw an empty view until the next
        # map_server restart -- which, for a static map, is never.
        current_map = self.platform.map_payload()
        if current_map and self.subscription.wants('map'):
            self.write_message(protocol.encode(current_map))

    def on_close(self):
        """
        Deregister, and stop the robot if that was the last client.

        Closing the tab is not a reason to keep driving. The node's own
        watchdog would catch it within DRIVE_TIMEOUT_S anyway; this makes
        it immediate.
        """
        self.platform.clients.discard(self)
        # A driver closing its tab must not keep the stick, or the next
        # client to connect is locked out by a browser that no longer
        # exists.
        self.platform.pilot_release(self)
        # The last watcher leaving must close the camera subscription,
        # not leave it decoding frames for an empty room.
        self.platform.reconcile()
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
            # A refusal carries a specific code where one exists, so a UI
            # can distinguish "someone else is driving" -- which resolves
            # itself -- from "the robot said no", which does not.
            code = detail if detail in protocol.REFUSAL_CODES else 'refused'
            message = (protocol.REFUSAL_CODES.get(detail)
                       or detail or f'{kind} refused')
            self.write_message(protocol.encode(protocol.error(
                code, message, frame_id)))

    def _dispatch(self, kind, frame, node):
        """Map a validated frame onto the allowlisted ROS action."""
        if kind == 'hello':
            self.subscription.binary = frame.get('binary', False)
            return True, ''
        if kind == 'ping':
            self.write_message(protocol.encode(protocol.pong(frame['t'])))
            return True, ''
        if kind == 'drive':
            if not self.platform.pilot_claim(self):
                return False, 'not_in_control'
            node.publish_drive(frame['linear'], frame['angular'])
            return True, ''
        if kind == 'stop':
            # STOP is honoured from ANY client, always, whether or not it
            # holds the stick. A safety control that depends on who is in
            # control is not a safety control -- and the person who can
            # see the robot about to hit something may well be the one
            # watching rather than the one driving.
            node.publish_stop()
            self.platform.pilot_clear()
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
        if kind in ('subscribe', 'unsubscribe'):
            if kind == 'subscribe':
                self.subscription.subscribe(frame['streams'])
            else:
                self.subscription.unsubscribe(frame['streams'])
            self.write_message(protocol.encode(protocol.subscription(
                self.subscription.as_dict())))
            # Subscribing to the map must deliver the map, not wait for
            # one to change: for a static map, "on change" is never.
            if kind == 'subscribe' and 'map' in frame['streams']:
                current_map = self.platform.map_payload()
                if current_map:
                    self.write_message(protocol.encode(current_map))
            self.platform.reconcile()
            return True, ''
        if kind == 'set_stream':
            self.subscription.configure(
                frame['stream'], fps=frame['fps'],
                quality=frame['quality'], scale=frame['scale'])
            self.write_message(protocol.encode(protocol.subscription(
                self.subscription.as_dict())))
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


class MetricsHandler(tornado.web.RequestHandler):
    """``GET /api/metrics`` -- measured rates, drops and CPU."""

    def initialize(self, platform):
        """Receive the shared Platform."""
        self.platform = platform

    def get(self):
        """Return the live counters. Measured, never configured values."""
        self.set_header('Content-Type', 'application/json')
        self.write(json.dumps(self.platform.node.metrics.as_dict(), indent=1))

    def compute_etag(self):
        """No ETag: these numbers change continuously."""
        return None


def make_app(platform, web_root):
    """Build the tornado Application: API first, static files last."""
    return tornado.web.Application(
        [
            (r'/ws', ControlSocket, {'platform': platform}),
            (r'/healthz', HealthHandler, {'platform': platform}),
            (r'/api/session', SessionHandler, {'platform': platform}),
            (r'/api/metrics', MetricsHandler, {'platform': platform}),
            (r'/(.*)', tornado.web.StaticFileHandler,
             {'path': web_root, 'default_filename': 'index.html'}),
        ],
        # Protocol-level keepalive. A client whose network vanished
        # without a FIN otherwise sits in `clients` forever, counting as
        # a viewer -- which matters here because the LAST client
        # disconnecting is what stops the robot.
        websocket_ping_interval=PING_INTERVAL_S,
        websocket_ping_timeout=PING_TIMEOUT_S,
    )


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
        platform.tick, 1000.0 / TELEMETRY_HZ)
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
