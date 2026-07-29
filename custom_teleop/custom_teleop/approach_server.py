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
approach_server — cross the crest platform and stop where the arm can work.

The mission has a 1.198 m hole in it and this fills it. The RL climb
terminates at world x = 2.700 (``ramp_env.GOAL_SUMMIT``); the crest is at
3.000, the target row at 4.050, and the base has to stop at ~3.90 for the
arm to reach. Nav2 cannot drive that: from the flat the ramp scans as a
wall at the lidar plane, and the platform behind it is unmapped with
``allow_unknown: false``. The climb episode has ended. And ``/ramp/descend``
drives straight through the row to 6.65 — which is how the M5 run knocked
``target_yellow`` flat.

Why this lives in custom_teleop
-------------------------------
It is a **velocity source**, like cmd_vel_relay and the teleop nodes, and
it belongs next to the arbiter that mediates them. It consumes
``geometry_msgs/PointStamped`` off ``/perception/target``, which needs no
dependency on coco_perception — only on the message type.

Four phases, and why each exists
--------------------------------
``crest``  The robot is still on an 18 degree slope for the last 0.300 m.
           Drive forward holding yaw until the IMU says the chassis has
           levelled off. Pitch, not odometry: 0.314 rad of grade against a
           0.06 rad threshold is not a close call, and it needs no map.
``servo``  Pure pursuit on ``/perception/target``: steer to null the
           bearing, close the range. This is also what absorbs whatever
           lateral error the climb left, so a lane hold that lands within
           a few centimetres is good enough.
``align``  Stop and turn in place until the bearing is nulled properly.
           Pivoting in place is forbidden on a grade — see
           ``ramp_driver.descend_cmd`` — but this happens on level
           platform, where it is exactly the right move.
``creep``  Drive straight, blind, on wheel odometry.

The reason for align-then-creep rather than servoing all the way in:
**perception goes blind before the stop pose.** target_finder's minimum
range is 0.15 m of surface depth and the camera sits at base-x 0.125, so
the last fix lands at a target axis of ~0.29 m while the stop is at ~0.15.
Nulling the *bearing* first means the blind leg runs along the line to the
target, so it closes range without reintroducing lateral error — a
straight 0.17 m at the ~1 % odometry error measured for this base is under
2 mm, against a half-window of 7.75 mm.

Where it stops
--------------
``coco_config.robot.approach_stop_x(colour)`` — the CENTRE of that
colour's approach window, not the demo's fixed 0.152. See its docstring:
the window is bounded below by the chassis nose plus the target's radius
and above by the arm's reach at the *hover* height, and 0.152 is not
central in it.

Topics and services
-------------------
in   /perception/target      geometry_msgs/PointStamped  (base_footprint)
in   /mission/target_colour  std_msgs/String   (which window to stop in)
in   /diff_drive_controller/odom  nav_msgs/Odometry  (the blind creep)
in   /imu                    sensor_msgs/Imu   BEST_EFFORT
out  /cmd_vel_approach       geometry_msgs/TwistStamped
out  /approach/status        std_msgs/String   (5 Hz, key=value)
out  /approach/target        geometry_msgs/PointStamped  (final estimate)
srv  /approach/run           std_srvs/Trigger
srv  /approach/stop          std_srvs/Trigger

Like ramp_driver, this node does **not** publish /mission/mode. The
sequencer owns that; a second publisher would fight the arbiter's latch.
"""

import math
import threading
import time

from coco_config.robot import (approach_stop_x, is_best_effort,
                               TARGET_COLOURS, TARGET_GRASP_Z)

from geometry_msgs.msg import PointStamped, TwistStamped

from nav_msgs.msg import Odometry

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu

from std_msgs.msg import String
from std_srvs.srv import Trigger

APPROACH_CMD_VEL_TOPIC = '/cmd_vel_approach'
APPROACH_HZ = 20.0        # inside the arbiter's 0.3 s staleness window
STATUS_HZ = 5.0
APPROACH_TIMEOUT = 120.0  # s of wall clock for the whole run

# ── crest ──────────────────────────────────────────────────────────────────
# 0.18 m/s is descend_cmd's speed and for the same reason: fast enough not
# to stall on the grade, slow enough that the transition over the crest
# does not launch the nose.
CREST_SPEED = 0.18
# 18 deg of grade is 0.314 rad and the platform is level, so anything in
# between says "still on the slope". 0.06 rad (3.4 deg) is a fifth of the
# grade and well above the chassis's own pitch noise.
CREST_PITCH_EPS = 0.06
CREST_SETTLE = 0.5        # s of level attitude before believing it
# Safety net, not a control input: if the IMU never reports level the robot
# must not keep driving into the target row 1.35 m away.
CREST_MAX_TRAVEL = 1.0
CREST_YAW_GAIN = 1.2
CREST_YAW_CLAMP = 0.4

# ── servo ──────────────────────────────────────────────────────────────────
SERVO_YAW_GAIN = 1.5
SERVO_YAW_CLAMP = 0.5
SERVO_LIN_GAIN = 0.35     # m/s per metre of remaining range
SERVO_LIN_MIN = 0.05
SERVO_LIN_MAX = 0.22
# Hand over to align while perception can still see: the measured cutoff is
# a target axis of ~0.29 m (0.125 camera offset + 0.15 min range + the
# surface-to-axis correction), so 0.32 leaves 30 mm of margin for the align
# to run on live vision.
SERVO_HANDOFF_X = 0.32
SERVO_ACQUIRE_TIMEOUT = 8.0   # s to see the target at all before failing
VISION_STALE = 1.0            # s without a fix that counts as blind

# ── align ──────────────────────────────────────────────────────────────────
# 0.02 rad over the ~0.17 m creep is 3.4 mm of lateral error, which fits
# inside the 7.75 mm half-window with the odometry error on top.
ALIGN_EPS = 0.02
ALIGN_YAW_GAIN = 1.0
ALIGN_YAW_CLAMP = 0.35
# A diff-drive base does not creep off static friction: below ~0.1 rad/s
# the wheels buzz and the robot does not rotate, so a pure proportional
# term would converge to "not quite aligned" and sit there.
ALIGN_YAW_MIN = 0.10
ALIGN_TIMEOUT = 25.0

# ── creep ──────────────────────────────────────────────────────────────────
# Slow on purpose, and slower than it first looked. At APPROACH_HZ the loop
# can only stop on a tick, so the speed IS the stopping quantisation. The
# first end-to-end fetch ran this at 0.06 m/s and overshot its stop by
# 3.5 mm — which was fine against the 7.75 mm half-window the window
# arithmetic claimed at the time, and not fine at all against the 2.75 mm
# the arm's measured self-collision bound actually leaves.
#
# 0.03 m/s is 1.5 mm per tick, plus 0.2 mm of braking at the controller's
# 2.0 m/s^2 limit. CREEP_LEAD stops that half-tick-plus-braking early
# rather than leaving it as bias, which centres the residual instead of
# always overshooting.
CREEP_SPEED = 0.03
CREEP_LEAD = 0.5 * CREEP_SPEED / APPROACH_HZ + CREEP_SPEED ** 2 / (2 * 2.0)
CREEP_YAW_GAIN = 1.0
CREEP_YAW_CLAMP = 0.15
CREEP_TIMEOUT = 60.0

STATUS_KEYS = ('phase', 'tx', 'ty', 'range', 'bearing', 'stop', 'travel',
               'colour', 'outcome')
SIGNED_STATUS_KEYS = ('ty', 'bearing')


def wrap(angle):
    """Fold an angle onto (-pi, pi], the short way round."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    """Symmetric clamp to +/- `limit`."""
    return max(-limit, min(limit, value))


def heading_cmd(yaw_err, gain, limit):
    """Clamped proportional yaw correction toward yaw_err = 0."""
    return clamp(gain * wrap(yaw_err), limit)


def crest_cmd(pitch, yaw_err):
    """
    Command for driving off the slope onto the platform.

    Returns ``(linear, angular, level)``. Speed does not depend on heading
    error: the robot is still on a grade here and a stationary skid-steer
    pivot on a slope is how this base loses its footing.
    """
    return (CREST_SPEED,
            heading_cmd(-yaw_err, CREST_YAW_GAIN, CREST_YAW_CLAMP),
            abs(pitch) <= CREST_PITCH_EPS)


def bearing_to(target_x, target_y):
    """Bearing to a base_footprint point, radians, positive to the left."""
    return math.atan2(target_y, target_x)


def servo_cmd(target_x, target_y, stop_x, handoff_x=SERVO_HANDOFF_X):
    """
    Pure-pursuit command toward a target seen at (target_x, target_y).

    Returns ``(linear, angular, done)``. `done` means "close enough that
    perception is about to lose it" — not "arrived": the remaining range
    is closed by align + creep.
    """
    if target_x <= handoff_x:
        return 0.0, 0.0, True
    angular = clamp(SERVO_YAW_GAIN * bearing_to(target_x, target_y),
                    SERVO_YAW_CLAMP)
    linear = min(SERVO_LIN_MAX,
                 max(SERVO_LIN_MIN, SERVO_LIN_GAIN * (target_x - stop_x)))
    return linear, angular, False


def align_cmd(bearing):
    """
    Turn in place until the target is dead ahead.

    Returns ``(linear, angular, done)``; the linear term is always zero.
    Pivoting in place is the thing descend_cmd refuses to do — but that is
    a rule about grades, and this runs on the level platform, where it is
    the only way to point at something without translating away from it.
    """
    if abs(bearing) <= ALIGN_EPS:
        return 0.0, 0.0, True
    magnitude = min(ALIGN_YAW_CLAMP,
                    max(ALIGN_YAW_MIN, ALIGN_YAW_GAIN * abs(bearing)))
    return 0.0, math.copysign(magnitude, bearing), False


def creep_cmd(travelled, distance, yaw_err, lead=CREEP_LEAD):
    """
    Blind straight-line close over the last stretch, on wheel odometry.

    Returns ``(linear, angular, done)``. Stops `lead` short, because the
    loop can only decide on a tick and the base still has to brake: that
    is a known, one-sided error, and leaving it in makes every stop an
    overshoot instead of a centred one.

    The heading hold is what keeps the blind leg straight; without it a
    residual yaw rate would undo the align over exactly the distance
    perception cannot watch.
    """
    if travelled >= max(0.0, distance - lead):
        return 0.0, 0.0, True
    return (CREEP_SPEED,
            heading_cmd(-yaw_err, CREEP_YAW_GAIN, CREEP_YAW_CLAMP),
            False)


def _status_value(key, value):
    """Format one status field; None reads as '--', never as blank."""
    if value is None:
        return '--'
    if isinstance(value, float):
        return (f'{value:+.3f}' if key in SIGNED_STATUS_KEYS
                else f'{value:.3f}')
    return str(value)


def format_status(**fields):
    """
    One-line approach state, space-separated key=value for the panel.

    Same shape as cmd_vel_arbiter and ramp_driver, so the browser can split
    it without a JSON parser and `ros2 topic echo` stays readable.
    """
    return ' '.join(f'{key}={_status_value(key, fields.get(key))}'
                    for key in STATUS_KEYS)


class ApproachServer(Node):
    """Crosses the platform on request and stops where the arm can work."""

    def __init__(self):
        super().__init__('approach_server')
        self.declare_parameter('cmd_vel_topic', APPROACH_CMD_VEL_TOPIC)
        self.declare_parameter('target_topic', '/perception/target')
        self.declare_parameter('colour_topic', '/mission/target_colour')
        self.declare_parameter('odom_topic', '/diff_drive_controller/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('status_topic', '/approach/status')
        self.declare_parameter('estimate_topic', '/approach/target')
        self.declare_parameter('handoff_x', SERVO_HANDOFF_X)
        self.declare_parameter('status_hz', STATUS_HZ)
        # No silent default colour: stopping in the wrong window is worse
        # than refusing to start, and the window differs by 6 mm across the
        # four targets.
        self.declare_parameter('target_colour', '')

        self._handoff_x = float(self.get_parameter('handoff_x').value)

        self.colour = None
        colour = (self.get_parameter('target_colour').value or '').strip()
        if colour in TARGET_COLOURS:
            self.colour = colour

        self._target = None        # (x, y, z) in base_footprint
        self._target_at = None     # monotonic arrival time of that fix
        self._odom = None
        self._imu = None
        self._lock = threading.Lock()
        self._abort = threading.Event()
        self._busy = False

        self.phase = 'idle'
        self.outcome = None
        self.travel = 0.0

        sensor_qos = QoSProfile(
            depth=5,
            reliability=(ReliabilityPolicy.BEST_EFFORT
                         if is_best_effort('/imu')
                         else ReliabilityPolicy.RELIABLE))

        self.create_subscription(
            PointStamped, self.get_parameter('target_topic').value,
            self._on_target, 10)
        self.create_subscription(
            String, self.get_parameter('colour_topic').value,
            self._on_colour, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, 10)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value,
            self._on_imu, sensor_qos)

        self._cmd_pub = self.create_publisher(
            TwistStamped, self.get_parameter('cmd_vel_topic').value, 10)
        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self._estimate_pub = self.create_publisher(
            PointStamped, self.get_parameter('estimate_topic').value, 10)

        self.create_service(Trigger, '/approach/run', self._on_run)
        self.create_service(Trigger, '/approach/stop', self._on_stop)
        self.create_timer(
            1.0 / float(self.get_parameter('status_hz').value),
            self._publish_status)

        self.get_logger().info(
            f'approach_server ready -> '
            f'{self.get_parameter("cmd_vel_topic").value}. Set /mission/mode '
            f'to "approach" or the arbiter will not forward this.')

    # ── inputs ───────────────────────────────────────────────────────────
    def _on_target(self, msg):
        self._target = (msg.point.x, msg.point.y, msg.point.z)
        self._target_at = time.monotonic()

    def _on_colour(self, msg):
        colour = (msg.data or '').strip().lower()
        if colour in TARGET_COLOURS and colour != self.colour:
            self.get_logger().info(f'target {self.colour} -> {colour}')
            self.colour = colour

    def _on_odom(self, msg):
        self._odom = msg

    def _on_imu(self, msg):
        self._imu = msg

    # ── plumbing ─────────────────────────────────────────────────────────
    def _publish(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self._cmd_pub.publish(msg)

    def _publish_status(self):
        target = self._target
        bearing = None if target is None else bearing_to(target[0], target[1])
        rng = None if target is None else math.hypot(target[0], target[1])
        self._status_pub.publish(String(data=format_status(
            phase=self.phase,
            tx=None if target is None else target[0],
            ty=None if target is None else target[1],
            range=rng, bearing=bearing,
            stop=None if self.colour is None else approach_stop_x(self.colour),
            travel=self.travel, colour=self.colour, outcome=self.outcome)))

    def _pose(self):
        """(x, y, yaw) from wheel odometry, or None before the first message."""
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return p.x, p.y, yaw

    def _pitch(self):
        """IMU pitch in radians, or None if the IMU has not been heard."""
        if self._imu is None:
            return None
        q = self._imu.orientation
        return math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))

    def _fresh_target(self):
        """Return the latest fix, or None once it is older than VISION_STALE."""
        if self._target is None or self._target_at is None:
            return None
        if time.monotonic() - self._target_at > VISION_STALE:
            return None
        return self._target

    def _reject(self, why):
        self.get_logger().error(why)
        return Trigger.Response(success=False, message=why)

    # ── services ─────────────────────────────────────────────────────────
    def _on_run(self, request, response):
        del request
        if self._busy:
            return self._reject(f'busy in phase {self.phase}')
        if self.colour is None:
            return self._reject(
                'no target colour — publish /mission/target_colour or start '
                'with -p target_colour:=<red|green|blue|yellow>')
        threading.Thread(target=self._run, daemon=True).start()
        response.success = True
        response.message = 'approach started; watch /approach/status'
        return response

    def _on_stop(self, request, response):
        del request
        self._abort.set()
        self._publish(0.0, 0.0)
        self.phase, self.outcome = 'idle', 'stopped'
        response.success = True
        response.message = 'stopping'
        return response

    # ── the run ──────────────────────────────────────────────────────────
    def _tick(self, linear, angular):
        """Command one cycle and sleep out the rest of the control period."""
        self._publish(linear, angular)
        time.sleep(1.0 / APPROACH_HZ)

    def _wait_for_inputs(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._odom is not None and self._imu is not None:
                return True
            time.sleep(0.1)
        return False

    def _run(self):
        with self._lock:
            self._busy = True
            self._abort.clear()
            self.outcome = None
            self.travel = 0.0
            try:
                self._run_phases()
            except Exception as exc:                      # noqa: BLE001
                # A dead worker looks exactly like a slow approach to the
                # sequencer, which is waiting on /approach/status.
                self.outcome = f'error: {exc}'
                self.get_logger().error(f'approach failed: {exc}')
            finally:
                self._publish(0.0, 0.0)
                self.phase = 'idle'
                self._busy = False
        self.get_logger().info(
            f'approach finished: {self.outcome} after {self.travel:.3f} m')

    def _run_phases(self):
        stop_x = approach_stop_x(self.colour)
        deadline = time.monotonic() + APPROACH_TIMEOUT
        if not self._wait_for_inputs():
            self.outcome = 'no odometry or imu'
            return

        start = self._pose()
        heading = start[2]

        # ── crest ────────────────────────────────────────────────────────
        self.phase = 'crest'
        level_since = None
        while not self._abort.is_set() and time.monotonic() < deadline:
            pose = self._pose()
            self.travel = math.hypot(pose[0] - start[0], pose[1] - start[1])
            pitch = self._pitch()
            linear, angular, level = crest_cmd(pitch, wrap(pose[2] - heading))
            if level:
                level_since = level_since or time.monotonic()
                if time.monotonic() - level_since >= CREST_SETTLE:
                    break
            else:
                level_since = None
            if self.travel >= CREST_MAX_TRAVEL:
                # Not fatal on its own: if the target is already in view the
                # servo can take it from here. Say so, because a crest that
                # never levelled means the IMU or the world changed.
                self.get_logger().warn(
                    f'crest ran {self.travel:.2f} m without levelling '
                    f'(pitch {pitch:+.3f}) — handing to the servo anyway')
                break
            self._tick(linear, angular)
        self._publish(0.0, 0.0)
        if self._abort.is_set():
            self.outcome = 'stopped'
            return

        # ── servo ────────────────────────────────────────────────────────
        self.phase = 'servo'
        acquire_deadline = time.monotonic() + SERVO_ACQUIRE_TIMEOUT
        while not self._abort.is_set() and time.monotonic() < deadline:
            target = self._fresh_target()
            if target is None:
                if time.monotonic() > acquire_deadline:
                    self.outcome = 'no target in view'
                    return
                self._tick(0.0, 0.0)
                continue
            acquire_deadline = time.monotonic() + SERVO_ACQUIRE_TIMEOUT
            linear, angular, done = servo_cmd(
                target[0], target[1], stop_x, self._handoff_x)
            if done:
                break
            self._tick(linear, angular)
        self._publish(0.0, 0.0)
        if self._abort.is_set():
            self.outcome = 'stopped'
            return

        # ── align ────────────────────────────────────────────────────────
        # Vision is deliberately still live here; that is what handoff_x
        # buys, and it is the only reason turning in place is worth doing.
        self.phase = 'align'
        align_deadline = time.monotonic() + ALIGN_TIMEOUT
        while not self._abort.is_set() and time.monotonic() < align_deadline:
            target = self._fresh_target()
            if target is None:
                self.outcome = 'lost the target during align'
                return
            linear, angular, done = align_cmd(bearing_to(target[0], target[1]))
            if done:
                break
            self._tick(linear, angular)
        else:
            self.outcome = 'stopped' if self._abort.is_set() else 'align timeout'
            self._publish(0.0, 0.0)
            return
        self._publish(0.0, 0.0)

        # ── creep ────────────────────────────────────────────────────────
        # Take the fix AFTER the align: the bearing is nulled now, so the
        # remaining range really is a straight line along base-x.
        time.sleep(0.4)                    # let a fresh frame land
        target = self._fresh_target()
        if target is None:
            self.outcome = 'lost the target before the creep'
            return
        distance = target[0] - stop_x
        if distance <= 0.0:
            self.get_logger().info(
                f'already inside the window ({target[0]:.4f} <= {stop_x:.4f})')
            distance = 0.0
        creep_start = self._pose()
        heading = creep_start[2]
        self.phase = 'creep'
        creep_deadline = time.monotonic() + CREEP_TIMEOUT
        travelled = 0.0
        while not self._abort.is_set() and time.monotonic() < creep_deadline:
            pose = self._pose()
            travelled = math.hypot(pose[0] - creep_start[0],
                                   pose[1] - creep_start[1])
            linear, angular, done = creep_cmd(
                travelled, distance, wrap(pose[2] - heading))
            if done:
                break
            self._tick(linear, angular)
        self._publish(0.0, 0.0)
        if self._abort.is_set():
            self.outcome = 'stopped'
            return
        self.travel = math.hypot(self._pose()[0] - start[0],
                                 self._pose()[1] - start[1])

        # ── report ───────────────────────────────────────────────────────
        # The estimate is dead-reckoned from the last fix, not re-measured:
        # perception cannot see the target from here, and a stale reading
        # republished as current is how a grasp aims at where the object
        # used to be. z comes from the constant — the object stands on the
        # same plane the robot does, and the blob centroid is not its grasp
        # band.
        estimate = PointStamped()
        estimate.header.stamp = self.get_clock().now().to_msg()
        estimate.header.frame_id = 'base_footprint'
        estimate.point.x = float(target[0] - travelled)
        estimate.point.y = float(target[1])
        estimate.point.z = float(TARGET_GRASP_Z)
        self._estimate_pub.publish(estimate)
        self.outcome = 'arrived'
        self.get_logger().info(
            f'arrived: target axis at base-x {estimate.point.x:.4f} '
            f'(window centre {stop_x:.4f}), y {estimate.point.y:+.4f}, '
            f'creep {travelled:.4f} of {distance:.4f} m')


def main(args=None):
    rclpy.init(args=args)
    node = ApproachServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy invalidates the context from its own SIGINT handler, so
        # Ctrl-C arrives as the latter rather than as KeyboardInterrupt.
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
