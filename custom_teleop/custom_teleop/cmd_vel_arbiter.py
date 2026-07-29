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
cmd_vel_arbiter — the sole publisher of /diff_drive_controller/cmd_vel.

Before this node, four publishers wrote to the wheel topic with nothing
mediating them: the keyboard teleop, the Nav2 relay, the web panel's
joystick and the RL ramp environment. Two 10 Hz sources do not "override"
each other — they interleave roughly 50/50 and the robot tracks the
average, so grabbing the joystick to stop a running policy produced a
robot moving at half speed rather than a robot stopping. That is a safety
defect before it is a missing feature.

This node subscribes to one topic per source, picks exactly one, and is
the only thing that talks to the controller.

Selection
---------
- ``/mission/mode`` (std_msgs/String) latches which autonomous source is
  eligible: ``nav``, ``rl``, ``approach``, ``teleop`` or ``idle``.
  ``auto``, ``autonomous`` and ``nav2`` are accepted as aliases for
  ``nav``.
- **Teleop always preempts.** While ``/cmd_vel_teleop`` is fresh it wins
  regardless of mode, including in ``idle``. A human reaching for the
  stick does not have to negotiate with the state machine first.
- Otherwise the mode-selected source is forwarded, if it is fresh.
- If nothing eligible is fresh, the arbiter publishes zeros (see below).

Freshness is judged by ARRIVAL time, never by ``header.stamp``: the web
panel publishes ``stamp: {sec: 0, nanosec: 0}`` (index.html), so a
stamp-based timeout would treat every joystick message as infinitely
stale and the phone would never drive the robot.

Arrival times come from ``time.monotonic()`` and the watchdog runs on a
STEADY clock rather than the node clock. Under ``use_sim_time`` a paused
or dead simulator freezes the ROS clock, which would freeze the watchdog
and latch the last command forever — the exact case the watchdog exists
to catch. "Is that publisher still alive" is a wall-clock question.

Stopping
--------
When no eligible source is fresh the arbiter publishes zeros for
``zero_hold_seconds`` after the last forwarded command, then goes quiet.
The hold is deliberately longer than the controller's own
``cmd_vel_timeout`` (0.5 s, coco_controllers.yaml) so the stop is an
explicit command rather than a watchdog halt. Going quiet afterwards
matters: an idle arbiter must not spray zeros at 20 Hz over the
diagnostic scripts (``climb_check.py``, ``map_drive.py``) that still
publish to the controller directly when run standalone.

Every outgoing message is re-stamped with the current ROS time. Jazzy's
diff_drive_controller ages commands from ``header.stamp`` and drops any
message older than ``cmd_vel_timeout``, so forwarding a held command with
its original stamp would silently stop the robot.

Topics
------
in   /cmd_vel_teleop  geometry_msgs/TwistStamped
in   /cmd_vel_nav     geometry_msgs/TwistStamped
in   /cmd_vel_rl      geometry_msgs/TwistStamped
in   /cmd_vel_approach geometry_msgs/TwistStamped
in   /mission/mode    std_msgs/String
out  /diff_drive_controller/cmd_vel  geometry_msgs/TwistStamped
out  /cmd_vel_arbiter/status         std_msgs/String  (2 Hz)

Every topic name is a ROS parameter, so this node never needs a CLI remap.
"""

import time

from geometry_msgs.msg import TwistStamped

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

# The autonomous sources, and the mode names that select them. 'teleop' is
# in both lists: it is selectable as a mode AND preempts from any mode.
#
# 'approach' is the mission's vision servo across the crest platform
# (custom_teleop.approach_server). It gets its own source rather than
# borrowing /cmd_vel_rl for one reason worth the five lines: with two
# publishers on one input, /cmd_vel_arbiter/status can no longer say which
# controller is driving, and "the robot moved but not the way that
# controller intended" is the single hardest thing to diagnose in this
# stack.
SOURCES = ('teleop', 'nav', 'rl', 'approach')
MODES = ('idle', 'teleop', 'nav', 'rl', 'approach')

# Modes that select an autonomous source. 'teleop' is excluded on purpose:
# it preempts from any mode and is handled ahead of this.
AUTONOMOUS_MODES = ('nav', 'rl', 'approach')

# The web panel has said "auto" since before there was a mission state
# machine, and 'stop' reads better than 'idle' on a button.
MODE_ALIASES = {
    '': 'idle',
    'auto': 'nav',
    'autonomous': 'nav',
    'nav2': 'nav',
    'manual': 'teleop',
    'ramp': 'rl',
    'stop': 'idle',
    'none': 'idle',
}

# A source is stale this long after its last message ARRIVED. 0.3 s is
# three missed frames from a 10 Hz publisher — long enough to ride out a
# dropped websocket frame, short enough to be well inside the
# controller's own 0.5 s timeout.
SOURCE_TIMEOUT = 0.3

# Output rate floor. Messages are forwarded the instant they arrive, so
# this only fills gaps left by a slow source; it is not the latency of a
# preemption.
WATCHDOG_HZ = 20.0

# Keep commanding zero this long after the last forwarded command, then
# fall silent. Longer than the controller's 0.5 s cmd_vel_timeout.
ZERO_HOLD_SECONDS = 1.0

STATUS_HZ = 2.0


def normalise_mode(raw):
    """
    Fold a /mission/mode string onto one of MODES.

    Returns None for anything unrecognised, so the caller can warn and
    keep the previous mode rather than silently dropping into a state the
    operator did not ask for.
    """
    mode = (raw or '').strip().lower()
    mode = MODE_ALIASES.get(mode, mode)
    return mode if mode in MODES else None


def select_source(mode, ages, timeout=SOURCE_TIMEOUT):
    """
    Pick the source to forward, or None to stop the robot.

    `ages` maps source name -> seconds since that source's last message
    arrived, or None if it has never published. Ages rather than absolute
    timestamps keep this a pure function, which is the whole of the
    arbitration policy and is tested without a ROS graph.
    """
    def fresh(name):
        age = ages.get(name)
        return age is not None and age <= timeout

    # Teleop outranks the mode, not just the other sources. An operator
    # grabbing the stick during an RL climb must not have to switch mode
    # first, and must win in 'idle' too.
    if fresh('teleop'):
        return 'teleop'
    if mode in AUTONOMOUS_MODES and fresh(mode):
        return mode
    # mode == 'teleop' with stale teleop, or 'idle': nothing drives.
    # 'idle' is also what the mission asserts while the arm is working:
    # the wheels must be still for a grasp, and teleop must still be able
    # to preempt, which is exactly what 'idle' already means.
    return None


def format_status(mode, active, ages):
    """
    Build a one-line arbiter state for /cmd_vel_arbiter/status.

    Space-separated key=value so the browser can split it without a JSON
    parser and a human can read it in `ros2 topic echo`.
    """
    parts = [f'mode={mode}', f'active={active or "none"}']
    for name in SOURCES:
        age = ages.get(name)
        parts.append(f'{name}={"--" if age is None else f"{age:.2f}"}')
    return ' '.join(parts)


class CmdVelArbiter(Node):
    """Selects one velocity source and is the only publisher to the wheels."""

    def __init__(self):
        super().__init__('cmd_vel_arbiter')

        self.declare_parameter('teleop_topic', '/cmd_vel_teleop')
        self.declare_parameter('nav_topic', '/cmd_vel_nav')
        self.declare_parameter('rl_topic', '/cmd_vel_rl')
        self.declare_parameter('approach_topic', '/cmd_vel_approach')
        self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel')
        self.declare_parameter('mode_topic', '/mission/mode')
        self.declare_parameter('status_topic', '/cmd_vel_arbiter/status')
        self.declare_parameter('source_timeout', SOURCE_TIMEOUT)
        self.declare_parameter('watchdog_hz', WATCHDOG_HZ)
        self.declare_parameter('zero_hold_seconds', ZERO_HOLD_SECONDS)
        self.declare_parameter('status_hz', STATUS_HZ)
        # 'idle' is the safe default: until something asserts a mode, only
        # teleop can move the robot. A standalone Nav2 demo with no panel
        # running should start this node with initial_mode:=nav.
        self.declare_parameter('initial_mode', 'idle')

        self.source_timeout = float(self.get_parameter('source_timeout').value)
        self.zero_hold = float(self.get_parameter('zero_hold_seconds').value)
        watchdog_hz = float(self.get_parameter('watchdog_hz').value)
        status_hz = float(self.get_parameter('status_hz').value)

        self.mode = normalise_mode(
            self.get_parameter('initial_mode').value) or 'idle'

        self._twist = {name: None for name in SOURCES}
        self._arrival = {name: None for name in SOURCES}
        self._last_emit = None       # monotonic time of the last publish
        self._last_forward = None    # monotonic time of the last real command
        self._active = None
        self._unknown_mode = None
        self._warned_shared_output = False

        # RELIABLE + VOLATILE (rclpy's default). Deliberately not
        # transient-local on the mode topic: the vendored roslib 1.4.1 in
        # coco_web silently drops a `qos` key, so a durability the browser
        # cannot express would just fail to match.
        self._pub = self.create_publisher(
            TwistStamped, self.get_parameter('output_topic').value, 10)
        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)

        for name, param in (('teleop', 'teleop_topic'),
                            ('nav', 'nav_topic'),
                            ('rl', 'rl_topic'),
                            ('approach', 'approach_topic')):
            self.create_subscription(
                TwistStamped, self.get_parameter(param).value,
                lambda msg, src=name: self._on_source(src, msg), 10)
        self.create_subscription(
            String, self.get_parameter('mode_topic').value, self._on_mode, 10)

        # Steady clock: a paused sim must not freeze the watchdog.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / watchdog_hz, self._watchdog, clock=steady)
        self.create_timer(1.0 / status_hz, self._publish_status, clock=steady)

        self.get_logger().info(
            f'Arbitrating {self.get_parameter("teleop_topic").value} (preempts), '
            f'{self.get_parameter("nav_topic").value}, '
            f'{self.get_parameter("rl_topic").value}, '
            f'{self.get_parameter("approach_topic").value} '
            f'-> {self.get_parameter("output_topic").value}')
        self.get_logger().info(
            f'mode={self.mode} (from '
            f'{self.get_parameter("mode_topic").value}); source timeout '
            f'{self.source_timeout:.2f}s')

    # ── inputs ───────────────────────────────────────────────────────────────
    def _on_source(self, name, msg):
        self._arrival[name] = time.monotonic()
        self._twist[name] = msg.twist
        # Forward immediately so a preemption costs nothing but transport;
        # the watchdog only fills gaps between a slow source's messages.
        if self._select() == name:
            self._emit(msg.twist)

    def _on_mode(self, msg):
        mode = normalise_mode(msg.data)
        if mode is None:
            if msg.data != self._unknown_mode:
                self._unknown_mode = msg.data
                self.get_logger().warn(
                    f'ignoring unknown /mission/mode {msg.data!r} — '
                    f'staying in {self.mode!r}. Known: {", ".join(MODES)}')
            return
        self._unknown_mode = None
        if mode != self.mode:
            self.get_logger().info(f'mode {self.mode} -> {mode}')
            self.mode = mode

    # ── arbitration ──────────────────────────────────────────────────────────
    def _ages(self, now=None):
        now = time.monotonic() if now is None else now
        return {name: (None if t is None else now - t)
                for name, t in self._arrival.items()}

    def _select(self, now=None):
        return select_source(self.mode, self._ages(now), self.source_timeout)

    def _emit(self, twist):
        msg = TwistStamped()
        # Re-stamp: Jazzy's diff_drive_controller ages the command from
        # header.stamp and drops anything older than cmd_vel_timeout, so a
        # held command must not carry the stamp it arrived with.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist = twist
        self._pub.publish(msg)
        self._last_emit = time.monotonic()

    def _watchdog(self):
        now = time.monotonic()
        period = 1.0 / float(self.get_parameter('watchdog_hz').value)
        active = self._select(now)

        if active != self._active:
            self.get_logger().info(
                f'source {self._active or "none"} -> {active or "none"} '
                f'(mode={self.mode})')
            self._active = active

        if active is not None:
            self._last_forward = now
            # Already forwarded on arrival; only fill a gap left by a
            # source slower than watchdog_hz.
            if self._last_emit is None or now - self._last_emit >= period:
                self._emit(self._twist[active])
            return

        # Nothing eligible. Command a definite stop for zero_hold_seconds
        # after the last real command, then stay off the topic entirely.
        if self._last_forward is None:
            return
        if now - self._last_forward > self.zero_hold:
            return
        if self._last_emit is None or now - self._last_emit >= period:
            self._emit(TwistStamped().twist)

    def _publish_status(self):
        self._check_sole_publisher()
        self._status_pub.publish(String(
            data=format_status(self.mode, self._active, self._ages())))

    def _check_sole_publisher(self):
        # This node exists because four publishers shared the wheel topic
        # and interleaved instead of overriding. If that happens again --
        # typically `web.launch.py arbiter:=true` paired with
        # `nav.launch.py arbiter:=false` -- say so rather than let the
        # robot quietly track the average of two commands again.
        topic = self.get_parameter('output_topic').value
        if self.count_publishers(topic) > 1:
            if not self._warned_shared_output:
                self._warned_shared_output = True
                self.get_logger().warn(
                    f'another node is publishing {topic}. Arbitration is '
                    f'defeated: commands will interleave, not override. '
                    f'Point that source at an arbiter input instead.')
        else:
            self._warned_shared_output = False


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelArbiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy's own SIGINT handler invalidates the context before Python
        # sees the signal, so Ctrl-C arrives as ExternalShutdownException.
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
