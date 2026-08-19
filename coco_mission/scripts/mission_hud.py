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
mission_hud — one readable answer to "what is the robot doing right now".

The mission already publishes plenty of state. Five separate nodes each
emit a space-separated ``key=value`` line at 2-5 Hz, and between them they
say everything a watcher needs. The problem is that reading them means
five ``ros2 topic echo`` terminals, and a screen recording of five
terminals communicates nothing. This node subscribes to all of them and
renders ONE block.

It aggregates. It does not measure. Every number it prints is copied from
a topic some other node published, and the moment that topic stops
arriving the field is marked stale rather than held at its last value —
a HUD that keeps showing the last known good number while the publisher
is dead is worse than no HUD, because it actively misleads.

Honesty rules, which are the whole point
----------------------------------------
Three of the fields a mission HUD "should" have have no measured source
in this repo yet. They are printed with an explicit placeholder naming
the milestone that will provide them, never with a plausible-looking
number:

  TERRAIN GRADE     ROBOT PITCH is the robot's attitude, not an estimate
                    of the surface it is standing on. Those are the same
                    number only on rigid contact with no suspension
                    travel, and only while the robot is not accelerating.
                    Pitch is shown, under its own name; a surface-grade
                    estimator is M2 and does not exist.

                    This distinction is not academic here. ROBOT PITCH
                    used to be read off ``/ramp/status``, and the ramp
                    driver samples pitch only while a segment is running.
                    Measured in C2-M1.5: the field held **-0.314 rad**
                    -- an 18 deg nose-up reading taken on the ramp, which
                    is also exactly the ramp's grade -- through the whole
                    platform approach and pick, while ``/imu`` had already
                    returned to 0.000. A grade estimator built on that
                    field would have validated perfectly on the ramp and
                    then reported 18 deg on flat ground forever. It now
                    comes from ``/imu``.
  EST. FRICTION     nothing estimates friction. M2.
  RECOVERY          nothing implements recovery. M5.

LOCALIZATION is the interesting one. ``/amcl_pose`` carries a real
covariance, so the sigmas are shown — they are measured. What is NOT
shown is a GOOD/DEGRADED verdict, because the threshold that separates
them has not been calibrated against a known-bad run. Picking 0.25 m
because it looks about right would be inventing the one number the whole
display hangs on. M5 measures it; until then this reports the signal and
withholds the judgement.

Topics
------
in   /mission/state            std_msgs/String   sequencer step label
in   /mission/mode             std_msgs/String   commanded arbiter mode
in   /mission/target_colour    std_msgs/String
in   /cmd_vel_arbiter/status   std_msgs/String   key=value
in   /perception/status        std_msgs/String   key=value
in   /approach/status          std_msgs/String   key=value
in   /ramp/status              std_msgs/String   key=value. Collected and
                               aged, but no row renders it since C2-M1.5
                               moved ROBOT PITCH to /imu. Kept subscribed
                               because the climb's cross-track lives here
                               and C2-M2 is the milestone that will want
                               it; it is NOT the pitch source.
in   /grasp/status             std_msgs/String   key=value
in   /amcl_pose                geometry_msgs/PoseWithCovarianceStamped
in   /plan                     nav_msgs/Path
in   /imu                      sensor_msgs/Imu, BEST_EFFORT. The source
                               for ROBOT PITCH — see the TERRAIN GRADE
                               note above for why it is not /ramp/status.
out  /mission/hud              std_msgs/String   (2 Hz, the rendered block)
out  /mission/goal             geometry_msgs/PoseStamped, the end of the
                               current global plan. Exists because
                               /goal_pose never publishes during an
                               autonomous run — see _publish_goal.
out  /mission/hud_overlay      rviz_2d_overlay_msgs/OverlayText, IF the
                               package is installed; silently skipped if
                               not, so this node never becomes a reason
                               the mission will not start.

Every topic name is a ROS parameter, so this node never needs a CLI remap.
"""

import math
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from nav_msgs.msg import Path

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from sensor_msgs.msg import Imu

from std_msgs.msg import String

# The overlay renderer is optional. `ros-jazzy-rviz-2d-overlay-plugins`
# is not a ROS core package and is not installed on every machine that
# might run this mission; the String output carries the same content and
# needs nothing. Import failure is a normal, expected state, not an error
# worth a traceback at startup.
try:
    from rviz_2d_overlay_msgs.msg import OverlayText
    HAVE_OVERLAY = True
except ImportError:            # pragma: no cover - depends on the machine
    OverlayText = None
    HAVE_OVERLAY = False

HUD_HZ = 2.0

# A field is stale this long after its last message ARRIVED. Deliberately
# generous: the slowest source here publishes at 2 Hz, so 2.0 s is four
# missed frames. This is a wall-clock question — "is that publisher still
# alive" is not answerable from sim time, which stops when the simulator
# does and would freeze every age at whatever it held. cmd_vel_arbiter
# made the same call for the same reason.
STALE_AFTER = 2.0

# Printed for a field whose source has never published at all, as
# distinct from one that published and went quiet (STALE). The
# distinction matters when debugging a mission that will not start: "the
# node is dead" and "the node is up but blocked" look identical if both
# render as blank.
NEVER = '--'
STALE = 'STALE'

# Fields with no measured source yet. The milestone is part of the text
# so that a recruiter watching the recording, or a future maintainer, is
# told which of these are missing on purpose.
NOT_MEASURED = {
    'grade': 'not yet measured (M2)',
    'friction': 'not yet measured (M2)',
    'recovery': 'not implemented (M5)',
}

# Fixed label column so the block does not jitter as values change width.
LABEL_WIDTH = 18


def parse_kv(line):
    """
    Split one ``key=value key=value`` status line into a dict.

    Every status publisher in this repo emits this format and guarantees
    no value contains a space, so a plain split is correct rather than
    merely convenient. Tokens without '=' are skipped instead of raising:
    this node must not be able to crash the display because some future
    publisher adds a bare word to its line.
    """
    fields = {}
    for token in (line or '').split():
        key, sep, value = token.partition('=')
        if sep and key:
            fields[key] = value
    return fields


def age_of(stamp, now):
    """Seconds since `stamp`, or None if it never arrived."""
    return None if stamp is None else now - stamp


def format_age(age):
    """
    Render one source's freshness.

    None -> NEVER, over the limit -> STALE, otherwise the age in seconds.
    """
    if age is None:
        return NEVER
    if age > STALE_AFTER:
        return f'{STALE} {age:.0f}s'
    return f'{age:.1f}s'


def is_live(age):
    """Report whether a source has published and is not stale."""
    return age is not None and age <= STALE_AFTER


def pose_sigmas(covariance):
    """
    Extract (sigma_x, sigma_y, sigma_yaw) from a 36-element covariance.

    ROS covariance is row-major 6x6 over (x, y, z, roll, pitch, yaw), so
    the variances sit on the diagonal at 0, 7 and 35. Returns None if the
    array is the wrong length or holds a negative variance — both are
    signs of a malformed publisher, and sqrt of a negative would raise
    inside a timer callback and take the node down.
    """
    if covariance is None or len(covariance) != 36:
        return None
    picked = (covariance[0], covariance[7], covariance[35])
    if any(v is None or v < 0.0 or math.isnan(v) for v in picked):
        return None
    return tuple(math.sqrt(v) for v in picked)


def format_localisation(sigmas, age):
    """
    Report the localisation signal WITHOUT a health verdict.

    The sigmas are measured, so they are shown. GOOD/DEGRADED is not
    shown, because the threshold separating them has never been measured
    against a known-bad run — see the module docstring. M5 calibrates it.

    Unlike every other field here, age is NOT treated as staleness. AMCL
    publishes on update, and nav2_params sets update_min_d 0.25 m /
    update_min_a 0.2 rad, so a stationary robot produces no /amcl_pose
    at all — by design, not by failure. The first live run of this HUD
    showed 'STALE 17s' with the sigmas hidden while the robot sat
    perfectly localised at its start pose, which reads as exactly the
    fault M5 is supposed to detect. So the last known pose stays on
    screen and the age is stated beside it instead of replacing it.
    """
    if age is None:
        return NEVER
    if sigmas is None:
        return 'malformed covariance'
    sx, sy, syaw = sigmas
    body = (f'sigma x {sx:.3f} m  y {sy:.3f} m  '
            f'yaw {math.degrees(syaw):.1f} deg')
    return body if is_live(age) else f'{body}  ({age:.0f}s since update)'


def format_distance(approach_fields, perception_fields):
    """
    Distance to the target, preferring whichever source is authoritative.

    During the approach the servo's own `range` is the number the stop
    decision is actually made on, so it wins. Before the approach starts
    the only range available is perception's, which is the depth-camera
    reading. Naming which one is being shown matters: they are measured
    by different sensors through different code paths and disagreeing by
    a few mm is normal.
    """
    for fields, source in ((approach_fields, 'approach'),
                           (perception_fields, 'vision')):
        value = fields.get('range')
        if value not in (None, '', NEVER):
            return f'{value} m ({source})'
    return NEVER


def format_goal(goal):
    """Render the current Nav2 goal, which is the last pose of /plan."""
    if goal is None:
        return NEVER
    return f'x {goal[0]:+.2f}  y {goal[1]:+.2f}  (map)'


def quat_pitch(x, y, z, w):
    """
    Pitch in radians from an orientation quaternion.

    The reference definition is ``coco_rl.ramp_env.quat_to_rp``; this is
    the same arithmetic, kept local so the HUD does not import the RL
    stack (gymnasium, numpy, the env) to print one number. `test_hud_pitch
    _sign_convention` pins the result against hand-computed values, which
    is what actually stops the two drifting apart — a shared import would
    not have caught a sign error anyway, only a rename.

    REP-103 body frame: x forward, y left, z up. Pitch is rotation about
    +y, so **nose-up is NEGATIVE**. On the 18 deg wedge the robot reads
    about -0.314 rad going up and +0.314 rad coming down.

    asin's argument is clamped: a quaternion that is fractionally
    un-normalised after transport can push it past 1.0, and a ValueError
    raised inside a timer callback takes the node down.
    """
    return math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))


def format_pitch(pitch, age):
    """
    Render body pitch with its freshness.

    Stale is shown rather than the last value, because a held attitude is
    the specific failure this field was found to have: see the TERRAIN
    GRADE note in the module docstring. Degrees are printed alongside
    radians — every geometry number in this project's docs is in degrees
    (the wedge is "18 deg"), and the reader should not have to convert to
    see that a reading matches the ramp.
    """
    if pitch is None or not is_live(age):
        return format_age(age)
    return f'{pitch:+.3f} rad  ({math.degrees(pitch):+.1f} deg)'


def format_mission_state(line):
    """
    Render ``/mission/state``.

    C2-M1 published a free-text step label there ('2. RL climb'), because
    that was what the blocking sequencer had. C2-M3's executive publishes
    a key=value line like every other status topic in this project, so
    the STATE row shows the state and, when a state can time out, how far
    into its budget it is.

    Both shapes render: a line with no ``state=`` field is passed through
    unchanged, which is what keeps traverse_demo.py readable on the same
    HUD. It has to stay readable — traverse_demo is the harness the
    M4/M5/M6 numbers were measured with.
    """
    fields = parse_kv(line)
    state = fields.get('state')
    if not state:
        return line
    timeout = fields.get('timeout')
    elapsed = fields.get('elapsed')
    if timeout and timeout != '--' and elapsed:
        return f'{state}   ({elapsed}s / {timeout}s)'
    return state


def format_recovery(line, fallback):
    """
    Render the RECOVERY row from the executive's state line.

    C2-M1 shipped this row reading 'not implemented (M5)', which was
    true: nothing recorded why a step failed or whether it would be
    retried. The executive does, so the row now has a source — for the
    retry bookkeeping. **Localization recovery is still C2-M5**, and
    nothing here claims otherwise.
    """
    fields = parse_kv(line)
    if not fields.get('state'):
        return fallback
    reason = fields.get('reason') or '--'
    attempt = fields.get('attempt', '--')
    retries = fields.get('retries', '--')
    budget = f'attempt {attempt}, {retries} retries allowed'
    if reason == '--':
        return f'none   ({budget})'
    return f'{reason}   ({budget})'


def render(state):
    """
    Build the HUD block.

    Pure: takes a dict of already-resolved strings and returns text. All
    of the "is this stale" and "is this measured" logic happens before
    here, so this function is trivially testable and the interesting
    decisions are each tested on their own.
    """
    rows = [
        ('MISSION', state['mission']),
        ('STATE', state['mission_state']),
        ('ACTIVE CONTROLLER', state['controller']),
        ('LOCALIZATION', state['localisation']),
        ('TARGET', state['target']),
        ('DISTANCE TO TARGET', state['distance']),
        ('CURRENT GOAL', state['goal']),
        ('ROBOT PITCH', state['pitch']),
        ('TERRAIN GRADE', NOT_MEASURED['grade']),
        ('EST. FRICTION', NOT_MEASURED['friction']),
        ('RECOVERY', state['recovery']),
    ]
    width = LABEL_WIDTH
    body = '\n'.join(f'{label:<{width}} {value}' for label, value in rows)
    return f'COCO 2.0 — MISSION HUD\n{"-" * 46}\n{body}'


class MissionHud(Node):
    """Aggregates the mission's status topics into one rendered block."""

    def __init__(self):
        """Subscribe every status source and start the render timer."""
        super().__init__('mission_hud')

        self.declare_parameter('hud_topic', '/mission/hud')
        self.declare_parameter('overlay_topic', '/mission/hud_overlay')
        self.declare_parameter('goal_topic', '/mission/goal')
        self.declare_parameter('hud_hz', HUD_HZ)

        # Ages are measured against a steady clock, never the ROS clock.
        # See STALE_AFTER.
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._wall = time.monotonic

        # source name -> (last raw string, monotonic arrival time)
        self._lines = {}
        self._stamps = {}

        self._sigmas = None
        self._goal = None
        self._goal_pose = None
        self._goal_frame = ''
        self._pitch = None

        for name, topic in (
            ('state', '/mission/state'),
            ('mode', '/mission/mode'),
            ('colour', '/mission/target_colour'),
            ('arbiter', '/cmd_vel_arbiter/status'),
            ('perception', '/perception/status'),
            ('approach', '/approach/status'),
            ('ramp', '/ramp/status'),
            ('grasp', '/grasp/status'),
        ):
            self.create_subscription(
                String, topic,
                (lambda msg, key=name: self._on_string(key, msg)), 10)

        # AMCL latches its pose with TRANSIENT_LOCAL durability. A
        # VOLATILE subscriber matches it, but misses the pose already
        # published before this node started, which is exactly the case
        # when the HUD is launched after the stack is up.
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, latched)
        self.create_subscription(Path, '/plan', self._on_plan, 10)

        # BEST_EFFORT for /imu. The gz bridge currently advertises it
        # RELIABLE, which a best-effort subscriber still matches; the
        # reverse does not hold, and a RELIABLE subscriber on a sensor
        # that ever goes best-effort matches nothing and reports NEVER
        # with no error anywhere. Sensor data takes the lenient side.
        sensor = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, '/imu', self._on_imu, sensor)

        self._hud_pub = self.create_publisher(
            String, self.get_parameter('hud_topic').value, 10)
        self._goal_pub = self.create_publisher(
            PoseStamped, self.get_parameter('goal_topic').value, 10)

        self._overlay_pub = None
        if HAVE_OVERLAY:
            self._overlay_pub = self.create_publisher(
                OverlayText, self.get_parameter('overlay_topic').value, 10)
        else:
            self.get_logger().info(
                'rviz_2d_overlay_msgs not found; publishing '
                f'{self.get_parameter("hud_topic").value} only. For the '
                'in-RViz overlay: '
                'sudo apt install ros-jazzy-rviz-2d-overlay-plugins')

        hz = float(self.get_parameter('hud_hz').value)
        # Steady clock again: the HUD must keep updating (and keep
        # marking things stale) even if /clock stops, which is precisely
        # the failure it is most useful for diagnosing.
        self.create_timer(1.0 / hz, self._tick, clock=self._steady)

    def _on_string(self, key, msg):
        self._lines[key] = msg.data
        self._stamps[key] = self._wall()

    def _on_amcl(self, msg):
        self._sigmas = pose_sigmas(msg.pose.covariance)
        self._stamps['amcl'] = self._wall()

    def _on_imu(self, msg):
        q = msg.orientation
        self._pitch = quat_pitch(q.x, q.y, q.z, q.w)
        self._stamps['imu'] = self._wall()

    def _on_plan(self, msg):
        if msg.poses:
            last = msg.poses[-1].pose.position
            self._goal = (last.x, last.y)
            # Keep the whole pose, not just x/y, so /mission/goal carries
            # the goal heading too.
            self._goal_pose = msg.poses[-1]
            self._goal_frame = msg.header.frame_id
        self._stamps['plan'] = self._wall()

    def _publish_goal(self):
        """
        Republish the current goal as a pose RViz can actually display.

        Measured on the first live mission: /goal_pose is ADVERTISED but
        never publishes during an autonomous run. Nothing is wrong with
        it — traverse_demo drives Nav2 through the NavigateToPose ACTION,
        and /goal_pose is only ever written by RViz's own "2D Goal Pose"
        tool. An RViz display pointed at it therefore sits dead for the
        entire mission while the robot is very obviously navigating
        somewhere, which is worse than having no goal display at all.

        The end of the current global plan is where the robot is actually
        being driven, so that is what is published here. It is DERIVED,
        not the raw action goal: the planner may snap the goal to a free
        cell, so this can differ from the commanded pose by up to a
        costmap cell. The action goal itself is not on any topic.
        """
        if self._goal_pose is None:
            return
        msg = PoseStamped()
        msg.header.frame_id = self._goal_frame or 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = self._goal_pose.pose
        self._goal_pub.publish(msg)

    def _age(self, key, now):
        return age_of(self._stamps.get(key), now)

    def _fields(self, key):
        """Parse a source's latest key=value line into a dict."""
        return parse_kv(self._lines.get(key))

    def _tick(self):
        now = self._wall()

        arbiter = self._fields('arbiter')
        approach = self._fields('approach')
        perception = self._fields('perception')
        grasp = self._fields('grasp')

        arbiter_age = self._age('arbiter', now)
        # The arbiter is the only node that knows which source is
        # actually reaching the wheels. /mission/mode is what the
        # sequencer ASKED for, and the two differ whenever the requested
        # source has gone stale — which is the single most useful thing
        # this HUD can show, so both are printed.
        if is_live(arbiter_age):
            active = arbiter.get('active', NEVER)
            mode = arbiter.get('mode', NEVER)
            controller = f'{active}   (mode {mode})'
        else:
            controller = format_age(arbiter_age)

        state_age = self._age('state', now)
        state_line = self._lines.get('state', '')
        live_state = is_live(state_age)
        mission_state = (format_mission_state(state_line) if live_state
                         else format_age(state_age))
        recovery = (format_recovery(state_line, NOT_MEASURED['recovery'])
                    if live_state else NOT_MEASURED['recovery'])

        colour_age = self._age('colour', now)
        colour = self._lines.get('colour') if is_live(colour_age) else None
        # Perception reports the colour it is actually selecting on. If
        # that disagrees with the requested colour the mission is about
        # to fetch the wrong object, so show both rather than one.
        selected = perception.get('sel')
        if colour and selected and selected != colour:
            target = f'{colour}  (!! vision selecting {selected})'
        else:
            target = colour or selected or NEVER

        grasp_phase = grasp.get('phase')

        mission = 'fetch'
        if grasp_phase not in (None, '', NEVER, 'idle'):
            mission = f'fetch  [grasp: {grasp_phase}]'

        block = render({
            'mission': mission,
            'mission_state': mission_state,
            'recovery': recovery,
            'controller': controller,
            'localisation': format_localisation(
                self._sigmas, self._age('amcl', now)),
            'target': target,
            'distance': format_distance(approach, perception),
            'goal': format_goal(self._goal),
            'pitch': format_pitch(self._pitch, self._age('imu', now)),
        })

        self._hud_pub.publish(String(data=block))
        self._publish_overlay(block)
        self._publish_goal()

    def _publish_overlay(self, block):
        """
        Push the same block to RViz, if the overlay plugin is installed.

        Fields are set through hasattr rather than assigned directly.
        rviz_2d_overlay_msgs has changed its message definition between
        releases, and this node must not be the reason a mission fails to
        launch on a machine with a slightly different version of an
        OPTIONAL dependency.
        """
        if self._overlay_pub is None:
            return
        msg = OverlayText()
        for field, value in (
            ('text', block),
            ('width', 460),
            ('height', 260),
            ('horizontal_distance', 20),
            ('vertical_distance', 20),
            ('horizontal_alignment', 0),   # left
            ('vertical_alignment', 3),     # top
            ('text_size', 12.0),
        ):
            if hasattr(msg, field):
                setattr(msg, field, value)
        for field, rgba in (('fg_color', (0.35, 0.85, 0.95, 1.0)),
                            ('bg_color', (0.05, 0.06, 0.08, 0.72))):
            colour = getattr(msg, field, None)
            if colour is not None:
                colour.r, colour.g, colour.b, colour.a = rgba
        self._overlay_pub.publish(msg)


def main():
    """Run the HUD until shutdown."""
    rclpy.init()
    node = MissionHud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
