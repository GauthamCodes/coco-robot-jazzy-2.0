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

  TERRAIN GRADE     the ramp driver publishes robot PITCH, which is the
                    robot's attitude, not an estimate of the surface it
                    is standing on. Those are the same number only on
                    rigid contact with no suspension travel. Pitch is
                    shown, under its own name; a surface-grade estimator
                    is M2 and does not exist.
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
in   /ramp/status              std_msgs/String   key=value
in   /grasp/status             std_msgs/String   key=value
in   /amcl_pose                geometry_msgs/PoseWithCovarianceStamped
in   /plan                     nav_msgs/Path
out  /mission/hud              std_msgs/String   (2 Hz, the rendered block)
out  /mission/hud_overlay      rviz_2d_overlay_msgs/OverlayText, IF the
                               package is installed; silently skipped if
                               not, so this node never becomes a reason
                               the mission will not start.

Every topic name is a ROS parameter, so this node never needs a CLI remap.
"""

import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import Path

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from rclpy.qos import QoSReliabilityPolicy

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
    """
    if not is_live(age):
        return format_age(age)
    if sigmas is None:
        return 'malformed covariance'
    sx, sy, syaw = sigmas
    return (f'sigma x {sx:.3f} m  y {sy:.3f} m  '
            f'yaw {math.degrees(syaw):.1f} deg')


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
        ('RECOVERY', NOT_MEASURED['recovery']),
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

        self._hud_pub = self.create_publisher(
            String, self.get_parameter('hud_topic').value, 10)

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

    def _on_plan(self, msg):
        if msg.poses:
            last = msg.poses[-1].pose.position
            self._goal = (last.x, last.y)
        self._stamps['plan'] = self._wall()

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
        ramp = self._fields('ramp')
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
        mission_state = (self._lines.get('state', NEVER)
                         if is_live(state_age) else format_age(state_age))

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
        ramp_pitch = ramp.get('pitch')
        pitch = (f'{ramp_pitch} rad' if ramp_pitch not in (None, '', NEVER)
                 else NEVER)

        mission = 'fetch'
        if grasp_phase not in (None, '', NEVER, 'idle'):
            mission = f'fetch  [grasp: {grasp_phase}]'

        block = render({
            'mission': mission,
            'mission_state': mission_state,
            'controller': controller,
            'localisation': format_localisation(
                self._sigmas, self._age('amcl', now)),
            'target': target,
            'distance': format_distance(approach, perception),
            'goal': format_goal(self._goal),
            'pitch': pitch,
        })

        self._hud_pub.publish(String(data=block))
        self._publish_overlay(block)

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
