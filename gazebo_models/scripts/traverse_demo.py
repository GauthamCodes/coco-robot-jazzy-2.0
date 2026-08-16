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
traverse_demo.py
================
The fetch mission, end to end: there -> up -> over -> pick -> down ->
home -> put it down.

Nav2 drives the flat ground and the RL policy drives the ramp, because
the split is forced by geometry: the lidar plane sits at 0.2135 m and an
18 degree slope intersects it at world x=1.657, so from the flat the ramp
scans as a solid wall. Nav2 will not plan onto its own costmap obstacle,
and after M3 `allow_unknown: false` stops it routing through the unmapped
space behind it either.

This is the sequencer's skeleton. It owns `/mission/mode` — nothing else
should publish it, because cmd_vel_arbiter latches the last value and two
publishers would fight. The arbiter is what actually enforces the
handoff: only the selected source reaches the wheels, and teleop preempts
either of them at any time.

  1.  mode=nav       Nav2 to the pre-ramp pose (0.5, lane_y), flat ground
  2.  mode=rl        /ramp/climb    — PPO policy up the slope, lane held
  2b. (gate)         vision must confirm the colour is in front
  2c. mode=idle      /grasp/stow    — arm up, before driving at the target
  3.  mode=approach  /approach/run  — cross the platform under vision
  4.  mode=idle      /grasp/pick    — hover, weld, lift, carry at 'up'
  5.  mode=rl        /ramp/descend  — scripted heading-hold, carrying
  6.  mode=nav       Nav2 home
  7.  mode=idle      /grasp/place   — set it down, release, arm home

Requires: the sim with traverse:=true, and everything mission.launch.py
starts (Nav2 with arbiter:=true, the arbiter, perception, move_group,
ramp_driver with a policy, approach_server, grasp_server).

`--no-grasp` runs 1, 2, 2b (reporting only), 5, 6 — the M4/M5 traverse,
kept runnable so those measurements stay reproducible.

The lane comes from the CHOSEN COLOUR, not from a bare number: the four
targets sit in lanes 0.5 m apart and coco_config.robot.TARGETS is the one
table that says which colour is in which. It has to be a lookup rather
than a look: the crest edge occludes the whole platform from every point
on the flat ground, so no pre-ramp census of the objects is possible.

The colour comes from the phone on `/mission/target_colour`, or from
--colour for a headless run. There is no silent default: fetching a
different object than the one that was asked for, successfully, is worse
than not starting.

Usage:
  ros2 run gazebo_models traverse_demo.py --colour blue
  ros2 run gazebo_models traverse_demo.py            # wait for the phone
  ros2 run gazebo_models traverse_demo.py --colour blue --lane 0.0
"""
import argparse
import math
import sys
import time

from action_msgs.msg import GoalStatus
from coco_config.robot import TARGET_COLOURS, lane_for_colour
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

# The map frame is anchored at the spawn pose (see docs/RESULTS.md).
WORLD_TO_MAP_X = 2.0

PRE_RAMP_X = 0.5      # flat ground west of the ramp foot (x=1.0), and the
                      # only clear spot in lane +0.75: the Zone A gate cubes
                      # block x -1.35..-0.85 and the cylinder x -0.4..0.0
HOME = (-2.0, 0.0)


class TraverseDemo(Node):
    """Sequences Nav2 and the ramp driver through one full traverse."""

    def __init__(self):
        super().__init__('traverse_demo')
        self.pose = None
        self.ramp_status = ''
        self.vision_status = ''
        self.colour = None
        self.create_subscription(
            Odometry, '/model/coco/odometry', self._odom_cb, 10)
        self.create_subscription(
            String, '/ramp/status', self._ramp_cb, 10)
        self.create_subscription(
            String, '/perception/status', self._vision_cb, 10)
        self.approach_status = ''
        self.grasp_status = ''
        self.create_subscription(
            String, '/approach/status', self._approach_cb, 10)
        self.create_subscription(
            String, '/grasp/status', self._grasp_cb, 10)
        # The panel asserts the operator's choice at 2 Hz. This node owns
        # /mission/mode but NOT this topic — it is an input, and the CLI
        # is the alternative source for a headless run.
        self.create_subscription(
            String, '/mission/target_colour', self._colour_cb, 10)
        # RELIABLE + VOLATILE, matching the panel and the arbiter. Asserted
        # repeatedly rather than latched: the vendored roslib in coco_web
        # cannot express transient-local, so the whole system agreed to
        # re-assert instead.
        self._mode_pub = self.create_publisher(String, '/mission/mode', 10)
        # Only used when --colour supplied the choice. With the panel up it
        # is the publisher and this stays silent; two publishers asserting
        # the same topic at 2 Hz is how the mode topic would have gone
        # wrong, and there is no reason to repeat it here.
        self._colour_pub = self.create_publisher(
            String, '/mission/target_colour', 10)
        # /mission/state is the step label this sequencer is currently
        # executing, and nothing else publishes it. It exists because the
        # step labels were already here and already correct -- they were
        # just going to stdout, where a HUD cannot reach them and a screen
        # recording shows a wall of scrolling text instead of a state.
        #
        # This is NOT the mission executive that M3 asks for. It reports
        # the state of a blocking script; it does not give any state an
        # entry condition, a timeout or a recovery. Publishing the label
        # is the part that can be done honestly today, and it is what the
        # real executive will replace rather than something it duplicates.
        #
        # Asserted at 2 Hz on a timer, not once on entry: a HUD started
        # mid-mission would otherwise show '--' until the next step
        # boundary, which on the climb is over a minute away.
        self._state_pub = self.create_publisher(String, '/mission/state', 10)
        self.mission_state = 'IDLE'
        self.create_timer(0.5, self._publish_state)
        self.announce_colour = None
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.climb = self.create_client(Trigger, '/ramp/climb')
        self.descend = self.create_client(Trigger, '/ramp/descend')
        self.ramp_stop = self.create_client(Trigger, '/ramp/stop')
        self.approach = self.create_client(Trigger, '/approach/run')
        self.approach_stop = self.create_client(Trigger, '/approach/stop')
        self.stow = self.create_client(Trigger, '/grasp/stow')
        self.pick = self.create_client(Trigger, '/grasp/pick')
        self.place = self.create_client(Trigger, '/grasp/place')

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def _ramp_cb(self, msg):
        self.ramp_status = msg.data

    def _vision_cb(self, msg):
        self.vision_status = msg.data

    def _approach_cb(self, msg):
        self.approach_status = msg.data

    def _grasp_cb(self, msg):
        self.grasp_status = msg.data

    def _colour_cb(self, msg):
        colour = (msg.data or '').strip().lower()
        if colour in TARGET_COLOURS:
            self.colour = colour

    def _publish_state(self):
        self._state_pub.publish(String(data=self.mission_state))

    def set_state(self, state):
        """
        Record the step being executed, and say so immediately.

        The timer re-asserts it at 2 Hz, but a step boundary is exactly
        the moment a watcher cares about, and waiting up to 500 ms to
        show it makes the HUD look like it is lagging the robot.
        """
        self.mission_state = state
        self._publish_state()

    @staticmethod
    def _field_of(line, key):
        """Value of `key=` in a space-separated key=value status line."""
        for part in line.split(' '):
            if part.startswith(f'{key}='):
                return part.split('=', 1)[1]
        return None

    def _field(self, key):
        return self._field_of(self.ramp_status, key)

    def wait_for_colour(self, timeout=60.0):
        """Block until the phone picks a target. Returns the colour."""
        deadline = time.time() + timeout
        self.get_logger().info(
            'waiting for /mission/target_colour — pick one on the panel')
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.colour is not None:
                return self.colour
        self.get_logger().error(
            'no target colour after '
            f'{timeout:.0f}s. Pick one on the panel, or pass --colour.')
        return None

    def verify_target(self, colour, settle=3.0):
        """
        Check what the camera sees now that the robot is up there.

        A GATE for the grasp, but never for the traverse. Failing it skips
        the approach and the pick and lets the descent and the drive home
        run anyway: aborting outright would leave the robot parked on a
        0.65 m platform needing a manual recovery, which is a worse
        outcome than coming home empty and saying vision disagreed.
        """
        deadline = time.time() + settle
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.vision_status:
            self.get_logger().warn(
                'no /perception/status — is target_finder running?')
            return False
        found = self._field_of(self.vision_status, 'found') == '1'
        seen = self._field_of(self.vision_status, 'seen')
        if not found:
            # `seen` is the wrong-lane signal: the neighbouring lane's
            # target stays in frame at this distance, so naming what IS
            # visible turns "nothing here" into a diagnosis.
            self.get_logger().warn(
                f'{colour} NOT confirmed. visible: {seen} '
                f'({self.vision_status})')
            return False
        self.get_logger().info(f'{colour} confirmed: {self.vision_status}')
        return True

    def set_mode(self, mode, hold=1.5):
        """Assert a mission mode and hold it long enough to be seen."""
        deadline = time.time() + hold
        while time.time() < deadline and rclpy.ok():
            self._mode_pub.publish(String(data=mode))
            self._assert_colour()
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'mode -> {mode}')

    def _assert_colour(self):
        """Republish the CLI's colour, if the CLI is where it came from.

        target_finder, approach_server and grasp_server all take the
        choice off this topic, and two of the three refuse to start
        without it. With --colour and no panel there would otherwise be no
        publisher at all, and the mission would stall at the approach with
        every node individually behaving correctly.
        """
        if self.announce_colour is not None:
            self._colour_pub.publish(String(data=self.announce_colour))

    def call_service(self, client, name, timeout=180.0, poll=None):
        """
        Trigger a mission service and wait for it to report itself idle.

        `poll` is (status_getter, key) — the status topic field that reads
        'idle' when the worker is done, and the 'outcome' beside it. All
        three servers publish the same key=value shape for exactly this.
        """
        if not client.wait_for_service(timeout_sec=20.0):
            self.get_logger().error(f'{name} unavailable')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if future.result() is None or not future.result().success:
            self.get_logger().error(f'{name} refused: {future.result()}')
            return False
        if poll is None:
            return True

        getter, mode = poll
        t0 = time.time()
        while rclpy.ok():
            self._mode_pub.publish(String(data=mode))
            self._assert_colour()
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 < 2.0:
                continue                      # let it pick the work up
            status = getter()
            if self._field_of(status, 'phase') in ('idle', None) and status:
                break
            if time.time() - t0 > timeout:
                self.get_logger().error(f'{name} timed out')
                return False
        outcome = self._field_of(getter(), 'outcome')
        self.get_logger().info(
            f'{name}: outcome={outcome} after {time.time() - t0:.1f}s '
            f'({getter()})')
        return outcome in ('arrived', 'held', 'placed', 'done')

    def abort(self):
        """Stop whatever is driving and park the mode.

        /ramp/stop had no caller anywhere in the tree until now, which
        meant a failed step left the last commanded twist to age out
        against the arbiter's 0.3 s watchdog rather than being cancelled.
        On the platform that is the difference between stopping and
        coasting off a 0.65 m edge.
        """
        for client, name in ((self.ramp_stop, '/ramp/stop'),
                             (self.approach_stop, '/approach/stop')):
            if client.service_is_ready():
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        self.set_mode('idle', hold=1.0)

    def wait_ready(self, timeout=40.0):
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None and self.ramp_status:
                return True
        self.get_logger().error(
            f'not ready (odom={self.pose is not None}, '
            f'ramp_driver={bool(self.ramp_status)})')
        return False

    def nav_to(self, wx, wy, timeout=240.0):
        """Drive a flat-ground leg with Nav2. Returns True on SUCCEEDED."""
        if not self._nav.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('navigate_to_pose unavailable')
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = wx + WORLD_TO_MAP_X
        goal.pose.pose.position.y = wy
        goal.pose.pose.orientation.w = 1.0

        t0 = time.time()
        send = self._nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=20.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'goal ({wx}, {wy}) rejected')
            return False
        result = handle.get_result_async()
        while not result.done() and rclpy.ok():
            # Keep asserting the mode: if it lapsed the arbiter would stop
            # forwarding Nav2 mid-leg and the robot would simply halt.
            self._mode_pub.publish(String(data='nav'))
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > timeout:
                handle.cancel_goal_async()
                self.get_logger().error('nav leg timed out — cancelled')
                return False
        ok = result.result().status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(
            f'nav to ({wx:.2f}, {wy:.2f}): '
            f'{"SUCCEEDED" if ok else "FAILED"} in {time.time() - t0:.1f}s')
        return ok

    def ramp_segment(self, client, name, timeout=180.0):
        """Call a ramp_driver service and wait for it to finish."""
        if not client.wait_for_service(timeout_sec=20.0):
            self.get_logger().error(f'/ramp/{name} unavailable')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if future.result() is None or not future.result().success:
            self.get_logger().error(f'{name} refused: {future.result()}')
            return False

        t0 = time.time()
        # The driver reports segment=idle with an outcome when it is done.
        while rclpy.ok():
            self._mode_pub.publish(String(data='rl'))
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 < 2.0:
                continue                      # let it pick the segment up
            if self._field('segment') == 'idle':
                break
            if time.time() - t0 > timeout:
                self.get_logger().error(f'{name} timed out')
                return False
        outcome = self._field('outcome')
        self.get_logger().info(
            f'{name}: outcome={outcome} after {time.time() - t0:.1f}s '
            f'({self.ramp_status})')
        return outcome == 'goal'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--colour', choices=TARGET_COLOURS, default=None,
                        help='which target to fetch; picks its lane. '
                             'Omit to wait for /mission/target_colour '
                             'from the phone.')
    parser.add_argument('--lane', type=float, default=None,
                        help='override the lane with a bare world y, for '
                             'testing a pose the table does not name')
    parser.add_argument('--no-grasp', action='store_true',
                        help='traverse only, skipping the approach, the '
                             'pick and the place — reproduces the M4/M5 run')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = TraverseDemo()
    rc = 1
    confirmed = False
    fetched = False
    try:
        if not node.wait_ready():
            sys.exit(1)
        colour = args.colour or node.wait_for_colour()
        if colour is None:
            sys.exit(1)
        if args.colour:
            # The CLI is the only source in a headless run; see
            # _assert_colour. With the panel up, the panel already has it.
            node.announce_colour = args.colour
        lane = args.lane if args.lane is not None else lane_for_colour(colour)
        start = node.pose
        print(f'\nstart (world): ({start[0]:.2f}, {start[1]:.2f})')
        print(f'fetching {colour} from lane {lane:+.2f}'
              f'{" (traverse only)" if args.no_grasp else ""}\n')

        def verify():
            nonlocal confirmed
            confirmed = node.verify_target(colour)
            # Never blocks the traverse — see verify_target. It gates the
            # grasp instead, below.
            return True

        def approach():
            return node.call_service(
                node.approach, '/approach/run',
                poll=(lambda: node.approach_status, 'approach'))

        def pick():
            return node.call_service(
                node.pick, '/grasp/pick',
                poll=(lambda: node.grasp_status, 'idle'))

        def stow():
            return node.call_service(
                node.stow, '/grasp/stow',
                poll=(lambda: node.grasp_status, 'idle'))

        def place():
            return node.call_service(
                node.place, '/grasp/place',
                poll=(lambda: node.grasp_status, 'idle'))

        # Steps 2c-4 and 7 are the grasp half of the mission. They are
        # skipped both by --no-grasp and by a failed 2b, and in the second
        # case the descent and the drive home still run: a robot that comes
        # home empty is recoverable, a robot parked on the platform is not.
        grasp_steps = [
            ('2c. stow the arm', lambda: (node.set_mode('idle'), stow())[-1]),
            ('3. approach the target',
             lambda: (node.set_mode('approach'), approach())[-1]),
            ('4. pick it up', lambda: (node.set_mode('idle'), pick())[-1]),
        ]
        traverse_tail = [
            ('5. scripted descent',
             lambda: (node.set_mode('rl'),
                      node.ramp_segment(node.descend, 'descend'))[-1]),
            ('6. nav home',
             lambda: (node.set_mode('nav'),
                      node.nav_to(*HOME))[-1]),
        ]

        ok = True
        head = [
            ('1. nav to the pre-ramp pose',
             lambda: (node.set_mode('nav'),
                      node.nav_to(PRE_RAMP_X, lane))[-1]),
            ('2. RL climb',
             lambda: (node.set_mode('rl'),
                      node.ramp_segment(node.climb, 'climb'))[-1]),
            (f'2b. confirm {colour} is in front', verify),
        ]
        for label, step in head:
            print(f'--- {label} ---')
            node.set_state(label)
            if not step():
                print(f'FAILED at: {label}')
                node.set_state(f'ABORT ({label})')
                node.abort()
                ok = False
                break

        do_grasp = ok and not args.no_grasp and confirmed
        if ok and not do_grasp and not args.no_grasp:
            print(f'--- SKIPPING the grasp: {colour} was not confirmed '
                  f'in front of the robot ---')

        tail = (grasp_steps if do_grasp else []) + traverse_tail
        if do_grasp:
            tail.append(
                ('7. put it down at home',
                 lambda: (node.set_mode('idle'), place())[-1]))
        for label, step in tail:
            if not ok:
                break
            print(f'--- {label} ---')
            node.set_state(label)
            if not step():
                print(f'FAILED at: {label}')
                node.set_state(f'ABORT ({label})')
                node.abort()
                ok = False
                break
            if label.startswith('4.'):
                fetched = True

        node.set_mode('idle')
        end = node.pose
        print(f'\nend (world): ({end[0]:.2f}, {end[1]:.2f})')
        print(f'vision: {colour} '
              f'{"CONFIRMED" if confirmed else "not confirmed"}')
        if ok:
            print(f'home to within '
                  f'{math.hypot(end[0] - start[0], end[1] - start[1]):.2f} m')
            if args.no_grasp:
                print('TRAVERSE COMPLETE')
            elif fetched:
                print(f'FETCH COMPLETE — {colour} delivered')
            else:
                # Came home, came home empty. Distinguish it loudly: a
                # traverse that completes is not a fetch that succeeded.
                print(f'FETCH FAILED (wrong lane: saw '
                      f'{node._field_of(node.vision_status, "seen")}) — '
                      f'traverse completed, nothing picked up')
                ok = False
        rc = 0 if ok else 1
        # Leave a terminal state behind rather than just going quiet. A
        # HUD showing STALE tells a watcher the sequencer died; it does
        # not tell them whether the mission succeeded, and those are the
        # two outcomes worth distinguishing on a recording. Only set here
        # if a step did not already abort -- that label is more specific.
        if not node.mission_state.startswith('ABORT'):
            node.set_state('COMPLETE' if ok else 'FAILED')
        # The publish above is queued, not sent. Without a spin the node
        # is destroyed first and the last state never leaves the process.
        deadline = time.time() + 0.5
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
