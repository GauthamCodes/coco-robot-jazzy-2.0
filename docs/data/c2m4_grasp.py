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
c2m4_grasp.py — drive ONE perception-driven grasp and say what happened.

The C2-M4.1 manipulation instrument. `c2m4_localisation.py` answers "how
far is the estimate from the truth"; this answers the question that
actually matters, "does that estimate pick the object up".

ONE RUN PER SIMULATOR. Not a style choice: the gz DetachableJoint binds
its child once, and a second grasp in the same simulator reports success
while welding nothing (CLAUDE.md rule 5, and `arm_control.check_lifted`
exists because it happened). The caller is a shell script that brings up
a fresh world for every invocation.

THE CHAIN THIS EXERCISES
------------------------
    camera -> target_pose_node -> /perception/target (PointStamped)
           -> approach_server servo/align/creep
           -> /approach/target
           -> grasp_server check_target_pose -> arm_ik -> MoveIt
           -> magnet -> check_lifted -> /grasp/place -> check_released

`target_pose_node` is run with `point_topic:=/perception/target`, so the
node under test is the C2-M4.0 pipeline and `target_finder` is NOT
running. One publisher on that topic, always — running both would put
two different estimates on one topic and let the race decide the grasp.

GROUND TRUTH, and where it is allowed
-------------------------------------
Read here for verification only, never fed anywhere:

    /world/coco_world/pose/info   the target's world pose, before and
                                  after the lift and after the place
    /model/coco/odometry          the robot's world pose

The deployable path — perception, approach, grasp — never sees any of
it. `grasp_server` runs its own `check_lifted` off the same gz reading;
this script records an INDEPENDENT copy so a passing run is backed by a
number in the CSV rather than by a log line.

Usage
-----
    python3 c2m4_grasp.py --colour blue --standoff 0.45 --lateral 0.0 \
        --out c2m4_grasp.csv
"""

import argparse
import csv
import math
import os
import sys
import time

from c2m4_localisation import (PLATFORM_Z, WORLD, gz_service,
                               gz_world_poses)

from coco_config.robot import SPAWN_Z, TARGET_ROW_X, target_by_colour

from geometry_msgs.msg import PointStamped

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String

from std_srvs.srv import Trigger


# ── the outcome vocabulary (section 10 of the C2-M4.1 brief) ────────────
# Each of these fails for a different reason and the point of the
# benchmark is that they stay apart.
NOT_DETECTED = 'target_not_detected'
DEPTH_INVALID = 'depth_invalid'
NO_TRANSFORM = 'transform_unavailable'
APPROACH_FAILED = 'approach_failed'
NO_APPROACH_FIX = 'approach_target_invalid'
OFF_ARM_PLANE = 'target_outside_workspace'
IK_INFEASIBLE = 'ik_infeasible'
PLAN_FAILED = 'motion_planning_failed'
GRASP_FAILED = 'grasp_failed'
LIFT_UNVERIFIED = 'grasp_verification_failed'
PLACE_FAILED = 'placement_failed'
SUCCESS = 'success'

# A lift of at least this much says the object physically came up. Same
# figure arm_control.LIFT_MIN_RISE uses; duplicated deliberately so this
# script's verdict does not depend on importing the node it is checking.
LIFT_MIN_RISE = 0.02

FIELDS = ('colour', 'standoff_cmd', 'lateral_cmd', 'outcome',
          'perception_validity', 'perception_x', 'perception_y',
          'perception_qual', 'perception_reach_appr', 'perception_cand',
          'approach_outcome', 'approach_travel', 'fix_x', 'fix_y',
          'grasp_outcome', 'grasp_phase', 'grasp_x', 'grasp_y',
          'target_z_before', 'target_z_lifted', 'lift_mm', 'lift_verified',
          'target_z_placed', 'place_dxy_mm', 'place_verified',
          'robot_x', 'robot_y', 'seconds')


def parse_kv(text):
    """`key=value key=value ...` -> dict, '--' as None.

    Note the C2-M3.0 finding: grasp_server writes free text after
    `outcome=`, so `outcome` is recovered separately by `tail_value`.
    """
    fields = {}
    for token in (text or '').split():
        if '=' not in token:
            continue
        key, _, value = token.partition('=')
        fields[key] = None if value == '--' else value
    return fields


def tail_value(text, key):
    """Everything after `key=`, spaces included.

    grasp_server's status line ends with `outcome=<free text>`, and
    splitting that on spaces keeps only the first word — which turns
    'failed at magnet attach' into 'failed' and drops the diagnosis.
    """
    marker = f'{key}='
    index = (text or '').find(marker)
    return None if index < 0 else (text[index + len(marker):].strip() or None)


class GraspRun(Node):
    """Brings one placement all the way to a verified grasp, or explains."""

    def __init__(self):
        super().__init__('c2m4_grasp')
        self.perception = ''
        self.approach = ''
        self.grasp = ''
        self.fix = None
        self.odom = None

        self.create_subscription(
            String, '/perception/target_pose/status',
            lambda m: setattr(self, 'perception', m.data), 10)
        self.create_subscription(
            String, '/approach/status',
            lambda m: setattr(self, 'approach', m.data), 10)
        self.create_subscription(
            String, '/grasp/status',
            lambda m: setattr(self, 'grasp', m.data), 10)
        self.create_subscription(
            PointStamped, '/approach/target', self._on_fix, 10)
        self.create_subscription(
            Odometry, '/model/coco/odometry',
            lambda m: setattr(self, 'odom', m),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        self.colour_pub = self.create_publisher(
            String, '/mission/target_colour', 10)
        # The arbiter latches this. 'approach' opens /cmd_vel_approach;
        # 'idle' closes every source, which is what the arm needs -- a
        # wheel command during a grasp moves the thing being grasped.
        self.mode_pub = self.create_publisher(String, '/mission/mode', 10)

        self.approach_run = self.create_client(Trigger, '/approach/run')
        self.grasp_pick = self.create_client(Trigger, '/grasp/pick')
        self.grasp_place = self.create_client(Trigger, '/grasp/place')
        self.grasp_stow = self.create_client(Trigger, '/grasp/stow')

    def _on_fix(self, msg):
        self.fix = (msg.point.x, msg.point.y, msg.point.z)

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def announce(self, colour, mode, seconds=2.0):
        """Republish the latched inputs; both are latch-by-repetition."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            self.colour_pub.publish(String(data=colour))
            self.mode_pub.publish(String(data=mode))
            self.spin(0.25)

    def call(self, client, name, timeout=60.0):
        """Call a Trigger and pump callbacks while it runs.

        NOTE: both /approach/run and /grasp/pick are ASYNCHRONOUS. They
        start a worker thread and return success immediately with
        'watch /<name>/status'. This returns the ACCEPTANCE, not the
        outcome — `await_idle` is what waits for the work. Reading the
        acceptance as the result is a run that reports 'ok' 17 s into a
        90 s grasp, which is exactly what it did the first time.
        """
        if not client.wait_for_service(timeout_sec=30.0):
            return None, f'{name} service unavailable'
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                response = future.result()
                return response.success, response.message
        return None, f'{name} timed out after {timeout:.0f}s'

    def await_idle(self, which, timeout, settle=2.0):
        """Wait for an async worker to finish; return its status line.

        Both servers set `phase=idle` and a terminal `outcome=` when the
        worker returns. `settle` covers the gap between the service
        accepting and the worker's first status tick, so an immediate
        `phase=idle` read from BEFORE the work started is not mistaken
        for the work being over.
        """
        self.spin(settle)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.spin(0.5)
            text = getattr(self, which)
            fields = parse_kv(text)
            if fields.get('phase') == 'idle' and fields.get('outcome'):
                return text, True
        return getattr(self, which), False

    def place_robot(self, x, y):
        ok = gz_service(
            f'/world/{WORLD}/set_pose', 'gz.msgs.Pose', 'gz.msgs.Boolean',
            f'name: "coco", position: {{x: {x}, y: {y}, '
            f'z: {PLATFORM_Z + SPAWN_Z}}}, orientation: {{z: 0.0, w: 1.0}}')
        if not ok:
            raise RuntimeError('set_pose failed — is the simulator up?')
        self.spin(3.0)
        return ok


def target_z(model):
    """The target's world z, from gz. Ground truth, verification only."""
    poses = gz_world_poses()
    return None if model not in poses else poses[model][2]


def target_xy(model):
    poses = gz_world_poses()
    return None if model not in poses else poses[model][:2]


def run(args):
    target = target_by_colour(args.colour)
    row = {key: '' for key in FIELDS}
    row.update(colour=args.colour, standoff_cmd=args.standoff,
               lateral_cmd=args.lateral)
    started = time.monotonic()

    node = GraspRun()
    node.spin(3.0)

    try:
        # ── place, then declare the colour ───────────────────────────
        node.place_robot(TARGET_ROW_X - args.standoff,
                         target.lane_y + args.lateral)
        node.announce(args.colour, 'idle', seconds=3.0)

        if node.odom is not None:
            row['robot_x'] = round(node.odom.pose.pose.position.x, 4)
            row['robot_y'] = round(node.odom.pose.pose.position.y, 4)

        # Stow the arm before anything drives: at 'home' the gripper
        # volume overlaps the target and the approach would knock it over.
        node.call(node.grasp_stow, 'stow')
        node.await_idle('grasp', timeout=180.0)

        # ── perception ───────────────────────────────────────────────
        node.spin(3.0)
        fields = parse_kv(node.perception)
        row['perception_validity'] = fields.get('validity') or ''
        row['perception_x'] = fields.get('x') or ''
        row['perception_y'] = fields.get('y') or ''
        row['perception_qual'] = fields.get('qual') or ''
        row['perception_reach_appr'] = fields.get('reach_appr') or ''
        row['perception_cand'] = fields.get('cand') or ''

        validity = fields.get('validity')
        if validity != 'VALID':
            row['outcome'] = {
                'DEPTH_INVALID': DEPTH_INVALID,
                'NO_TRANSFORM': NO_TRANSFORM,
                'STALE_TRANSFORM': NO_TRANSFORM,
            }.get(validity, NOT_DETECTED)
            return row

        # ── approach ─────────────────────────────────────────────────
        node.announce(args.colour, 'approach', seconds=2.0)
        accepted, message = node.call(node.approach_run, 'approach')
        if not accepted:
            row['outcome'] = APPROACH_FAILED
            row['approach_outcome'] = message or 'refused'
            node.announce(args.colour, 'idle', seconds=2.0)
            return row
        # The service only accepted the job. Wait for the worker.
        status, finished = node.await_idle('approach', timeout=300.0)
        approach_fields = parse_kv(status)
        row['approach_outcome'] = (approach_fields.get('outcome')
                                   or ('timeout' if not finished else ''))
        row['approach_travel'] = approach_fields.get('travel') or ''
        # Wheels off before the arm moves.
        node.announce(args.colour, 'idle', seconds=2.0)

        if row['approach_outcome'] != 'arrived':
            row['outcome'] = APPROACH_FAILED
            return row
        if node.fix is None:
            row['outcome'] = NO_APPROACH_FIX
            return row
        row['fix_x'] = round(node.fix[0], 5)
        row['fix_y'] = round(node.fix[1], 5)

        # ── grasp ────────────────────────────────────────────────────
        before = target_z(target.model)
        row['target_z_before'] = '' if before is None else round(before, 5)

        accepted, message = node.call(node.grasp_pick, 'pick')
        if not accepted:
            row['outcome'] = GRASP_FAILED
            row['grasp_outcome'] = message or 'refused'
            return row
        status, finished = node.await_idle('grasp', timeout=420.0)
        grasp_fields = parse_kv(status)
        row['grasp_phase'] = grasp_fields.get('phase') or ''
        row['grasp_x'] = grasp_fields.get('x') or ''
        row['grasp_y'] = grasp_fields.get('y') or ''
        outcome_text = tail_value(status, 'outcome') or (
            'timeout' if not finished else '')
        row['grasp_outcome'] = outcome_text
        # 'held' is _do_pick's success sentinel. Anything else is a
        # refusal or a failure, and the text says which.
        ok = outcome_text == 'held'

        # ── physical verification, independent of the server's own ───
        lifted = target_z(target.model)
        row['target_z_lifted'] = '' if lifted is None else round(lifted, 5)
        if before is not None and lifted is not None:
            rise = lifted - before
            row['lift_mm'] = round(rise * 1000, 1)
            row['lift_verified'] = rise >= LIFT_MIN_RISE
        else:
            row['lift_verified'] = ''

        if not ok:
            text = (outcome_text or '').lower()
            if 'off the arm plane' in text or 'sideways' in text:
                row['outcome'] = OFF_ARM_PLANE
            elif 'outside its approach window' in text or 'no grasp' in text:
                row['outcome'] = IK_INFEASIBLE
            elif 'plan' in text or 'move_group' in text:
                row['outcome'] = PLAN_FAILED
            else:
                row['outcome'] = GRASP_FAILED
            return row
        if row['lift_verified'] is not True:
            # The server said yes and the object did not move. This is
            # exactly the stale-binding failure check_lifted exists for.
            row['outcome'] = LIFT_UNVERIFIED
            return row

        # ── place ────────────────────────────────────────────────────
        # KNOWN LIMITATION, and it is the server's, not this script's.
        # grasp_server.check_released asserts the object stands at
        # TARGET_HEIGHT/2 — the ground plane AT HOME. These runs place on
        # the platform deck, PLATFORM_Z higher, so that check reports a
        # good release as a failure. Recorded, not worked around: the
        # server's verdict is kept in `grasp_outcome`, and the physical
        # question is answered here instead, from gz, against the deck
        # the object actually started on.
        accepted, message = node.call(node.grasp_place, 'place')
        status, _finished = (node.await_idle('grasp', timeout=420.0)
                             if accepted else (node.grasp, False))
        placed = target_z(target.model)
        row['target_z_placed'] = '' if placed is None else round(placed, 5)
        here = target_xy(target.model)
        if here is not None and node.odom is not None:
            dx = here[0] - node.odom.pose.pose.position.x
            dy = here[1] - node.odom.pose.pose.position.y
            row['place_dxy_mm'] = round(math.hypot(dx, dy) * 1000, 1)
        # Released means it came back down onto the same deck it was
        # standing on before the grasp, and is no longer riding the palm.
        row['place_verified'] = bool(
            placed is not None and before is not None
            and abs(placed - before) < LIFT_MIN_RISE)
        row['grasp_outcome'] = (tail_value(status, 'outcome')
                                or message or row['grasp_outcome'])
        if not row['place_verified']:
            row['outcome'] = PLACE_FAILED
            return row

        row['outcome'] = SUCCESS
        return row
    finally:
        row['seconds'] = round(time.monotonic() - started, 1)
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--colour', required=True)
    parser.add_argument('--standoff', type=float, required=True)
    parser.add_argument('--lateral', type=float, default=0.0)
    parser.add_argument('--out', default=None)
    args = parser.parse_args()

    rclpy.init()
    try:
        row = run(args)
    finally:
        rclpy.shutdown()

    print('\n=== RESULT ===')
    for key in FIELDS:
        print(f'  {key:>22} : {row.get(key)}')

    if args.out:
        exists = os.path.exists(args.out)
        with open(args.out, 'a', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS,
                                    extrasaction='ignore')
            if not exists:
                writer.writeheader()
            writer.writerow(row)
        print(f'appended to {args.out}')
    return 0 if row['outcome'] == SUCCESS else 1


if __name__ == '__main__':
    sys.exit(main())
