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
grasp_server — the mission's pick and place, behind two services.

`pick_place.py` is a script that builds its own scene: it spawns a pedestal
and a cylinder in front of a robot it assumes is parked at its spawn pose,
runs sixteen steps, and exits. The mission needs the same arm machinery
pointed at an object that is already in the world, 5.9 m from spawn, at a
pose perception measured. The machinery is shared through arm_control.py;
what differs is here.

Four differences, each forced by the mission's scene
----------------------------------------------------
**No pedestal.** The targets are 158 mm cylinders standing on the crest
platform, tall enough that the grasp band lands at the one verified pinch
height with nothing else nearby. FUTURE_WORK 5b's palm-vs-pedestal
planning collision therefore has no object to collide with.

**The poses are solved, not tabulated.** approach_server stops at the
centre of the colour's approach window, and the last 0.17 m of that is
dead reckoning, so where the target actually ends up is a measurement
rather than a constant. `arm_ik.ik_or_none` is asked for the grasp and
hover at that x, exactly as pick_place.retarget() already does. The *z*
does not come from vision: the object is known to stand on the same plane
the robot stands on, and the blob centroid is not its grasp band.

**The magnet fires BEFORE the fingers close**, which is the reverse of the
demo. The demo's cylinder sits on a pedestal and is 28 mm thick; these
stand free on the platform and the thinnest is 20 mm, which tips at about
7 degrees. The fingers do not close symmetrically (grip2 carries a
0.2332 rad pitch in its collision geometry), so closing first risks
nudging a free-standing target over before anything is holding it. Welding
first freezes it where it stands and makes the close what it already
was under the magnet grasp: corroborating evidence.

**The arm must already be stowed.** At 'home' the pinch sits at base-x
0.162, z 0.148 — inside the volume the target occupies once the robot has
driven up to it. That is not just a planning-scene collision at the start
state; it is the gripper physically pushing the cylinder over during the
approach. `/grasp/stow` exists so the sequencer can raise the arm to 'up'
before the approach runs, and 'up' is also the carry pose.

Why 'up' is the carry pose
--------------------------
Computed against arm_ik with the palm rotation the weld freezes in — the
object tilts with the palm, it does not stay vertical — the carried
object's lowest point sits this far above the wheel contact plane:

    pose    level     18 deg nose-down   24 deg
    home    +0.025    -0.037             -0.057
    grasp   +0.000    -0.047             -0.062
    raise   +0.036    -0.012             -0.027
    lift    +0.110    +0.068             +0.053
    up      +0.227    +0.218             +0.210

The mission carries its target down an 18 degree slope. Three of those
five poses would drag it into the ramp.

Services and topics
-------------------
srv  /grasp/stow    std_srvs/Trigger  raise the arm clear, before driving
srv  /grasp/pick    std_srvs/Trigger  hover, descend, weld, lift, carry
srv  /grasp/place   std_srvs/Trigger  set it down, release, retreat, home
in   /mission/target_colour  std_msgs/String
in   /approach/target        geometry_msgs/PointStamped  (base_footprint)
out  /grasp/status           std_msgs/String  (5 Hz, key=value)

Requires move_group. It is not in any other mission launch file — see
coco_mission/launch/mission.launch.py.
"""

import threading
import time

import arm_ik

from arm_control import ArmControl, GRIP_CLOSED, GRIP_OPEN, model_z

from coco_config.robot import (approach_stop_x, approach_window,
                               GRASP_APPROACH_X, GRASP_HOVER_CLEARANCE,
                               GRASP_MAX_LATERAL, target_by_colour,
                               TARGET_COLOURS, TARGET_GRASP_Z, TARGET_HEIGHT)

from geometry_msgs.msg import PointStamped

import rclpy
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import String
from std_srvs.srv import Trigger

STATUS_HZ = 5.0

# Joint-space waypoints shared with the demo. 'grasp' and 'hover' are
# solved per pick and inserted at run time; the rest are fixed.
POSES = {
    'home': [0.0, 0.0],
    'up': [-1.2, -0.5],
    'lift': [-0.3, 0.2],
    'raise': [0.10, 0.45],
}
CARRY_POSE = 'up'

# GRASP_MAX_LATERAL comes from coco_config.robot: the approach has to hit
# it and the grasp enforces it, so it cannot live in only one of the two.

# An approach fix older than this is not describing where the robot is now.
APPROACH_FIX_MAX_AGE = 120.0

# The platform under the robot, as a planning-scene floor. Big enough to
# stop the arm planning through it, small enough not to swallow the target.
GROUND_SIZE = (2.0, 2.0, 0.02)
GROUND_POS = (0.0, 0.0, -0.012)

STATUS_KEYS = ('phase', 'colour', 'x', 'y', 'lifted', 'outcome')
SIGNED_STATUS_KEYS = ('y',)


def _status_value(key, value):
    """Format one status field; None reads as '--', never as blank."""
    if value is None:
        return '--'
    if isinstance(value, bool):
        return '1' if value else '0'
    if isinstance(value, float):
        return f'{value:+.4f}' if key in SIGNED_STATUS_KEYS else f'{value:.4f}'
    return str(value)


def format_status(**fields):
    """One-line grasp state, space-separated key=value for the panel."""
    return ' '.join(f'{key}={_status_value(key, fields.get(key))}'
                    for key in STATUS_KEYS)


def solve_grasp(x, hover_clearance=GRASP_HOVER_CLEARANCE):
    """
    Joint poses for grasping a target whose axis is at base-x `x`.

    Returns ``(grasp, hover)`` or None if either end of the vertical
    descent is out of reach. Both are checked, because a reachable grasp
    under an unreachable hover is a plan that cannot be started — which
    move_group reports identically to an unreachable goal.
    """
    grasp = arm_ik.ik_or_none(x, TARGET_GRASP_Z)
    hover = arm_ik.ik_or_none(x, TARGET_GRASP_Z + hover_clearance)
    if grasp is None or hover is None:
        return None
    return list(grasp), list(hover)


def check_target_pose(colour, x, y, max_lateral=GRASP_MAX_LATERAL):
    """
    Is this target somewhere the arm can actually take it?

    Returns None when it is, or a human-readable reason when it is not.
    Separated out and pure so the refusals are testable: a grasp server
    that welds whatever it is pointed at would report success from the
    wrong lane, and the magnet is quite capable of welding across a gap.
    """
    window = approach_window(colour)
    if window is None:
        return f'{colour!r} is not one of the mission targets'
    near, far = window
    if not near <= x <= far:
        return (f'{colour} axis at base-x {x:.4f} is outside its approach '
                f'window [{near:.4f}, {far:.4f}]')
    if abs(y) > max_lateral:
        return (f'{colour} is {y * 1000:+.1f} mm off the arm plane, over the '
                f'{max_lateral * 1000:.0f} mm limit — the arm cannot reach '
                f'sideways')
    if solve_grasp(x) is None:
        return f'no grasp/hover IK for a target at base-x {x:.4f}'
    return None


class GraspServer(ArmControl):
    """Picks the chosen target off the platform and puts it down at home."""

    def __init__(self):
        # magnet_model is None until a colour arrives: binding to the wrong
        # one is worse than binding late, and bind_magnet() is called from
        # the colour callback, long before any grasp.
        # spun_externally: main() spins this node, so the sequence worker
        # must never call rclpy.spin_once itself. See ArmControl.
        super().__init__('grasp_server', None, dict(POSES),
                         spun_externally=True)

        self.declare_parameter('colour_topic', '/mission/target_colour')
        self.declare_parameter('approach_topic', '/approach/target')
        self.declare_parameter('status_topic', '/grasp/status')
        self.declare_parameter('status_hz', STATUS_HZ)
        self.declare_parameter('target_colour', '')

        self.colour = None
        self.phase = 'idle'
        self.outcome = None
        self.lifted = None
        self.target_xy = None
        self._fix = None            # (x, y, z) from approach_server
        self._fix_at = None
        self._lock = threading.Lock()
        self._busy = False

        self.create_subscription(
            String, self.get_parameter('colour_topic').value,
            self._on_colour, 10)
        self.create_subscription(
            PointStamped, self.get_parameter('approach_topic').value,
            self._on_fix, 10)
        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)

        self.create_service(Trigger, '/grasp/stow', self._on_stow)
        self.create_service(Trigger, '/grasp/pick', self._on_pick)
        self.create_service(Trigger, '/grasp/place', self._on_place)
        self.create_timer(
            1.0 / float(self.get_parameter('status_hz').value),
            self._publish_status)

        initial = (self.get_parameter('target_colour').value or '').strip()
        if initial in TARGET_COLOURS:
            self._select(initial)

        self.get_logger().info(
            'grasp_server ready: /grasp/stow, /grasp/pick, /grasp/place. '
            'Needs move_group; assert /mission/mode idle while it runs so '
            'the wheels stay still.')

    # ── inputs ───────────────────────────────────────────────────────────
    def _select(self, colour):
        self.colour = colour
        self.bind_magnet(target_by_colour(colour).model)

    def _on_colour(self, msg):
        colour = (msg.data or '').strip().lower()
        if colour in TARGET_COLOURS and colour != self.colour:
            self.get_logger().info(f'target {self.colour} -> {colour}')
            self._select(colour)

    def _on_fix(self, msg):
        self._fix = (msg.point.x, msg.point.y, msg.point.z)
        self._fix_at = time.monotonic()

    def _publish_status(self):
        self._status_pub.publish(String(data=format_status(
            phase=self.phase, colour=self.colour,
            x=None if self.target_xy is None else self.target_xy[0],
            y=None if self.target_xy is None else self.target_xy[1],
            lifted=self.lifted, outcome=self.outcome)))

    def _reject(self, why):
        self.get_logger().error(why)
        self.outcome = why
        return Trigger.Response(success=False, message=why)

    def _start(self, worker, name):
        if self._busy:
            return self._reject(f'busy in {self.phase}')
        if self.colour is None:
            return self._reject(
                'no target colour — publish /mission/target_colour or start '
                'with -p target_colour:=<red|green|blue|yellow>')
        self.outcome = None
        threading.Thread(target=self._run, args=(worker, name),
                         daemon=True).start()
        return Trigger.Response(
            success=True, message=f'{name} started; watch /grasp/status')

    def _run(self, worker, name):
        with self._lock:
            self._busy = True
            self.phase = name
            try:
                ok = worker()
                self.outcome = self.outcome or ('done' if ok else 'failed')
            except Exception as exc:                      # noqa: BLE001
                # A dead worker looks exactly like a slow grasp to the
                # sequencer, which is waiting on /grasp/status.
                self.outcome = f'error: {exc}'
                self.get_logger().error(f'{name} failed: {exc}')
                # Do NOT open the gripper on an abort: mid-carry on a 0.65 m
                # platform an unplanned finger motion is a second failure on
                # top of the first. Drop the weld and leave the fingers.
                if self.holding:
                    self.get_logger().warn(
                        'still holding the target — detaching before abort')
                    self.magnet('detached')
            finally:
                self.phase = 'idle'
                self._busy = False
        self.get_logger().info(f'{name} finished: {self.outcome}')

    # ── services ─────────────────────────────────────────────────────────
    def _on_stow(self, request, response):
        del request, response
        if self._busy:
            return self._reject(f'busy in {self.phase}')
        self.outcome = None
        threading.Thread(target=self._run, args=(self._do_stow, 'stow'),
                         daemon=True).start()
        return Trigger.Response(success=True, message='stowing')

    def _on_pick(self, request, response):
        del request, response
        return self._start(self._do_pick, 'pick')

    def _on_place(self, request, response):
        del request, response
        return self._start(self._do_place, 'place')

    # ── sequences ────────────────────────────────────────────────────────
    def _do_stow(self):
        """Raise the arm clear before the robot drives at the target.

        Not cosmetic. At 'home' the pinch is at base-x 0.162, z 0.148 and
        the target's body spans z 0 to 0.158 at base-x ~0.149, so an
        approach with the arm down walks the gripper straight into the
        cylinder and knocks it over before anything has looked at it.
        """
        if not self.move_client.wait_for_server(timeout_sec=20.0):
            self.outcome = 'move_group action server unavailable'
            return False
        # Nothing is in the scene yet, so this plans against an empty world
        # — which is the point of doing it before the target is added.
        self.remove_scene_objects(('ground', 'target'))
        return self.move_arm(CARRY_POSE)

    def _target_pose(self):
        """Where the target is, from the approach fix or from the nominal."""
        if self._fix is not None and self._fix_at is not None and (
                time.monotonic() - self._fix_at <= APPROACH_FIX_MAX_AGE):
            return self._fix[0], self._fix[1]
        # No fix is not fatal: the approach stops at a known place by
        # construction. Say which one is being used, because "grasped at the
        # nominal" and "grasped at the measured" fail in different ways.
        nominal = approach_stop_x(self.colour)
        self.get_logger().warn(
            f'no fresh /approach/target — assuming the nominal stop pose '
            f'({nominal:.4f}, 0.0). Did approach_server run?')
        return nominal, 0.0

    def _do_pick(self):
        if not self.move_client.wait_for_server(timeout_sec=20.0):
            self.outcome = 'move_group action server unavailable'
            return False
        # expect_xy=None: the mission grasps ~5.9 m from spawn. Upright is
        # still checked — a toppled robot would plan joint-space goals
        # perfectly and grasp air.
        if not self.check_robot_pose('pre-grasp', expect_xy=None):
            self.outcome = 'robot is not upright'
            return False

        x, y = self._target_pose()
        self.target_xy = (x, y)
        why = check_target_pose(self.colour, x, y)
        if why is not None:
            self.outcome = why
            self.get_logger().error(why)
            return False

        grasp, hover = solve_grasp(x)
        self.poses['grasp'] = grasp
        self.poses['hover'] = hover
        target = target_by_colour(self.colour)
        self.get_logger().info(
            f'{self.colour} at base-x {x:.4f}, y {y:+.4f}: grasp '
            f'{[round(v, 4) for v in grasp]}, hover '
            f'{[round(v, 4) for v in hover]}')

        # Stow first, with an empty scene, then declare the obstacles. Same
        # ordering the demo uses and for the same reason: at 'home' the
        # gripper volume overlaps the target, so the start state would be in
        # collision and nothing would plan.
        self.remove_scene_objects(('ground', 'target'))
        if not self.move_arm(CARRY_POSE):
            self.outcome = 'could not stow the arm before the grasp'
            return False

        # The target is a strict obstacle for the transit to 'hover': the
        # planner must route the open gripper AROUND it, not through it.
        self.publish_scene([
            self.box('ground', GROUND_SIZE, GROUND_POS),
            self.cylinder('target', target.diameter, TARGET_HEIGHT,
                          (x, y, TARGET_HEIGHT / 2.0)),
        ])

        steps = [
            ('open gripper',
             lambda: self.move_gripper(GRIP_OPEN, 'open')),
            ('hover above target', lambda: self.move_arm('hover')),
            ('allow gripper-target contact',
             self.allow_gripper_target_contact),
            ('grasp approach', lambda: self.move_arm('grasp', speed=0.1)),
            # Weld BEFORE closing: see the module docstring. A free-standing
            # 20 mm cylinder tips at about 7 degrees.
            ('magnet attach', self.grasp_magnet),
            ('close gripper',
             lambda: self.move_gripper(GRIP_CLOSED, 'closed',
                                       expect_object=None)),
            # The target now moves with the arm, so the static collision
            # object at its old pose is a phantom obstacle in every
            # subsequent plan.
            ('release the scene object',
             lambda: self.remove_scene_objects(('target',))),
            ('raise', lambda: self.move_arm('raise', speed=0.05)),
            ('confirm lift', self.check_lifted),
            ('carry', lambda: self.move_arm(CARRY_POSE, speed=0.08)),
        ]
        for label, step in steps:
            self.phase = f'pick:{label}'
            if not step():
                self.outcome = f'failed at {label}'
                self.get_logger().error(f"Step '{label}' failed — aborting")
                if self.holding:
                    self.magnet('detached')
                return False
        self.lifted = True
        self.outcome = 'held'
        return True

    def _do_place(self):
        if not self.move_client.wait_for_server(timeout_sec=20.0):
            self.outcome = 'move_group action server unavailable'
            return False
        if not self.check_robot_pose('pre-place', expect_xy=None):
            self.outcome = 'robot is not upright'
            return False
        if not self.holding:
            self.outcome = 'nothing held — nothing to place'
            self.get_logger().error(self.outcome)
            return False

        # Put it down at the verified pinch point rather than back where it
        # was picked from — the robot is at home now, not on the platform.
        # That pose puts the pinch exactly TARGET_GRASP_Z above the ground,
        # which is the cylinder's own bottom face on the floor, so the
        # release needs no new geometry at all.
        place = arm_ik.ik_or_none(GRASP_APPROACH_X, TARGET_GRASP_Z)
        if place is None:
            self.outcome = 'the verified place pose is unreachable'
            self.get_logger().error(self.outcome)
            return False
        self.poses['place'] = list(place)
        self.publish_scene([self.box('ground', GROUND_SIZE, GROUND_POS)])

        steps = [
            ('lift clear', lambda: self.move_arm('lift', speed=0.15)),
            ('descend to place', lambda: self.move_arm('place', speed=0.1)),
            # Detach first, then open. Opening while still welded leaves the
            # object hanging under the palm, which reads as a successful
            # place until the arm moves.
            ('magnet detach', lambda: self.magnet('detached')),
            ('release', lambda: self.move_gripper(GRIP_OPEN, 'open')),
            ('retreat', lambda: self.move_arm('lift', speed=0.15)),
            ('confirm it stayed down', self.check_released),
            ('home', lambda: self.move_arm('home')),
        ]
        for label, step in steps:
            self.phase = f'place:{label}'
            if not step():
                self.outcome = f'failed at {label}'
                self.get_logger().error(f"Step '{label}' failed — aborting")
                return False
        self.lifted = False
        self.outcome = 'placed'
        return True

    def check_released(self):
        """
        Confirm the object stayed on the ground when the arm left.

        The inverse of check_lifted, and needed for the same reason: the
        magnet's state topic says "detached" whether or not the weld it was
        holding was real, so an object still stuck to the palm would be
        reported as a completed delivery right up until the arm moved.
        """
        height = model_z(self.magnet_model)
        if height is None:
            self.get_logger().warn(
                'could not read the target height — release check skipped')
            return True
        expected = TARGET_HEIGHT / 2.0
        if abs(height - expected) > 0.02:
            self.get_logger().error(
                f'Target is at z {height:.4f}, not standing on the ground '
                f'({expected:.4f}). It is still attached to the arm, or it '
                f'fell over on release.')
            return False
        self.get_logger().info(
            f'Target standing at z {height:.4f} — the place is real')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = GraspServer()
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
