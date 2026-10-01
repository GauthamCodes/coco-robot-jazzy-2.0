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
arm_control.py
==============
The arm plumbing shared by the pick-and-place demo and the mission grasp.

`pick_place.py` was the only thing that knew how to drive this arm, and
everything it knew was tangled up with its own scene: a pedestal it spawns,
a cylinder called `pick_target`, and a robot assumed to be sitting at its
spawn pose. The mission needs the same machinery pointed at an object that
is already in the world, 5.9 m away, at a pose perception measured. So the
machinery moved here and the two scenes stayed where they were.

What is in here is everything that is true of the arm regardless of what it
is picking up: MoveIt joint goals, the gripper's confirm-against-joint-states
close, the magnet weld and the height check that proves it took, and the
planning-scene helpers.

Two behaviours of the gz DetachableJoint are load-bearing and both were
measured rather than read (see pick_place.py's MAGNET_MODEL block for the
full account):

- it attaches its child **the instant the model spawns**, not when
  commanded, so anything that spawns a graspable object must detach first;
- it binds to that entity **once**. Remove and re-spawn the model and a
  later attach still answers "attached" while welding nothing — a silent
  success. Hence `check_lifted`, which reads the object's own height out of
  Gazebo instead of believing the plugin.
"""

import math
import subprocess
import time

from builtin_interfaces.msg import Duration as DurationMsg

from coco_config.robot import SPAWN_XY

from geometry_msgs.msg import Pose

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    CollisionObject,
    Constraints,
    JointConstraint,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene

from nav_msgs.msg import Odometry

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from sensor_msgs.msg import JointState

from shape_msgs.msg import SolidPrimitive

from std_msgs.msg import Empty as EmptyMsg, String

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['m_link1_Revolute-6', 'm_link2_Revolute-7']
GRIPPER_JOINTS = ['m_link3_Revolute-8', 'm_link3_Revolute-9']
GRIPPER_LINKS = ['grip1', 'grip2', 'm_link3']

# Gripper motion is confirmed against /joint_states rather than timed.
GRIP_TIMEOUT = 8.0        # s to reach the setpoint before giving up
GRIP_TOLERANCE = 0.02     # rad from the commanded position = "arrived"
GRIP_STALL_EPS = 0.002    # rad of motion between samples that counts as moving
GRIP_STALL_TIME = 0.5     # s of no motion before declaring the fingers stopped

GRIP_OPEN = [0.5, -0.5]
# A hard pinch, and deliberately NOT a function of the object's width.
# 0.02 rad commands a gap well under the thinnest mission target (20 mm),
# so the fingers stall on all four and the position-controlled joints keep
# pressing (effort-limited) through the lift. Deriving a per-width setpoint
# would need a gap(q) model of a gripper whose two fingers are not
# symmetric — grip2 carries a 0.2332 rad pitch in its collision geometry —
# and it would buy nothing: the magnet is the grasp and the fingers are
# corroborating evidence (see move_gripper's expect_object=None).
GRIP_CLOSED = [0.02, -0.02]

MAGNET_TIMEOUT = 5.0      # s to see the state transition before giving up
# Metres the target must gain between the grasp pose and the top of the
# first lift. The arm lifts it far further; the threshold only has to
# separate "came with us" from "stayed where it was".
LIFT_MIN_RISE = 0.03

# MoveIt joint-goal tolerance. This is the single number behind the measured
# 0/5 re-targeting failure (docs/RESULTS.md), and it is a MARGIN problem, not
# an approach-path problem: the descent path deviates only 2.87 mm from
# vertical, while the clearance between the open gripper and the cylinder
# over the 70 mm descent is just 5.88-7.76 mm.
#
# The pinch point is 0.2367 m from the shoulder and 0.0867 m from the elbow, so
# a goal anywhere inside a 0.02 rad tolerance can place the fingers
#     0.02 * 0.2367 + 0.02 * 0.0867 = 6.47 mm
# from where the IK intended -- at or beyond the entire clearance budget. At
# 0.003 rad the same sum is 0.97 mm. Note arm_controller declares no per-joint
# goal tolerance of its own (coco_controllers.yaml), so this really is the
# binding term.
JOINT_GOAL_TOLERANCE = 0.003

# Robot spawn pose in the Gazebo world (must match full_world_robo.launch.py)
ROBOT_WORLD_X, ROBOT_WORLD_Y = SPAWN_XY


def run_cmd(cmd, timeout):
    """Run a shell-out, turning environment problems into a clear message.

    Every gz / ros2 shell-out raises a bare FileNotFoundError if
    setup_env.sh was not sourced — the most likely first-run mistake, and
    a stack trace tells the reader nothing. Returns None if the command
    could not run or timed out; callers decide whether that is fatal.
    """
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout)
    except FileNotFoundError:
        raise RuntimeError(
            f'`{cmd[0]}` not found on PATH — source setup_env.sh before '
            f'running this.') from None
    except subprocess.TimeoutExpired:
        return None


def model_z(name):
    """
    Read the height of a Gazebo model, or None if it could not be read.

    The physical check behind 'did the grasp actually take'. The magnet's
    own state topic cannot answer that: see the module docstring for the
    measured case where it reports "attached" and welds nothing.
    """
    out = run_cmd(['gz', 'model', '-m', name, '-p'], timeout=10)
    if out is None or out.returncode != 0:
        return None
    # ...  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    #          [0.000000 1.000000 0.800000]
    lines = out.stdout.splitlines()
    for i, line in enumerate(lines):
        if 'Pose' in line and i + 1 < len(lines):
            parts = lines[i + 1].strip().strip('[]').split()
            if len(parts) == 3:
                try:
                    return float(parts[2])
                except ValueError:
                    return None
    return None


def remove_gz_model(name):
    """Delete a model from the running world; missing is not an error."""
    return run_cmd(
        ['gz', 'service', '-s', '/world/coco_world/remove',
         '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '2000', '--req', f'name: "{name}", type: MODEL'],
        timeout=10)


class ArmControl(Node):
    """Everything the arm can do, independent of what it is picking up."""

    def __init__(self, node_name, magnet_model, poses, spun_externally=False):
        super().__init__(node_name)
        # WHETHER SOMETHING ELSE IS ALREADY SPINNING THIS NODE, and it is
        # load-bearing. pick_place.py is a script: it owns its main thread
        # and never calls rclpy.spin(), so the helpers below have to pump
        # callbacks themselves. grasp_server is a node: main() spins it and
        # the sequences run on a worker thread, where a second spin_once
        # dies instantly with "Executor is already spinning" — both
        # rclpy.spin() and rclpy.spin_once() fall back to the GLOBAL
        # executor when no executor is given, so they are the same object.
        #
        # Measured, on the first end-to-end fetch: /grasp/stow returned
        # `error: Executor is already spinning` two seconds in, with the
        # robot parked on the platform and nothing else obviously wrong.
        self.spun_externally = spun_externally
        self.magnet_model = magnet_model
        # The caller's dict, not a copy: pick_place.retarget() rewrites its
        # module-level POSES in place and expects the change to be live.
        self.poses = poses

        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)
        self.scene_client = self.create_client(
            GetPlanningScene, '/get_planning_scene')
        self.grip_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self._gt = None
        self.create_subscription(
            Odometry, '/model/coco/odometry', self._gt_cb, 10)
        self._joints = {}
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)

        self._magnet_pubs = {}
        self._magnet_state = None
        self._holding = False
        self._grasp_z = None      # target height at the moment of the grasp
        self.magnet_attach = None
        self.magnet_detach = None
        if magnet_model is not None:
            self.bind_magnet(magnet_model)

    def bind_magnet(self, model):
        """
        Point the magnet commands at `model`, creating its endpoints once.

        The demo welds one fixed cylinder; the mission welds whichever of
        the four targets was asked for, and only learns which from
        /mission/target_colour. Endpoints are cached per model so a colour
        change mid-session does not stack duplicate subscriptions.

        Bind EARLY, not at the moment of the grasp. The plugin's state
        topic is edge-triggered, so a subscriber created after the command
        misses the transition and then waits out its whole timeout on a
        grasp that actually worked.
        """
        self.magnet_model = model
        if model not in self._magnet_pubs:
            self._magnet_pubs[model] = (
                self.create_publisher(
                    EmptyMsg, f'/magnet/{model}/attach', 10),
                self.create_publisher(
                    EmptyMsg, f'/magnet/{model}/detach', 10))
            self.create_subscription(
                String, f'/magnet/{model}/state',
                lambda msg, m=model: self._magnet_cb_for(m, msg), 10)
        self.magnet_attach, self.magnet_detach = self._magnet_pubs[model]

    def _magnet_cb_for(self, model, msg):
        # Four subscriptions, one state: ignore transitions from a target
        # this node is not currently holding, or releasing the blue one
        # would look like the yellow one letting go.
        if model == self.magnet_model:
            self._magnet_cb(msg)

    # ── spinning, or not ─────────────────────────────────────────────────────
    def pump(self, timeout_sec):
        """Let callbacks run for `timeout_sec`, whoever owns the executor."""
        if self.spun_externally:
            time.sleep(timeout_sec)
        else:
            rclpy.spin_once(self, timeout_sec=timeout_sec)

    def await_future(self, future, timeout_sec, poll=0.02):
        """Wait for a future, returning its result or None on timeout."""
        if not self.spun_externally:
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=timeout_sec)
            return future.result()
        deadline = time.time() + timeout_sec
        while not future.done() and time.time() < deadline and rclpy.ok():
            time.sleep(poll)
        return future.result() if future.done() else None

    # ── inputs ───────────────────────────────────────────────────────────────
    def _gt_cb(self, msg):
        self._gt = msg

    def _joint_cb(self, msg):
        for name, position in zip(msg.name, msg.position):
            self._joints[name] = position

    def _magnet_cb(self, msg):
        self._magnet_state = msg.data.strip().lower()

    # ── magnet grasp ─────────────────────────────────────────────────────────
    def magnet(self, want, required=True):
        """
        Attach or detach the target, and wait for the plugin to confirm it.

        `want` is 'attached' or 'detached'. With `required`, a missing
        confirmation fails the step; without it, it is only logged.

        The distinction exists because the state topic is edge-triggered, so
        silence is ambiguous: it means either "already in that state" or
        "nobody is listening". At spawn time both readings are harmless — if
        the target is already free, that is what we wanted — so that call
        passes required=False. The grasp and the release are load-bearing and
        keep the hard requirement.
        """
        pub = self.magnet_attach if want == 'attached' else self.magnet_detach
        if self._magnet_state == want:
            self._holding = (want == 'attached')
            self.get_logger().info(f'Magnet already {want}')
            return True

        self._magnet_state = None
        deadline = time.time() + MAGNET_TIMEOUT
        # Republished every half second: the bridge and the gz subscriber
        # may not both be up the first time this is called.
        next_send = 0.0
        while time.time() < deadline:
            if time.time() >= next_send:
                pub.publish(EmptyMsg())
                next_send = time.time() + 0.5
            self.pump(0.05)
            if self._magnet_state == want:
                self._holding = (want == 'attached')
                self.get_logger().info(f'Magnet {want}')
                return True

        if not required:
            self.get_logger().info(
                f'Magnet did not report {want!r} — taking it as already '
                f'{want} (the state topic only fires on transitions)')
            return True
        self.get_logger().error(
            f'Magnet did not report {want!r} within {MAGNET_TIMEOUT:.0f}s '
            f'(last state {self._magnet_state!r}). Is the ros_gz_bridge '
            f'running with the /magnet entries, and was the robot spawned '
            f'from the xacro that carries the DetachableJoint plugin?')
        return False

    @property
    def holding(self):
        """Whether the magnet is currently believed to be welded."""
        return self._holding

    def grasp_magnet(self):
        """Note where the target is sitting, then weld it to the palm."""
        self._grasp_z = model_z(self.magnet_model)
        if self._grasp_z is None:
            self.get_logger().warn(
                'could not read the target height before the grasp — the '
                'lift check will be skipped')
        return self.magnet('attached')

    def check_lifted(self):
        """
        Confirm the target physically came up with the arm.

        This is the real grasp test. The magnet's state topic answers
        "attached" even when its binding is stale and it is welding nothing,
        so believing it would reproduce the exact bug this was fixed for
        once already: reporting a completed sequence after lifting air.
        """
        if self._grasp_z is None:
            self.get_logger().warn(
                'no target height recorded at the grasp — lift check skipped')
            return True
        now = model_z(self.magnet_model)
        if now is None:
            self.get_logger().warn(
                'could not read the target height — lift check skipped')
            return True
        rise = now - self._grasp_z
        if rise < LIFT_MIN_RISE:
            self.get_logger().error(
                f'Target did not come up with the arm: z {self._grasp_z:.4f} '
                f'-> {now:.4f} ({rise * 1000:.1f} mm, needed '
                f'{LIFT_MIN_RISE * 1000:.0f} mm). The magnet reported a grasp '
                f'but is holding nothing — if this ran once already in this '
                f'simulator, restart it: the DetachableJoint binds to the '
                f'target only on first spawn.')
            return False
        self.get_logger().info(
            f'Target lifted {rise * 1000:.1f} mm (z {self._grasp_z:.4f} -> '
            f'{now:.4f}) — the grasp is real')
        return True

    def check_robot_pose(self, label, expect_xy=(ROBOT_WORLD_X, ROBOT_WORLD_Y)):
        """Verify the robot's actual world pose (ground-truth odometry)
        matches the pose the scene geometry assumes. MoveIt goals are
        joint-space, so without this a toppled or displaced robot would
        'succeed' while grasping air."""
        for _ in range(20):
            if self._gt is not None:
                break
            self.pump(0.1)
        if self._gt is None:
            self.get_logger().warn(
                'No /model/coco/odometry — robot pose check skipped '
                '(is the ground-truth bridge running?)')
            return True
        p = self._gt.pose.pose.position
        q = self._gt.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                          1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        # expect_xy=None means "wherever it is, just verify it is upright". The
        # mission grasps on the ramp platform ~5.9 m from spawn, where an
        # unconditional position check aborts at pre-flight with a misleading
        # message about resetting the robot.
        if expect_xy is None:
            offset = 0.0
        else:
            offset = math.hypot(p.x - expect_xy[0], p.y - expect_xy[1])
        if offset > 0.05 or abs(roll) > 0.15 or abs(pitch) > 0.15:
            self.get_logger().error(
                f'Robot pose check FAILED ({label}): at ({p.x:.2f}, {p.y:.2f}, '
                f'{p.z:.2f}), roll {roll:.2f}, pitch {pitch:.2f} — expected '
                f'upright{"" if expect_xy is None else f" at {expect_xy}"}. '
                'Reset the robot pose (or restart the sim) first.')
            return False
        self.get_logger().info(f'Robot pose check ok ({label})')
        return True

    # ── planning scene ───────────────────────────────────────────────────────
    @staticmethod
    def box(name, dims, xyz, frame='base_footprint'):
        """A CollisionObject box, in base_footprint coordinates."""
        co = CollisionObject()
        co.header.frame_id = frame
        co.id = name
        prim = SolidPrimitive(type=SolidPrimitive.BOX,
                              dimensions=[float(d) for d in dims])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [
            float(v) for v in xyz]
        pose.orientation.w = 1.0
        co.primitives = [prim]
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD
        return co

    @staticmethod
    def cylinder(name, diameter, height, xyz, frame='base_footprint'):
        """A CollisionObject cylinder — the shape the mission targets are."""
        co = CollisionObject()
        co.header.frame_id = frame
        co.id = name
        prim = SolidPrimitive(type=SolidPrimitive.CYLINDER,
                              dimensions=[float(height), float(diameter) / 2.0])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [
            float(v) for v in xyz]
        pose.orientation.w = 1.0
        co.primitives = [prim]
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD
        return co

    def publish_scene(self, objects, repeats=3, settle=0.3):
        """Add collision objects, republished so move_group cannot miss them."""
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.extend(objects)
        for _ in range(repeats):
            self.scene_pub.publish(scene)
            time.sleep(settle)

    def remove_scene_objects(self, names, repeats=3, settle=0.2):
        """Drop named collision objects a previous run may have left behind."""
        scene = PlanningScene()
        scene.is_diff = True
        for name in names:
            co = CollisionObject()
            co.header.frame_id = 'base_footprint'
            co.id = name
            co.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(co)
        for _ in range(repeats):
            self.scene_pub.publish(scene)
            time.sleep(settle)
        return True

    def allow_gripper_target_contact(self, target='target'):
        """Extend the AllowedCollisionMatrix so the gripper may touch the
        target — grasping requires contact, everything else stays checked."""
        if not self.scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('get_planning_scene service unavailable')
            return False
        req = GetPlanningScene.Request()
        req.components.components = \
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        fut = self.scene_client.call_async(req)
        res = self.await_future(fut, 10.0)
        if res is None:
            # The spin is bounded, so this is reachable whenever move_group
            # is slow or dies mid-run. Dereferencing it raised
            # AttributeError in the middle of a grasp, with the props
            # already spawned and the arm left wherever it stopped.
            self.get_logger().error(
                'get_planning_scene did not reply within 10s — is move_group '
                'still running?')
            return False
        acm = res.scene.allowed_collision_matrix

        names = list(acm.entry_names)
        rows = [list(e.enabled) for e in acm.entry_values]
        for link in GRIPPER_LINKS + [target]:
            if link not in names:
                names.append(link)
                for row in rows:
                    row.append(False)
                rows.append([False] * len(names))
        ti = names.index(target)
        for link in GRIPPER_LINKS:
            li = names.index(link)
            rows[ti][li] = rows[li][ti] = True

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix.entry_names = names
        for row in rows:
            scene.allowed_collision_matrix.entry_values.append(
                AllowedCollisionEntry(enabled=row))
        self.scene_pub.publish(scene)
        time.sleep(0.5)
        self.get_logger().info(f'ACM: gripper links may contact {target}')
        return True

    # ── arm through MoveIt ───────────────────────────────────────────────────
    def move_arm(self, name, speed=0.5):
        """Plan and execute a joint-space goal from `self.poses`."""
        target = self.poses[name]
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'arm'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = speed
        req.max_acceleration_scaling_factor = speed

        constraints = Constraints()
        for joint, position in zip(ARM_JOINTS, target):
            constraints.joint_constraints.append(JointConstraint(
                joint_name=joint, position=float(position),
                tolerance_above=JOINT_GOAL_TOLERANCE,
                tolerance_below=JOINT_GOAL_TOLERANCE, weight=1.0))
        req.goal_constraints = [constraints]

        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info(f"Arm -> '{name}' {target}")
        send = self.move_client.send_goal_async(goal)
        handle = self.await_future(send, 15.0)
        if handle is None or not handle.accepted:
            self.get_logger().error(f"Goal '{name}' not accepted")
            return False
        result_fut = handle.get_result_async()
        result = self.await_future(result_fut, 40.0)
        code = result.result.error_code.val if result else -999
        ok = code == 1   # moveit_msgs/MoveItErrorCodes.SUCCESS
        # rclpy forbids alternating severities at a single call site, so the
        # success and failure logs are separate statements.
        if ok:
            self.get_logger().info(f"Arm '{name}' done (MoveItErrorCode {code})")
        else:
            self.get_logger().error(
                f"Arm '{name}' FAILED (MoveItErrorCode {code})")
        return ok

    # ── gripper through its JTC ──────────────────────────────────────────────
    def move_gripper(self, positions, label, expect_object=False,
                     timeout=GRIP_TIMEOUT):
        """Command the gripper and wait until /joint_states confirms it.

        This used to publish the trajectory, sleep 1.5 s of WALL clock and
        return True unconditionally. Two problems: nothing ever checked
        that the fingers moved, and wall time is not sim time — under an
        unlocked real-time factor or a loaded machine, 'close gripper'
        reported success while the fingers were still moving, the lift then
        grasped air, and the demo printed a completed sequence.

        The two outcomes are physically distinguishable, and measured:
        closing onto the cylinder stalls the fingers at ~0.23 rad, while
        closing on empty air runs all the way to the 0.02 rad setpoint.

        So `expect_object` inverts the success condition. With it True, a
        stall means the object is held and *arriving* at the setpoint
        means we closed on nothing. With it False (opening, which meets no
        obstruction) the fingers must actually arrive.

        With it None, either outcome passes and the observed one is logged.
        That is the right semantics under the magnet grasp: the weld is
        what holds the object, so finger contact is corroborating evidence,
        not the contract.
        """
        traj = JointTrajectory()
        traj.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = DurationMsg(sec=1)
        traj.points = [pt]
        self.grip_pub.publish(traj)
        self.get_logger().info(f'Gripper -> {label}')

        target = dict(zip(GRIPPER_JOINTS, [float(p) for p in positions]))
        deadline = time.time() + timeout
        settled_since = None
        last = None

        while time.time() < deadline:
            self.pump(0.05)
            now = [self._joints.get(j) for j in GRIPPER_JOINTS]
            if any(v is None for v in now):
                continue

            if all(abs(self._joints[j] - target[j]) <= GRIP_TOLERANCE
                   for j in GRIPPER_JOINTS):
                if expect_object:
                    self.get_logger().error(
                        f'Gripper closed to {label} with nothing between the '
                        'fingers — the grasp is empty. Expected the target to '
                        'stall them well short of the setpoint.')
                    return False
                if expect_object is None:
                    self.get_logger().info(
                        f'Gripper reached {label} without touching the target '
                        '— the magnet, not the pinch, is the grasp')
                    return True
                self.get_logger().info(f'Gripper reached {label}')
                return True

            moving = last is None or any(
                abs(a - b) > GRIP_STALL_EPS for a, b in zip(now, last))
            last = now
            if moving:
                settled_since = None
                continue

            # Not at the setpoint and no longer moving.
            settled_since = settled_since or time.time()
            if time.time() - settled_since >= GRIP_STALL_TIME:
                gap = {j: round(self._joints[j], 4) for j in GRIPPER_JOINTS}
                if expect_object or expect_object is None:
                    self.get_logger().info(
                        f'Gripper stalled at {gap} — object in grasp')
                    return True
                self.get_logger().error(
                    f'Gripper stalled at {gap}, commanded {target} — '
                    'nothing should obstruct this move')
                return False

        self.get_logger().error(
            f'Gripper did not reach {label} within {timeout:.0f}s '
            f'(at {[self._joints.get(j) for j in GRIPPER_JOINTS]})')
        return False
