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
pick_place.py
=============
Scripted pick-and-place demo for the Coco arm through MoveIt2.

What it demonstrates:
  1. Real objects (pedestal + cylinder) spawned into Gazebo in front of
     the robot.
  2. Matching collision objects published into the MoveIt planning scene:
     - 'ground' and 'pedestal' are obstacles the planner must avoid;
     - 'target' (the cylinder) is added to the AllowedCollisionMatrix for
       the gripper links, because grasping REQUIRES contact with it.
  3. Joint-space goals planned by OMPL (collision-checked against the
     scene) and executed through the arm_controller
     JointTrajectoryController:
       up -> open -> hover -> (allow contact) -> grasp -> close ->
       raise -> lift -> place -> open -> hover -> home

Requires: full_world_robo.launch.py + move_group.launch.py running, robot
at its spawn pose (the Gazebo object spawn assumes it).
"""

import argparse
import subprocess
import sys
import time

import math

import arm_ik
from coco_config.robot import SPAWN_XY
import rclpy
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty as EmptyMsg, String
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg

ARM_JOINTS = ['m_link1_Revolute-6', 'm_link2_Revolute-7']
GRIPPER_JOINTS = ['m_link3_Revolute-8', 'm_link3_Revolute-9']
GRIPPER_LINKS = ['grip1', 'grip2', 'm_link3']

# Joint-space waypoints (shoulder, elbow), verified in simulation.
# The grasp depth is bounded by arm-vs-chassis self-collision: anything
# deeper than about [0.30, 0.58] clips m_link2/m_link3 into the chassis box
# (probed with /check_state_validity), so the planner rejects it — by design.
# 'hover' is solved analytically (arm_ik) directly above the target: the
# approach descends from there, so the free-swinging transit motions can
# never sweep through the object (it stays a strict collision obstacle
# until the gripper is above it — see run()).
POSES = {
    'home':     [0.0, 0.0],
    'up':       [-1.2, -0.5],
    'grasp':    [0.30, 0.58],
    'raise':    [0.10, 0.45],   # small, mostly-vertical first lift
    'lift':     [-0.3, 0.2],
    'place':    [0.30, 0.58],
}

HOVER_CLEARANCE = 0.07   # pinch point this far above the target for approach


def _run(cmd, timeout):
    """subprocess.run that turns environment problems into a clear message.

    Every gz / ros2 shell-out here raises a bare FileNotFoundError if
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
            f'running the pick-and-place demo.') from None
    except subprocess.TimeoutExpired:
        return None


def _model_z(name):
    """
    Height of a Gazebo model, or None if it could not be read.

    The physical check behind 'did the grasp actually take'. The magnet's
    own state topic cannot answer that: see MAGNET_MODEL for the measured
    case where it reports "attached" and welds nothing.
    """
    out = _run(['gz', 'model', '-m', name, '-p'], timeout=10)
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


def _remove_gz_model(name):
    """Delete a model from the running world; missing is not an error."""
    return _run(
        ['gz', 'service', '-s', '/world/coco_world/remove',
         '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '2000', '--req', f'name: "{name}", type: MODEL'],
        timeout=10)


def _solve_hover(x, z):
    q = arm_ik.ik_or_none(x, z + HOVER_CLEARANCE)
    if q is None:
        raise ValueError(f'no hover pose above ({x:.3f}, {z:.3f})')
    return list(q)


# Solved at import so POSES is complete for every caller (and for the
# joint-limit test that iterates it). The inputs are the verified grasp
# point, so this succeeds today — but if the arm geometry in arm_ik.py is
# ever edited to put it out of reach, record the failure and report it
# from run() rather than raising out of an `import` statement, which no
# caller can catch and which would break test collection.
_HOVER_ERROR = None
try:
    POSES['hover'] = _solve_hover(0.152, 0.128)
except ValueError as exc:
    _HOVER_ERROR = str(exc)

# Gripper motion is confirmed against /joint_states rather than timed.
GRIP_TIMEOUT = 8.0        # s to reach the setpoint before giving up
GRIP_TOLERANCE = 0.02     # rad from the commanded position = "arrived"
GRIP_STALL_EPS = 0.002    # rad of motion between samples that counts as moving
GRIP_STALL_TIME = 0.5     # s of no motion before declaring the fingers stopped

# ── Magnet grasp ────────────────────────────────────────────────────────────
# The grasp is a gz DetachableJoint (coco_robo2.xacro), not friction. Measured
# reason: the arm is 2-DOF with no wrist, the pinch clearance over the 70 mm
# descent is 5.88-7.76 mm, and even after cutting the MoveIt joint tolerance
# from 0.02 to 0.003 rad the docs/RESULTS.md points scored 2/4. The +/-10 mm
# box that was supposed to settle it could not be scored at all: at z = 0.128
# the arm's x reach limit is 0.156, so 4 of its 10 points were unreachable
# before physics ran. Welding on command makes the grasp a decision rather
# than a tolerance stack-up.
#
# MEASURED, and the reason for the detach in spawn_gazebo_objects(): the
# plugin attaches the instant the child model appears, NOT when commanded.
# Verified by spawning the cylinder 1 m to the side at z = 0.80 -- it hung
# there instead of falling, and dropped to z = 0.03 the moment a detach was
# published. Left alone, the demo would drag the target around from spawn.
#
# MEASURED, and the reason this demo needs a FRESH SIM per run: the plugin
# binds to the child entity once, on first sight, and never re-scans. Remove
# pick_target and spawn it again -- which is exactly what clear_scene() does
# between runs -- and the new model is not bound. It then falls freely, no
# state transition is published, and worst of all a later attach command
# still answers "attached" while welding nothing. That last part is a silent
# success: the demo would lift air and report a completed sequence.
#
# So the state topic is necessary but not sufficient, and LIFT_MIN_RISE below
# is the check that actually decides whether the grasp took: the target's own
# height, read out of Gazebo, has to go up when the arm does.
#
# The state topic is also edge-triggered ("attached"/"detached" on
# transitions only), so the subscription has to exist before the command is
# sent, and "no transition" can mean "already in that state" rather than
# "failed" -- hence `required` on magnet() below.
MAGNET_MODEL = 'pick_target'
MAGNET_ATTACH_TOPIC = f'/magnet/{MAGNET_MODEL}/attach'
MAGNET_DETACH_TOPIC = f'/magnet/{MAGNET_MODEL}/detach'
MAGNET_STATE_TOPIC = f'/magnet/{MAGNET_MODEL}/state'
MAGNET_TIMEOUT = 5.0      # s to see the state transition before giving up
# Metres the target must gain between the grasp pose and the top of the
# 'raise'. The arm lifts it far further than this; the threshold only has to
# separate "came with us" from "stayed on the pedestal", and the pedestal is
# 98 mm tall, so anything above sensor noise works.
LIFT_MIN_RISE = 0.03

GRIP_OPEN = [0.5, -0.5]
GRIP_CLOSED = [0.02, -0.02]   # hard pinch: commanded gap well under the 28 mm
                              # cylinder so the position-controlled fingers
                              # keep pressing (effort-limited) through the lift

# Pick scene geometry, in base_link/base_footprint coordinates
PEDESTAL_SIZE = [0.05, 0.05, 0.098]
PEDESTAL_POS = [0.152, 0.0, 0.049]
# MoveIt joint-goal tolerance. This is the single number behind the measured
# 0/5 re-targeting failure (docs/RESULTS.md:106-112), and it is a MARGIN problem,
# not the approach-path problem FUTURE_WORK 5b assumes: the descent path deviates
# only 2.87 mm from vertical, while the clearance between the open gripper and the
# cylinder over the 70 mm descent is just 5.88-7.76 mm.
#
# The pinch point is 0.2367 m from the shoulder and 0.0867 m from the elbow, so a
# goal anywhere inside the tolerance can place the fingers
#     0.02 * 0.2367 + 0.02 * 0.0867 = 6.47 mm
# from where the IK intended -- at or beyond the entire clearance budget, including
# at the one target that works. 4/4 there and 0/5 nearby was a coin flip that
# happened to land the same way four times.
#
# At 0.003 rad the same sum is 0.97 mm, which fits inside the budget with room to
# spare. Note arm_controller declares no per-joint goal tolerance of its own
# (coco_controllers.yaml), so this constraint really is the binding term.
JOINT_GOAL_TOLERANCE = 0.003

# Planning-scene proxy for the target. Must match the SPAWNED cylinder below
# (radius 0.014 -> 0.028 diameter). It was 0.03, i.e. 2 mm of phantom padding that
# MoveIt reserved out of a ~7 mm clearance budget.
TARGET_SIZE = [0.028, 0.028, 0.06]
TARGET_POS = [0.152, 0.0, 0.128]

# Robot spawn pose in the Gazebo world (must match full_world_robo.launch.py)
ROBOT_WORLD_X, ROBOT_WORLD_Y = SPAWN_XY

PEDESTAL_SDF = """
<sdf version="1.8"><model name="pick_pedestal"><static>true</static>
  <link name="link">
    <collision name="c"><geometry><box><size>0.05 0.05 0.098</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>0.05 0.05 0.098</size></box></geometry>
      <material><ambient>0.4 0.4 0.45 1</ambient><diffuse>0.4 0.4 0.45 1</diffuse></material></visual>
  </link></model></sdf>"""

TARGET_SDF = """
<sdf version="1.8"><model name="pick_target">
  <link name="link">
    <inertial><mass>0.02</mass>
      <inertia><ixx>8e-6</ixx><iyy>8e-6</iyy><izz>3e-6</izz>
               <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <collision name="c"><geometry><cylinder><radius>0.014</radius><length>0.06</length></cylinder></geometry>
      <surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction></surface></collision>
    <visual name="v"><geometry><cylinder><radius>0.014</radius><length>0.06</length></cylinder></geometry>
      <material><ambient>0.85 0.1 0.1 1</ambient><diffuse>0.85 0.1 0.1 1</diffuse></material></visual>
  </link></model></sdf>"""


class PickPlace(Node):
    def __init__(self):
        super().__init__('pick_place_demo')
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.grip_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self._gt = None
        self.create_subscription(
            Odometry, '/model/coco/odometry', self._gt_cb, 10)
        self._joints = {}
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)

        # Magnet. The state subscription is created here, at startup, rather
        # than around each command: the topic is edge-triggered, so a
        # subscriber that appears after the command misses the transition
        # and then waits out its whole timeout on a grasp that worked.
        self.magnet_attach = self.create_publisher(
            EmptyMsg, MAGNET_ATTACH_TOPIC, 10)
        self.magnet_detach = self.create_publisher(
            EmptyMsg, MAGNET_DETACH_TOPIC, 10)
        self._magnet_state = None
        self._holding = False
        self._grasp_z = None      # target height at the moment of the grasp
        self.create_subscription(
            String, MAGNET_STATE_TOPIC, self._magnet_cb, 10)

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
            rclpy.spin_once(self, timeout_sec=0.05)
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

    def grasp_magnet(self):
        """Note where the target is sitting, then weld it to the palm."""
        self._grasp_z = _model_z(MAGNET_MODEL)
        if self._grasp_z is None:
            self.get_logger().warn(
                'could not read the target height before the grasp — the '
                'lift check will be skipped')
        return self.magnet('attached')

    def check_lifted(self):
        """
        Confirm the target physically came up with the arm.

        This is the real grasp test. The magnet's state topic answers
        "attached" even when its binding is stale and it is welding nothing
        (see MAGNET_MODEL), so believing it would reproduce the exact bug
        this demo was fixed for once already: reporting a completed
        sequence after lifting air.
        """
        if self._grasp_z is None:
            self.get_logger().warn(
                'no target height recorded at the grasp — lift check skipped')
            return True
        now = _model_z(MAGNET_MODEL)
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
                f'but is holding nothing — if this demo already ran once in '
                f'this simulator, restart it: the DetachableJoint binds to '
                f'the target only on first spawn.')
            return False
        self.get_logger().info(
            f'Target lifted {rise * 1000:.1f} mm (z {self._grasp_z:.4f} -> '
            f'{now:.4f}) — the grasp is real')
        return True

    def check_robot_pose(self, label, expect_xy=(ROBOT_WORLD_X, ROBOT_WORLD_Y)):
        """Verify the robot's actual world pose (ground-truth odometry)
        matches the spawn pose the scene geometry assumes. MoveIt goals are
        joint-space, so without this a toppled or displaced robot would
        'succeed' while grasping air."""
        for _ in range(20):
            if self._gt is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
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
                f'upright at ({ROBOT_WORLD_X}, {ROBOT_WORLD_Y}). Reset the '
                'robot pose (or restart the sim) before running the demo.')
            return False
        self.get_logger().info(f'Robot pose check ok ({label})')
        return True

    # ── gazebo objects ───────────────────────────────────────────────────────
    def spawn_gazebo_objects(self):
        """Spawn the physical pedestal + cylinder in front of the robot.

        Returns True only if both entities were created. A failed spawn
        used to be logged and then discarded, so the demo carried on and
        closed the gripper on empty air before reporting success.
        """
        # Remove leftovers from a previous run so the demo is re-runnable
        for name in ('pick_pedestal', 'pick_target'):
            _remove_gz_model(name)
        time.sleep(0.5)
        all_ok = True
        for name, sdf, pos in [
            ('pick_pedestal', PEDESTAL_SDF, PEDESTAL_POS),
            ('pick_target', TARGET_SDF, TARGET_POS),
        ]:
            cmd = ['ros2', 'run', 'ros_gz_sim', 'create',
                   '-name', name, '-string', sdf,
                   '-x', str(ROBOT_WORLD_X + pos[0]),
                   '-y', str(ROBOT_WORLD_Y + pos[1]),
                   '-z', str(pos[2])]
            out = _run(cmd, timeout=30)
            # Exit status is the contract; the log line is a human-readable
            # confirmation that a future ros_gz release is free to reword.
            # Requiring only the string would turn a wording change into
            # "spawn failed" on a working sim.
            ok = out is not None and out.returncode == 0
            if ok:
                self.get_logger().info(f'Gazebo spawn {name}: ok')
            else:
                detail = (out.stderr.strip()[:160] if out is not None
                          else 'command did not run')
                self.get_logger().error(f'Gazebo spawn {name} FAILED: {detail}')
                all_ok = False
        if not all_ok:
            return False
        # The DetachableJoint welds the target to the palm as soon as the
        # model appears — see MAGNET_MODEL above. Without this the cylinder
        # is carried along by every subsequent arm move and the demo grasps
        # a target it is already holding. Not required: on any run after the
        # first in a given simulator the plugin never bound to this model, so
        # it is already free and there is no transition to observe.
        return self.magnet('detached', required=False)

    def clear_scene(self):
        """Remove scene objects a previous (possibly aborted) run left in
        move_group's planning scene, and leftover Gazebo models — makes the
        demo re-runnable and keeps stale boxes from blocking the start state."""
        scene = PlanningScene()
        scene.is_diff = True
        for name in ('ground', 'pedestal', 'target'):
            co = CollisionObject()
            co.header.frame_id = 'base_footprint'
            co.id = name
            co.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(co)
        for _ in range(3):
            self.scene_pub.publish(scene)
            time.sleep(0.2)
        for name in ('pick_pedestal', 'pick_target'):
            _remove_gz_model(name)
        return True

    def stage_scene(self):
        """Spawn the Gazebo objects and mirror them into the planning scene
        (called once the arm is clear of the spawn spot).

        Aborts the demo if the props never appeared — grasping at nothing
        and declaring success is worse than stopping.
        """
        if not self.spawn_gazebo_objects():
            return False
        self.add_scene_objects()
        return True

    # ── planning scene ───────────────────────────────────────────────────────
    @staticmethod
    def _box(name, dims, xyz):
        co = CollisionObject()
        co.header.frame_id = 'base_footprint'
        co.id = name
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(d) for d in dims])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [float(v) for v in xyz]
        pose.orientation.w = 1.0
        co.primitives = [prim]
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD
        return co

    def add_scene_objects(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(
            self._box('ground', [2.0, 2.0, 0.02], [0.0, 0.0, -0.012]))
        scene.world.collision_objects.append(
            self._box('pedestal', PEDESTAL_SIZE, PEDESTAL_POS))
        scene.world.collision_objects.append(
            self._box('target', TARGET_SIZE, TARGET_POS))
        for _ in range(3):
            self.scene_pub.publish(scene)
            time.sleep(0.3)
        self.get_logger().info('Planning scene: added ground, pedestal, target')

    def allow_gripper_target_contact(self):
        """Extend the AllowedCollisionMatrix so the gripper may touch the
        target — grasping requires contact, everything else stays checked."""
        if not self.scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('get_planning_scene service unavailable')
            return False
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        fut = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
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
        for link in GRIPPER_LINKS + ['target']:
            if link not in names:
                names.append(link)
                for row in rows:
                    row.append(False)
                rows.append([False] * len(names))
        ti = names.index('target')
        for link in GRIPPER_LINKS:
            li = names.index(link)
            rows[ti][li] = rows[li][ti] = True

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix.entry_names = names
        for row in rows:
            from moveit_msgs.msg import AllowedCollisionEntry
            scene.allowed_collision_matrix.entry_values.append(
                AllowedCollisionEntry(enabled=row))
        self.scene_pub.publish(scene)
        time.sleep(0.5)
        self.get_logger().info('ACM: gripper links may contact target')
        return True

    # ── arm through MoveIt ───────────────────────────────────────────────────
    def move_arm(self, name, speed=0.5):
        target = POSES[name]
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
        rclpy.spin_until_future_complete(self, send, timeout_sec=15.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"Goal '{name}' not accepted")
            return False
        result_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut, timeout_sec=40.0)
        result = result_fut.result()
        code = result.result.error_code.val if result else -999
        ok = code == 1   # moveit_msgs/MoveItErrorCodes.SUCCESS
        # rclpy forbids alternating severities at a single call site, so the
        # success and failure logs are separate statements.
        if ok:
            self.get_logger().info(f"Arm '{name}' done (MoveItErrorCode {code})")
        else:
            self.get_logger().error(f"Arm '{name}' FAILED (MoveItErrorCode {code})")
        return ok

    # ── gripper through its JTC ──────────────────────────────────────────────
    def move_gripper(self, positions, label, expect_object=False,
                     timeout=GRIP_TIMEOUT):
        """Command the gripper and wait until /joint_states confirms it.

        This used to publish the trajectory, sleep 1.5 s of WALL clock and
        return True unconditionally. Two problems: nothing ever checked
        that the fingers moved, and wall time is not sim time — under an
        unlocked real-time factor (train_ppo --fast sets that globally on
        this world) or a loaded machine, 'close gripper' reported success
        while the fingers were still moving, the lift then grasped air,
        and the demo printed "Pick-and-place sequence complete".

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
        not the contract. Demanding a stall would fail a grasp that the
        magnet holds perfectly well when the fingers happen to close
        cleanly either side of the cylinder; demanding arrival would fail
        the ones where they touch it.
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
            rclpy.spin_once(self, timeout_sec=0.05)
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

    # ── demo sequence ────────────────────────────────────────────────────────
    def run(self):
        if _HOVER_ERROR:
            self.get_logger().error(
                f'{_HOVER_ERROR} — the default hover pose is unreachable with '
                f'the current arm geometry in arm_ik.py')
            return False
        if not self.move_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('move_group action server unavailable')
            return False
        if not self.check_robot_pose('pre-flight'):
            return False
        self.clear_scene()

        # Ordering matters:
        # 1. The arm moves up BEFORE the objects appear — at 'home' the
        #    gripper volume overlaps the target's spawn spot, which would
        #    put the start state in collision.
        # 2. The target then stays a strict collision obstacle for the
        #    transit to 'hover': the planner must route the open gripper
        #    AROUND it, not through it. Contact is only allowed once the
        #    gripper hovers directly above, so the grasp is a short, clean
        #    descent instead of a free-form swing into the object.
        steps = [
            ('move up', lambda: self.move_arm('up')),
            ('open gripper', lambda: self.move_gripper(GRIP_OPEN, 'open')),
            ('stage scene objects', self.stage_scene),
            ('hover above target', lambda: self.move_arm('hover')),
            ('allow gripper-target contact', self.allow_gripper_target_contact),
            ('grasp approach', lambda: self.move_arm('grasp', speed=0.1)),
            # expect_object=None: with the magnet, whether the fingers stall
            # on the cylinder or close past it is corroborating detail, not
            # the pass/fail condition. The weld is.
            ('close gripper',
             lambda: self.move_gripper(GRIP_CLOSED, 'closed',
                                       expect_object=None)),
            # Attach with the gripper already down at the grasp pose, so the
            # weld freezes the object where it actually sits rather than
            # snapping it to some nominal offset.
            ('magnet attach', self.grasp_magnet),
            ('raise', lambda: self.move_arm('raise', speed=0.05)),
            # The grasp is only proven once the target has moved with us.
            ('confirm lift', self.check_lifted),
            ('lift', lambda: self.move_arm('lift', speed=0.08)),
            ('place', lambda: self.move_arm('place', speed=0.15)),
            # Release order matters: detach first, then open. Opening while
            # still welded leaves the object hanging in mid-air under the
            # palm, which reads as a successful place until the arm moves.
            ('magnet detach', lambda: self.magnet('detached')),
            ('release', lambda: self.move_gripper(GRIP_OPEN, 'open')),
            ('retreat above target', lambda: self.move_arm('hover')),
            ('home', lambda: self.move_arm('home')),
        ]
        for label, step in steps:
            if not step():
                self.get_logger().error(f"Step '{label}' failed — aborting demo")
                # Drop what we are holding, but do NOT open the gripper: on
                # the mission the abort can happen mid-carry on the ramp
                # platform, where an unplanned finger motion is a second
                # failure on top of the first. Detaching leaves the object
                # under gravity where it is; the fingers stay put.
                if self._holding:
                    self.get_logger().warn(
                        'still holding the target — detaching before abort')
                    self.magnet('detached')
                return False
        if not self.check_robot_pose('post-run'):
            return False
        self.get_logger().info('✅ Pick-and-place sequence complete')
        return True


def retarget(x, z):
    """Re-aim the pick at a Cartesian pinch point (x, z) in base_footprint:
    solve the joint poses analytically (arm_ik) and move the spawned
    pedestal/cylinder so the physical scene matches. The default scene is
    exactly retarget(0.152, 0.128)."""
    grasp = arm_ik.ik_or_none(x, z)
    if grasp is None:
        print(f'target ({x:.3f}, {z:.3f}) is unreachable within joint limits',
              file=sys.stderr)
        return False
    try:
        hover = _solve_hover(x, z)
    except ValueError:
        print(f'no hover pose above ({x:.3f}, {z:.3f})', file=sys.stderr)
        return False
    # cylinder centre sits at the pinch height; pedestal fills the gap below
    ped_h = z - TARGET_SIZE[2] / 2
    if ped_h <= 0.0:
        # A non-positive height makes an invalid SDF box that would be
        # handed straight to Gazebo. Reject before mutating POSES, so a
        # bad --target leaves the module state untouched.
        print(f'pick height z={z:.3f} is below the cylinder radius '
              f'({TARGET_SIZE[2] / 2:.3f}) — no pedestal would fit',
              file=sys.stderr)
        return False
    POSES['grasp'] = POSES['place'] = list(grasp)
    POSES['hover'] = hover
    PEDESTAL_SIZE[2] = ped_h
    PEDESTAL_POS[:] = [x, 0.0, ped_h / 2]
    TARGET_POS[:] = [x, 0.0, z]
    global PEDESTAL_SDF
    PEDESTAL_SDF = PEDESTAL_SDF.replace('0.05 0.05 0.098',
                                        f'0.05 0.05 {ped_h:.4f}')
    print(f'IK target ({x:.3f}, {z:.3f}): grasp={[round(v, 4) for v in grasp]} '
          f'hover={[round(v, 4) for v in hover]} pedestal_h={ped_h:.3f}')
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--target', nargs=2, type=float, metavar=('X', 'Z'),
                    help='Cartesian pinch point in base_footprint (default: '
                         f'{TARGET_POS[0]} {TARGET_POS[2]}, the verified pose)')
    args = ap.parse_args()
    if args.target and not retarget(*args.target):
        sys.exit(2)

    rclpy.init()
    node = PickPlace()
    ok = False
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
