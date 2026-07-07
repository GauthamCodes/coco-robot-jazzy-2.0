#!/usr/bin/env python3
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
from shape_msgs.msg import SolidPrimitive
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


def _solve_hover(x, z):
    q = arm_ik.ik_or_none(x, z + HOVER_CLEARANCE)
    if q is None:
        raise ValueError(f'no hover pose above ({x:.3f}, {z:.3f})')
    return list(q)


POSES['hover'] = _solve_hover(0.152, 0.128)

GRIP_OPEN = [0.5, -0.5]
GRIP_CLOSED = [0.02, -0.02]   # hard pinch: commanded gap well under the 28 mm
                              # cylinder so the position-controlled fingers
                              # keep pressing (effort-limited) through the lift

# Pick scene geometry, in base_link/base_footprint coordinates
PEDESTAL_SIZE = [0.05, 0.05, 0.098]
PEDESTAL_POS = [0.152, 0.0, 0.049]
TARGET_SIZE = [0.03, 0.03, 0.06]
TARGET_POS = [0.152, 0.0, 0.128]

# Robot spawn pose in the Gazebo world (must match full_world_robo.launch.py)
ROBOT_WORLD_X, ROBOT_WORLD_Y = -2.0, 0.0

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

    def _gt_cb(self, msg):
        self._gt = msg

    def check_robot_pose(self, label):
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
        offset = math.hypot(p.x - ROBOT_WORLD_X, p.y - ROBOT_WORLD_Y)
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
        """Spawn the physical pedestal + cylinder in front of the robot."""
        # Remove leftovers from a previous run so the demo is re-runnable
        for name in ('pick_pedestal', 'pick_target'):
            subprocess.run(
                ['gz', 'service', '-s', '/world/coco_world/remove',
                 '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000', '--req', f'name: "{name}", type: MODEL'],
                capture_output=True, timeout=10)
        time.sleep(0.5)
        for name, sdf, pos in [
            ('pick_pedestal', PEDESTAL_SDF, PEDESTAL_POS),
            ('pick_target', TARGET_SDF, TARGET_POS),
        ]:
            cmd = ['ros2', 'run', 'ros_gz_sim', 'create',
                   '-name', name, '-string', sdf,
                   '-x', str(ROBOT_WORLD_X + pos[0]),
                   '-y', str(ROBOT_WORLD_Y + pos[1]),
                   '-z', str(pos[2])]
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            ok = 'Entity creation successful' in out.stdout + out.stderr
            self.get_logger().info(f'Gazebo spawn {name}: {"ok" if ok else out.stderr.strip()[:120]}')

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
            subprocess.run(
                ['gz', 'service', '-s', '/world/coco_world/remove',
                 '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000', '--req', f'name: "{name}", type: MODEL'],
                capture_output=True, timeout=10)
        return True

    def stage_scene(self):
        """Spawn the Gazebo objects and mirror them into the planning scene
        (called once the arm is clear of the spawn spot)."""
        self.spawn_gazebo_objects()
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
        acm = fut.result().scene.allowed_collision_matrix

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
                tolerance_above=0.02, tolerance_below=0.02, weight=1.0))
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
    def move_gripper(self, positions, label):
        traj = JointTrajectory()
        traj.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = DurationMsg(sec=1)
        traj.points = [pt]
        self.grip_pub.publish(traj)
        self.get_logger().info(f'Gripper -> {label}')
        time.sleep(1.5)
        return True

    # ── demo sequence ────────────────────────────────────────────────────────
    def run(self):
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
            ('close gripper', lambda: self.move_gripper(GRIP_CLOSED, 'closed')),
            ('raise', lambda: self.move_arm('raise', speed=0.05)),
            ('lift', lambda: self.move_arm('lift', speed=0.08)),
            ('place', lambda: self.move_arm('place', speed=0.15)),
            ('release', lambda: self.move_gripper(GRIP_OPEN, 'open')),
            ('retreat above target', lambda: self.move_arm('hover')),
            ('home', lambda: self.move_arm('home')),
        ]
        for label, step in steps:
            if not step():
                self.get_logger().error(f"Step '{label}' failed — aborting demo")
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
    POSES['grasp'] = POSES['place'] = list(grasp)
    POSES['hover'] = hover
    # cylinder centre sits at the pinch height; pedestal fills the gap below
    ped_h = z - TARGET_SIZE[2] / 2
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
