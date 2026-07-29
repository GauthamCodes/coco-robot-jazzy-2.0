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
     JointTrajectoryController. The sixteen steps run() actually performs:
       up -> open -> stage scene -> hover -> (allow contact) -> grasp ->
       close -> MAGNET ATTACH -> raise -> CONFIRM LIFT -> lift -> place ->
       MAGNET DETACH -> release -> hover -> home
     The two capitalised pairs are the grasp and its proof: the weld is
     what holds the object (see MAGNET_MODEL), and the object's own height
     read out of Gazebo is what says the weld took (see check_lifted).

Requires: full_world_robo.launch.py + move_group.launch.py running, robot
at its spawn pose (the Gazebo object spawn assumes it).

The mission's grasp is `grasp_server.py`, not this: same arm plumbing
(arm_control.py) aimed at an object that is already in the world, at a
pose perception measured, with no pedestal.
"""

import argparse
import sys
import time

import arm_ik
from arm_control import (
    ARM_JOINTS,
    ArmControl,
    GRIP_CLOSED,
    GRIP_OPEN,
    GRIP_STALL_EPS,
    GRIP_STALL_TIME,
    GRIP_TIMEOUT,
    GRIP_TOLERANCE,
    GRIPPER_JOINTS,
    GRIPPER_LINKS,
    JOINT_GOAL_TOLERANCE,
    LIFT_MIN_RISE,
    MAGNET_TIMEOUT,
    model_z,
    remove_gz_model,
    ROBOT_WORLD_X,
    ROBOT_WORLD_Y,
    run_cmd,
)
from coco_config.robot import GRASP_HOVER_CLEARANCE
import rclpy

# Re-exported so `pick_place.GRIP_OPEN` and friends keep resolving for
# test_pick_poses.py and for anyone reading this file first. The
# definitions live in arm_control.py, which is also what grasp_server.py
# uses; two copies of a joint tolerance is exactly the kind of drift that
# made the re-targeting failure so hard to see.
_SHARED = (ARM_JOINTS, GRIPPER_JOINTS, GRIPPER_LINKS, GRIP_OPEN, GRIP_CLOSED,
           GRIP_TIMEOUT, GRIP_TOLERANCE, GRIP_STALL_EPS, GRIP_STALL_TIME,
           JOINT_GOAL_TOLERANCE, MAGNET_TIMEOUT, LIFT_MIN_RISE,
           ROBOT_WORLD_X, ROBOT_WORLD_Y)

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

# The one source of truth, shared with coco_config's reach test and the
# mission grasp: the descent's start height is what bounds the approach
# window (GRASP_REACH_X_MAX), so it cannot live in one script.
HOVER_CLEARANCE = GRASP_HOVER_CLEARANCE


# _run, _model_z and _remove_gz_model moved to arm_control.py, which the
# mission grasp server shares. Aliased so the rest of this file (and its
# comments about them) still read as written.
_run = run_cmd
_model_z = model_z
_remove_gz_model = remove_gz_model


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
# Which model this demo welds. arm_control derives the three /magnet
# topics from it; the mission grasp server passes target_<colour> instead.
MAGNET_MODEL = 'pick_target'
MAGNET_ATTACH_TOPIC = f'/magnet/{MAGNET_MODEL}/attach'
MAGNET_DETACH_TOPIC = f'/magnet/{MAGNET_MODEL}/detach'
MAGNET_STATE_TOPIC = f'/magnet/{MAGNET_MODEL}/state'

# Pick scene geometry, in base_link/base_footprint coordinates
PEDESTAL_SIZE = [0.05, 0.05, 0.098]
PEDESTAL_POS = [0.152, 0.0, 0.049]
# Planning-scene proxy for the target. Must match the SPAWNED cylinder below
# (radius 0.014 -> 0.028 diameter). It was 0.03, i.e. 2 mm of phantom padding that
# MoveIt reserved out of a ~7 mm clearance budget.
TARGET_SIZE = [0.028, 0.028, 0.06]
TARGET_POS = [0.152, 0.0, 0.128]

PEDESTAL_SDF = """
<sdf version="1.8"><model name="pick_pedestal"><static>true</static>
  <link name="link">
    <collision name="c"><geometry><box><size>0.05 0.05 0.098</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>0.05 0.05 0.098</size></box></geometry>
      <material><ambient>0.4 0.4 0.45 1</ambient>
        <diffuse>0.4 0.4 0.45 1</diffuse></material></visual>
  </link></model></sdf>"""

TARGET_SDF = """
<sdf version="1.8"><model name="pick_target">
  <link name="link">
    <inertial><mass>0.02</mass>
      <inertia><ixx>8e-6</ixx><iyy>8e-6</iyy><izz>3e-6</izz>
               <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <collision name="c"><geometry>
        <cylinder><radius>0.014</radius><length>0.06</length></cylinder></geometry>
      <surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction></surface></collision>
    <visual name="v"><geometry>
        <cylinder><radius>0.014</radius><length>0.06</length></cylinder></geometry>
      <material><ambient>0.85 0.1 0.1 1</ambient>
        <diffuse>0.85 0.1 0.1 1</diffuse></material></visual>
  </link></model></sdf>"""


class PickPlace(ArmControl):
    """The Demo 4 scene: spawn a pedestal and a cylinder, then pick them."""

    def __init__(self):
        # POSES is passed, not copied: retarget() rewrites it in place and
        # move_arm() has to see that.
        super().__init__('pick_place_demo', MAGNET_MODEL, POSES)

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
        self.remove_scene_objects(('ground', 'pedestal', 'target'))
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

    def add_scene_objects(self):
        """Mirror the spawned pedestal and cylinder into the planning scene."""
        self.publish_scene([
            self.box('ground', [2.0, 2.0, 0.02], [0.0, 0.0, -0.012]),
            self.box('pedestal', PEDESTAL_SIZE, PEDESTAL_POS),
            self.box('target', TARGET_SIZE, TARGET_POS),
        ])
        self.get_logger().info('Planning scene: added ground, pedestal, target')

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
