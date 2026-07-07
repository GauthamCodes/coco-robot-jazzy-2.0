"""
ramp_env.py
===========
Gymnasium environment for Coco ramp traversal in Gazebo Harmonic.

The env drives the *running* simulation (launch full_world_robo.launch.py
gui:=false first) through ROS 2:

  action       : [linear_x, angular_z]  (continuous, scaled to limits)
  observation  : [x, y, sin(yaw), cos(yaw), v, w, roll, pitch]
                 planar pose from ground-truth odometry
                 (/model/coco/odometry, gz OdometryPublisher) when it is
                 being published, else wheel odometry
                 (/diff_drive_controller/odom — under-reads on the ramp
                 slope); attitude from /imu
  reward       : forward progress toward/up the ramp (+x), minus penalties
                 for tilt and for tipping over (terminal) — see reward.py
  reset        : teleports the robot back to the start pose with the
                 Gazebo /world/<world>/set_pose service and re-zeroes the
                 pose by tracking the offset
  stepping     : each step waits for STEP_DT of *simulation* time (from
                 odometry stamps), so training also works when the physics
                 real-time factor is unlocked above 1

Episodes end on tip-over (|roll| or |pitch| > 0.6 rad), on reaching the
ramp top region, or after max_steps.
"""

import math
import subprocess
import time

import gymnasium as gym
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from coco_rl.reward import is_tipped, reached_goal, step_reward

WORLD = 'coco_world'
START_POSE = (-2.0, 0.0, 0.03)   # world frame
MAX_LIN, MAX_ANG = 0.6, 1.2
STEP_DT = 0.1                    # agent control period, in SIM seconds

# Domain-randomization ranges (opt-in via CocoRampEnv(randomize=True)):
# lateral offset and approach yaw at spawn, so the policy can't overfit
# a single dead-straight run at the ramp.
RAND_Y = 0.5                     # +/- m
RAND_YAW = 0.4                   # +/- rad


def sample_start_pose(rng):
    """Sample a randomized start pose (x, y, z, yaw) from an np RNG."""
    y = START_POSE[1] + float(rng.uniform(-RAND_Y, RAND_Y))
    yaw = float(rng.uniform(-RAND_YAW, RAND_YAW))
    return START_POSE[0], y, START_POSE[2], yaw


def quat_to_rp(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class CocoRampEnv(gym.Env):
    """Minimal viable RL environment: assumes the sim is already running."""

    metadata = {'render_modes': []}

    def __init__(self, randomize=False):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()
        self.node: Node = rclpy.create_node('coco_ramp_env')
        self.randomize = randomize

        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        high = np.array([np.inf] * 8, dtype=np.float32)
        self.observation_space = gym.spaces.Box(-high, high, dtype=np.float32)

        self._odom = None       # wheel odometry (always present)
        self._gt = None         # ground-truth odometry (if plugin is bridged)
        self._imu = None
        self._use_gt = False
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(
            Odometry, '/diff_drive_controller/odom', self._odom_cb, 10)
        self.node.create_subscription(
            Odometry, '/model/coco/odometry', self._gt_cb, 10)
        self.node.create_subscription(Imu, '/imu', self._imu_cb, qos)
        self._cmd = self.node.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)

        self._x0 = 0.0
        self._y0 = 0.0
        self._prev_x = 0.0
        self._steps = 0
        self.max_steps = 400

    # ── ROS plumbing ─────────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        self._odom = msg

    def _gt_cb(self, msg):
        self._gt = msg

    def _imu_cb(self, msg):
        self._imu = msg

    def _pose_msg(self):
        return self._gt if self._use_gt else self._odom

    def _spin_sim(self, sim_seconds):
        """Spin until SIM time (odometry stamps) advances by sim_seconds.
        Robust to real_time_factor != 1; wall-clock deadline as a safety
        net in case the sim is paused/dead."""
        t_start = None
        deadline = time.time() + max(5.0, sim_seconds * 20)
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            src = self._pose_msg()
            if src is None:
                continue
            t = _stamp(src)
            if t_start is None:
                t_start = t
            elif t - t_start >= sim_seconds:
                return True
        return False

    def _publish(self, lin, ang):
        m = TwistStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.twist.linear.x = float(lin)
        m.twist.angular.z = float(ang)
        self._cmd.publish(m)

    def _state(self):
        o, i = self._pose_msg(), self._imu
        p = o.pose.pose
        x = p.position.x - self._x0
        y = p.position.y - self._y0
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        v = o.twist.twist.linear.x
        w = o.twist.twist.angular.z
        iq = i.orientation
        roll, pitch = quat_to_rp(iq.x, iq.y, iq.z, iq.w)
        return np.array([x, y, math.sin(yaw), math.cos(yaw), v, w, roll, pitch],
                        dtype=np.float32)

    # ── gym API ──────────────────────────────────────────────────────────────
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._publish(0.0, 0.0)
        if self.randomize:
            x, y, z, yaw = sample_start_pose(self.np_random)
        else:
            (x, y, z), yaw = START_POSE, 0.0
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        # Teleport the robot back to the start with the gz set_pose service
        subprocess.run(
            ['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req',
             f'name: "coco", position: {{x: {x}, y: {y}, z: {z}}}, '
             f'orientation: {{z: {qz}, w: {qw}}}'],
            capture_output=True, timeout=15)
        while self._odom is None or self._imu is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        # prefer ground-truth pose when the OdometryPublisher is bridged
        self._use_gt = self._gt is not None
        self._spin_sim(0.5)   # let physics settle and fresh odom/imu arrive
        # pose does not reset on teleport — rebase it
        src = self._pose_msg()
        self._x0 = src.pose.pose.position.x
        self._y0 = src.pose.pose.position.y
        self._prev_x = 0.0
        self._steps = 0
        return self._state(), {}

    def step(self, action):
        lin = float(np.clip(action[0], -1, 1)) * MAX_LIN
        ang = float(np.clip(action[1], -1, 1)) * MAX_ANG
        self._publish(lin, ang)
        self._spin_sim(STEP_DT)

        s = self._state()
        x, roll, pitch = s[0], s[6], s[7]

        progress = x - self._prev_x
        self._prev_x = x

        tipped = is_tipped(roll, pitch)
        reached = reached_goal(x)
        reward = step_reward(progress, roll, pitch, tipped, reached)

        self._steps += 1
        terminated = tipped or reached
        truncated = self._steps >= self.max_steps
        if terminated or truncated:
            self._publish(0.0, 0.0)
        return s, reward, terminated, truncated, {}

    def close(self):
        self._publish(0.0, 0.0)
        self.node.destroy_node()
