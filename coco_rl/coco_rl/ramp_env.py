"""
ramp_env.py
===========
Gymnasium environment for Coco ramp traversal in Gazebo Harmonic.

The env drives the *running* simulation (launch full_world_robo.launch.py
gui:=false first) through ROS 2:

  action       : [linear_x, angular_z]  (continuous, scaled to limits)
  observation  : [x, y, sin(yaw), cos(yaw), v, w, roll, pitch]
                 planar pose/velocity from /diff_drive_controller/odom,
                 attitude from /imu
  reward       : forward progress toward/up the ramp (+x), minus penalties
                 for tilt and for tipping over (terminal)
  reset        : teleports the robot back to the start pose with the
                 Gazebo /world/<world>/set_pose service and re-zeroes odom
                 by tracking the offset

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

WORLD = 'coco_world'
START_POSE = (-2.0, 0.0, 0.03)   # world frame
MAX_LIN, MAX_ANG = 0.6, 1.2
STEP_DT = 0.1                    # agent control period (sim seconds ~ wall)
TIP_LIMIT = 0.6                  # rad
GOAL_X_PROGRESS = 3.0            # odom-frame forward progress that ends the episode


def quat_to_rp(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


class CocoRampEnv(gym.Env):
    """Minimal viable RL environment: assumes the sim is already running."""

    metadata = {'render_modes': []}

    def __init__(self):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()
        self.node: Node = rclpy.create_node('coco_ramp_env')

        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        high = np.array([np.inf] * 8, dtype=np.float32)
        self.observation_space = gym.spaces.Box(-high, high, dtype=np.float32)

        self._odom = None
        self._imu = None
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(
            Odometry, '/diff_drive_controller/odom', self._odom_cb, 10)
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

    def _imu_cb(self, msg):
        self._imu = msg

    def _spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _publish(self, lin, ang):
        m = TwistStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.twist.linear.x = float(lin)
        m.twist.angular.z = float(ang)
        self._cmd.publish(m)

    def _state(self):
        o, i = self._odom, self._imu
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
        # Teleport the robot back to the start with the gz set_pose service
        subprocess.run(
            ['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req',
             f'name: "coco", position: {{x: {START_POSE[0]}, y: {START_POSE[1]}, '
             f'z: {START_POSE[2]}}}, orientation: {{w: 1.0}}'],
            capture_output=True, timeout=15)
        self._spin(1.0)   # let physics settle and fresh odom/imu arrive
        while self._odom is None or self._imu is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        # odom does not reset on teleport — rebase it
        self._x0 = self._odom.pose.pose.position.x
        self._y0 = self._odom.pose.pose.position.y
        self._prev_x = 0.0
        self._steps = 0
        return self._state(), {}

    def step(self, action):
        lin = float(np.clip(action[0], -1, 1)) * MAX_LIN
        ang = float(np.clip(action[1], -1, 1)) * MAX_ANG
        self._publish(lin, ang)
        self._spin(STEP_DT)

        s = self._state()
        x, roll, pitch = s[0], s[6], s[7]

        progress = x - self._prev_x
        self._prev_x = x
        reward = 10.0 * progress - 0.05 * (abs(roll) + abs(pitch)) - 0.01

        tipped = abs(roll) > TIP_LIMIT or abs(pitch) > TIP_LIMIT
        reached = x >= GOAL_X_PROGRESS
        if tipped:
            reward -= 10.0
        if reached:
            reward += 20.0

        self._steps += 1
        terminated = tipped or reached
        truncated = self._steps >= self.max_steps
        if terminated or truncated:
            self._publish(0.0, 0.0)
        return s, float(reward), terminated, truncated, {}

    def close(self):
        self._publish(0.0, 0.0)
        self.node.destroy_node()
