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

from coco_config.robot import RAMP_FOOT_X, RAMP_SUMMIT_X, SPAWN_XY
import gymnasium as gym
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from coco_rl.reward import (episode_outcome, is_tipped, reached_goal,
                            step_reward)

WORLD = 'coco_world'
# X/Y are the launch-file spawn point (coco_config.robot.SPAWN_XY). Z is
# deliberately NOT coco_config.robot.SPAWN_Z: that value is a drop height
# for spawning into an empty world, whereas this is a teleport onto ground
# the robot is already resting on. Changing it would shift every episode's
# initial condition and invalidate the published learning curve.
START_POSE = (SPAWN_XY[0], SPAWN_XY[1], 0.03)   # world frame

# The goal is the ramp *summit*, expressed as forward progress from the spawn.
# The env rebases x to the spawn origin at every reset, so this is
# (summit world x) - (spawn world x). Derived from coco_config.robot so it
# tracks the ramp geometry and the launch file automatically — reaching the
# foot is no longer mistaken for success. The episode terminates at the crest,
# so the robot never drives off the wedge's vertical back face.
# Stop GOAL_MARGIN short of the exact crest. The wedge's back face is vertical,
# so a robot whose base reaches the crest x is already pitching over the drop —
# and `is_tipped` (0.6 rad) fires on that pitch BEFORE x crosses the goal. It
# was measured: a policy climbing at a gentle 0.17 m/s reached x = 5.4838 and
# was recorded as `tipped`, 1.6 cm short of the 5.5 m goal. That made a
# successful climb indistinguishable from a fall, and is why the 180k-step
# curriculum logged 1 goal in 1,399 episodes while best returns reached +64.
#
# The docstring above always claimed the episode "terminates at the crest, so
# the robot never drives off the wedge's vertical back face". This is what
# actually makes that true. 0.3 m keeps the finish line firmly on the slope,
# well past the 2.5 m of climbing that constitutes the task.
GOAL_MARGIN = 0.3
GOAL_SUMMIT = RAMP_SUMMIT_X - SPAWN_XY[0] - GOAL_MARGIN

# Action scaling. MAX_LIN was 0.6 and is now 0.4, for two measured reasons.
#
# 1. Speed: driving the env with a *constant* full-throttle action reached the
#    summit in 384 steps, while half throttle (0.3 m/s) reached it in 187 — an
#    average of 0.14 m/s versus 0.29 m/s. Above ~0.4 m/s the wheels slip on the
#    grade, so the top of the range was both destabilising and slower.
# 2. Stability: see ACTION_TAU below.
# MAX_ANG was 1.2 and is now 0.5. Once the linear channel became forward-only
# the policy learned to hold full throttle — and constant full throttle with
# zero yaw summits 3/3 through this very env — but PPO still tipped 121/141
# episodes at ~0.5 m on that run, and +/-1.2 rad/s is far more authority than
# "drive straight up a ramp" needs. (That run turned out to be confounded by a
# degraded simulator, so the 121/141 figure is not clean evidence for this
# change -- but the cap is still right on its own terms.)
# 0.5 rad/s still turns the robot 90 deg in ~3 s, which is ample for the
# steering corrections this task actually requires.
MAX_LIN, MAX_ANG = 0.4, 0.5

# Floor on the linear channel. Measured speed sweep (constant action, fresh sim,
# 12 deg wedge): 0.00 and 0.05 m/s time out without reaching the ramp, 0.10 m/s
# times out at x=4.38, and every speed from 0.17 m/s upward reaches the goal 2/2
# (282 steps at 0.17, down to 122 at 0.40) with peak pitch exactly 12.0 deg --
# i.e. the ramp grade and nothing more. So the bottom of the range is not merely
# useless, it is the only part of the range that cannot finish, and exploring it
# just modulates speed. Mapping action[0] onto [MIN_LIN, MAX_LIN] keeps every
# commandable speed a winning one.
MIN_LIN = 0.15

# Where the twist goes. A constructor kwarg rather than a launch remap
# because this env calls rclpy.init() with no args and creates its node
# directly, so `remappings=` in a launch file is silently ignored. The
# default publishes straight to the controller, so training and evaluation
# behave exactly as measured; the mission's ramp driver passes
# '/cmd_vel_rl' to go through cmd_vel_arbiter instead.
CMD_VEL_TOPIC = '/diff_drive_controller/cmd_vel'

# First-order low-pass on the commanded twist, in SIM seconds. A stochastic
# policy samples a fresh action every STEP_DT (0.1 s), so an untrained one
# commands near-instantaneous reversals of the full linear range. Measured: with
# random actions the robot tipped 8/8 within ~37-46 steps, pitching nose-down
# progressively (-16 deg -> -37 deg over five steps) while moving barely 6 cm,
# and it kept going to -74 deg once left alone. It really falls over. Disabling
# yaw entirely did NOT help (still 8/8), so the culprit is the linear
# oscillation, not steering. The diff_drive_controller's own acceleration limits
# (2.0 m/s^2, verified enabled at runtime) are not enough on their own.
#
# Meanwhile a single constant action solves the task in 187 steps, so the task
# was never hard — PPO simply never survived long enough to find it. Smoothing
# the command removes that failure mode without touching the reward or the goal,
# and it is what a real robot's velocity controller does anyway.
#
# Note the observation already carries measured v and w, which track the
# smoothed command closely, so the filter state does not make the MDP
# meaningfully non-Markovian.
ACTION_TAU = 0.3
STEP_DT = 0.1                    # agent control period, in SIM seconds
SIM_WAIT_TIMEOUT = 15.0          # s to wait for odom/imu before giving up

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


def gz_service(service, reqtype, reptype, req, timeout_ms=3000, attempts=1):
    """Call a Gazebo transport service; return True iff it replied true.

    Wraps the `gz` CLI so a missing binary produces an actionable message
    instead of a bare FileNotFoundError traceback — forgetting to source
    setup_env.sh is the most likely first-run mistake.

    `attempts` retries a call that timed out or answered false. Each call
    spawns a short-lived `gz service` process which binds an ephemeral
    gz-transport node, and under the CPU load of --fast (unlimited RTF) the
    round trip occasionally overruns `timeout_ms`. The CLI then exits, and
    Gazebo logs `NodeShared::RecvSrvRequest() error sending response: Host
    unreachable` when it finally tries to reply to a process that is gone.
    That is transient, so callers whose request is *idempotent* — set_pose
    and set_physics both are, they carry absolute values — should retry
    instead of treating the first miss as fatal. A 180k-step curriculum was
    lost to exactly this: three phases each died at a random reset (after
    132, 67 and 130 episodes) while the simulator was still perfectly
    healthy and answering the very next service call.
    """
    cmd = ['gz', 'service', '-s', service,
           '--reqtype', reqtype, '--reptype', reptype,
           '--timeout', str(timeout_ms), '--req', req]
    for attempt in range(max(1, attempts)):
        if attempt:
            time.sleep(0.5)   # let the transport settle before retrying
        try:
            out = subprocess.run(cmd, capture_output=True, text=True,
                                 timeout=max(15, timeout_ms / 1000 * 5))
        except FileNotFoundError:
            # A missing binary will never succeed; do not burn the retries.
            raise RuntimeError(
                '`gz` not found on PATH — source setup_env.sh (or install '
                'Gazebo Harmonic) before running coco_rl.') from None
        except subprocess.TimeoutExpired:
            continue
        if 'true' in (out.stdout + out.stderr).lower():
            return True
    return False


def _rclpy_call(fn, *args, **kwargs):
    """Run an rclpy call, reporting a torn-down context as one exception.

    rclpy installs its own SIGINT handler and invalidates the context, so on
    Ctrl-C these calls raise either ExternalShutdownException or a raw
    RCLError about an invalid context/publisher/timer, depending on where the
    teardown lands (RCLError has no public import path). Callers only need to
    handle ExternalShutdownException. Anything raised while the context is
    still healthy is a real bug and propagates unchanged.
    """
    if not rclpy.ok():
        raise ExternalShutdownException
    try:
        return fn(*args, **kwargs)
    except Exception:
        if rclpy.ok():
            raise
        raise ExternalShutdownException from None


class CocoRampEnv(gym.Env):
    """Minimal viable RL environment: assumes the sim is already running."""

    metadata = {'render_modes': []}

    def __init__(self, randomize=False, start_progress=0.0,
                 cmd_vel_topic=CMD_VEL_TOPIC):
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
        self._closed = False
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(
            Odometry, '/diff_drive_controller/odom', self._odom_cb, 10)
        self.node.create_subscription(
            Odometry, '/model/coco/odometry', self._gt_cb, 10)
        self.node.create_subscription(Imu, '/imu', self._imu_cb, qos)
        self._cmd = self.node.create_publisher(TwistStamped, cmd_vel_topic, 10)

        self._x0 = 0.0
        self._y0 = 0.0
        self._prev_x = 0.0
        self._steps = 0
        self.max_steps = 400
        # Smoothed command carried between steps (see ACTION_TAU).
        self._lin_cmd = 0.0
        self._ang_cmd = 0.0
        # Reverse-curriculum start offset: metres along +x from the canonical
        # spawn to begin the episode. The goal stays fixed at the summit, so a
        # larger offset is a strictly easier episode. Clamped to the flat run
        # before the ramp foot, because starting *on* the wedge would need the
        # surface height and matching pitch, and dropping the robot onto a
        # slope from a flat attitude just makes it tumble.
        self.start_progress = float(
            min(max(0.0, start_progress), RAMP_FOOT_X - SPAWN_XY[0] - 0.1))

    # ── ROS plumbing ─────────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        self._odom = msg

    def _gt_cb(self, msg):
        self._gt = msg

    def _imu_cb(self, msg):
        self._imu = msg

    def _pose_msg(self):
        return self._gt if self._use_gt else self._odom

    def _spin(self, timeout_sec):
        _rclpy_call(rclpy.spin_once, self.node, timeout_sec=timeout_sec)

    def _spin_sim(self, sim_seconds):
        """Spin until SIM time (odometry stamps) advances by sim_seconds.
        Robust to real_time_factor != 1; a real-time deadline as a safety
        net in case the sim is paused/dead.

        The deadline uses `time.monotonic()`, never `time.time()`. The wall
        clock is not monotonic: on this dual-boot machine NTP corrected it
        backward by 5h30m on one boot (Windows leaves local time in the RTC
        while Linux reads it as UTC). A *forward* correction mid-run would
        expire every deadline at once, and each expiry truncates the episode
        as 'sim_stalled' — silently turning the rest of a multi-hour run into
        training on fabricated transitions. Suspend/resume does the same
        thing. monotonic() cannot be moved by either.
        """
        t_start = None
        deadline = time.monotonic() + max(5.0, sim_seconds * 20)
        while time.monotonic() < deadline:
            self._spin(0.02)
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
        _rclpy_call(self._cmd.publish, m)

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
        x += self.start_progress
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        # Teleport the robot back to the start with the gz set_pose service.
        # A failed teleport must not be silent: reset() rebases the episode
        # origin onto wherever the robot currently is, so a robot still lying
        # on its side at the ramp top would look like a fresh episode at the
        # start line and quietly poison the run.
        # Retried and given a longer window: this call runs once per episode,
        # so over a multi-hour run a 3s single-shot is a near-certain crash
        # (see gz_service). Re-sending the same absolute pose is harmless
        # even if the first request did land and only its reply was lost.
        if not gz_service(
                f'/world/{WORLD}/set_pose', 'gz.msgs.Pose', 'gz.msgs.Boolean',
                f'name: "coco", position: {{x: {x}, y: {y}, z: {z}}}, '
                f'orientation: {{z: {qz}, w: {qw}}}',
                timeout_ms=5000, attempts=5):
            raise RuntimeError(
                f'set_pose failed for world {WORLD!r} — is the sim running '
                f'and is the model named "coco"?')

        deadline = time.monotonic() + SIM_WAIT_TIMEOUT   # see _spin_sim
        while self._odom is None or self._imu is None:
            if time.monotonic() > deadline:
                missing = [n for n, v in (('/diff_drive_controller/odom',
                                           self._odom), ('/imu', self._imu))
                           if v is None]
                raise RuntimeError(
                    f'no {" or ".join(missing)} after {SIM_WAIT_TIMEOUT:.0f}s '
                    f'— is the sim running?')
            self._spin(0.1)
        # prefer ground-truth pose when the OdometryPublisher is bridged
        self._use_gt = self._gt is not None
        if not self._spin_sim(0.5):   # let physics settle, fresh odom/imu
            raise RuntimeError(
                'simulation time did not advance while settling after reset '
                '— is the sim paused or dead?')
        # Rebase progress on the CANONICAL spawn x, not on wherever the robot
        # actually is. With a reverse-curriculum start offset the two differ on
        # purpose, and rebasing onto the actual pose would move the goal along
        # with the start — making every stage equally hard and defeating the
        # whole point. Observation x is therefore "progress from spawn", which
        # also tells the policy how far along it already is.
        src = self._pose_msg()
        self._x0 = SPAWN_XY[0]
        self._y0 = src.pose.pose.position.y
        self._prev_x = src.pose.pose.position.x - self._x0
        self._steps = 0
        # Zero the command filter too: carrying the last episode's velocity
        # into a freshly teleported robot would have it lurch on step 1.
        self._lin_cmd = 0.0
        self._ang_cmd = 0.0
        return self._state(), {}

    def step(self, action):
        # action[0] maps to [0, MAX_LIN], NOT [-MAX_LIN, MAX_LIN]. Reversing is
        # never useful for "drive up the ramp", and the measured failure mode was
        # precisely forward/reverse oscillation: random full-range linear
        # commands reared the chassis nose-up and flipped it backwards 8/8 times
        # in ~40 steps, and disabling yaw entirely did not help.
        #
        # Removing the reverse half fixes both halves of the exploration bind
        # that the earlier attempts ran into. With sigma large enough to explore
        # (log_std_init -1.0) the robot tipped; with sigma small enough to
        # survive (-2.0) it never discovered forward drive at all and just bled
        # the time penalty — returns degraded from -5.0 to -16.2 over one run.
        # Under this mapping a *zero-mean* action is already 0.5 * MAX_LIN of
        # forward motion, so exploration explores steering and speed while
        # always making progress, and there is no reversal to pump the robot
        # over. The dense progress term then has something to sharpen from
        # step one.
        lin_t = MIN_LIN + (float(np.clip(action[0], -1, 1)) + 1.0) / 2.0 \
            * (MAX_LIN - MIN_LIN)
        ang_t = float(np.clip(action[1], -1, 1)) * MAX_ANG
        # Low-pass the command toward the policy's target (see ACTION_TAU).
        # alpha = dt / (tau + dt) is the standard first-order discrete blend, so
        # the smoothing is defined in seconds rather than in "per step" units
        # and stays correct if STEP_DT changes.
        alpha = STEP_DT / (ACTION_TAU + STEP_DT)
        self._lin_cmd += alpha * (lin_t - self._lin_cmd)
        self._ang_cmd += alpha * (ang_t - self._ang_cmd)
        self._publish(self._lin_cmd, self._ang_cmd)
        # A stalled sim returns the previous observation unchanged, so
        # progress computes to ~0 and the agent trains on fabricated
        # transitions. End the episode instead of learning from garbage.
        if not self._spin_sim(STEP_DT):
            self._publish(0.0, 0.0)
            return (self._state(), 0.0, False, True,
                    {'outcome': 'sim_stalled'})

        s = self._state()
        x, roll, pitch = s[0], s[6], s[7]

        progress = x - self._prev_x
        self._prev_x = x

        tipped = is_tipped(roll, pitch)
        reached = reached_goal(x, goal_x=GOAL_SUMMIT)
        reward = step_reward(progress, roll, pitch, tipped, reached)

        self._steps += 1
        terminated = tipped or reached
        truncated = self._steps >= self.max_steps
        if terminated or truncated:
            self._publish(0.0, 0.0)
        outcome = episode_outcome(tipped, reached, truncated)
        return s, reward, terminated, truncated, (
            {'outcome': outcome} if outcome else {})

    def close(self):
        """Idempotent, and safe to call after rclpy has already shut down.

        Both Monitor.close() and the training/eval scripts call this. On
        Ctrl-C rclpy's signal handler tears the context down first, so
        publishing the stop command raises RCLError — which then masks the
        real exception and, in train_ppo, skipped the model save entirely.
        Guard on rclpy.ok() and only stop the robot when the context is
        still live (if it isn't, the controller's cmd_vel watchdog stops it
        anyway).
        """
        if self._closed:
            return
        self._closed = True
        if rclpy.ok():
            self._publish(0.0, 0.0)
            self.node.destroy_node()
