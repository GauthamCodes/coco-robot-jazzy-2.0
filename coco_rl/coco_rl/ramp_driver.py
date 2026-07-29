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
ramp_driver — the RL policy as a mission service.

`evaluate.py` is a script: it owns its loop, runs N episodes and exits.
A mission needs something that sits there and drives one ramp segment
when asked. Same policy, same environment, different shape.

Nav2 drives the flat ground and this drives the ramp. That split is
forced by geometry, not preference: the lidar plane sits at 0.2135 m and
an 18 degree slope intersects it at world x=1.657, so from the flat the
ramp scans as a solid wall. Nav2 cannot plan onto something its costmap
says is an obstacle, and `allow_unknown: false` (M3) means it will not
route through the unmapped space behind it either.

Services (all ``std_srvs/Trigger``)
-----------------------------------
``/ramp/climb``    run the PPO policy from here to the summit
``/ramp/descend``  scripted heading-hold down the far slope
``/ramp/stop``     abort whatever is running and command zero

``/ramp/status`` (``std_msgs/String``, 5 Hz) reports as space-separated
key=value, the same shape ``cmd_vel_arbiter`` uses, so the web panel can
split it without a JSON parser.

Two things here are load-bearing
--------------------------------
- **Output goes to ``/cmd_vel_rl``, passed as a CONSTRUCTOR argument.**
  ``ramp_env`` calls ``rclpy.init()`` with no args and builds its node
  directly, so a launch-file ``remappings=`` is accepted and then
  silently ignored. A remap here would look right and do nothing.
- **``teleport=False``.** ``CocoRampEnv.reset()`` normally snaps the robot
  to the start line with ``set_pose``. Mid-mission that would teleport it
  off the pre-ramp pose Nav2 just drove to, back across the arena.

This node deliberately does **not** publish ``/mission/mode``. The panel
asserts the mode at 2 Hz and the arbiter latches it; a second publisher
would fight it. Mode belongs to the sequencer.

- **The climb runs the policy inside a lane hold.** The policy trained
  from a fixed spawn, so it holds a line but cannot correct one, and the
  heading error a Nav2 leg is allowed to finish with (0.25 rad) becomes
  0.58 m of lateral over a 2.5 m climb — off the edge of a 2.5 m-wide
  platform for the outermost lane. ``lateral_hold`` corrects the yaw
  action toward the lane centreline and takes the worst case to 0.05 m;
  ``lateral_hold:=false`` reproduces the bare policy the baseline was
  measured on. The gains are ROS parameters so both arms of that
  comparison can run against one simulator.

Observation distribution is preserved for free: ``reset()`` rebases x on
the canonical spawn (``SPAWN_XY[0]``), so handing off at world x=0.5 gives
observation x=2.5 rising to 5.0 at the summit — the exact range the policy
trained on. It rebases y on the *current* pose, so starting in lane +0.75
gives observation y=0, also in-distribution.
"""

import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from coco_rl.ramp_env import CocoRampEnv, MAX_ANG, quat_to_rp

# Where the RL segment publishes. The arbiter forwards this to the wheels
# only while /mission/mode is 'rl'.
RL_CMD_VEL_TOPIC = '/cmd_vel_rl'

STATUS_HZ = 5.0

# ── descent ────────────────────────────────────────────────────────────────
# Scripted, not a second policy. Driving down a slope in a straight line is
# a heading-hold controller; the RL story is fully told by the climb
# (traction, slip, pitch recovery) and a second training run would cost
# hours while adding nothing to it.
#
# map_drive.steer() was the obvious thing to reuse and is the wrong shape:
# it stops forward motion and turns in place whenever heading error exceeds
# its gate. On a downslope that is exactly what you must not do — the robot
# is being accelerated by gravity and a stationary pivot on a grade is how
# a skid-steer base loses its footing.
DESCEND_SPEED = 0.18       # m/s commanded down the slope
DESCEND_YAW_GAIN = 1.2     # rad/s per rad of heading error
DESCEND_YAW_CLAMP = 0.4    # rad/s ceiling on the correction
DESCEND_ARRIVE = 0.15      # m from the goal x that counts as arrived
DESCEND_TIMEOUT = 90.0     # s of wall clock before giving up
DESCEND_HZ = 20.0          # command rate; inside the arbiter 0.3 s timeout
TIP_LIMIT = 0.6            # rad of |roll| or |pitch|; matches ramp_env


def descend_cmd(yaw, x, goal_x):
    """
    Velocity command for the scripted descent, as (linear, angular, done).

    Pure: no ROS, no state, so the interesting cases are testable. Holds
    yaw at zero (straight down the fall line, +x) with a clamped
    proportional correction, and never commands a stationary pivot — the
    linear term does not depend on heading error.

    `yaw` is the heading in radians, `x` the current world x, `goal_x` the
    flat ground past the bottom of the slope.
    """
    remaining = goal_x - x
    if remaining <= DESCEND_ARRIVE:
        return 0.0, 0.0, True
    # Wrap to (-pi, pi] so a robot at +179 deg corrects 2 deg the short
    # way rather than 358 the long way.
    err = math.atan2(math.sin(-yaw), math.cos(-yaw))
    ang = max(-DESCEND_YAW_CLAMP,
              min(DESCEND_YAW_CLAMP, DESCEND_YAW_GAIN * err))
    return DESCEND_SPEED, ang, False


# ── lane hold on the climb ─────────────────────────────────────────────────
# The policy has no closed-loop lateral control. It was trained from a FIXED
# spawn where a near-constant action solves the task, so it never had to
# learn to steer, and the drift it leaves is systematic rather than noisy:
# +0.61 m measured in M4, +0.59 m in M5. The four target lanes are 0.5 m
# apart on a platform 2.5 m wide, so that is not a near miss — sent to
# yellow's lane at +0.75 the robot would arrive at +1.35, which is 0.10 m
# PAST the platform edge at 1.25.
#
# This corrects the policy's yaw action toward the lane centreline instead
# of retraining it. That is the same shape ramp_driver's own descend_cmd
# already uses on the down-slope, which is the evidence that a heading-hold
# works on a grade; the cross-track term is the addition.
#
# WHERE THE +0.6 m ACTUALLY COMES FROM, and it is not the policy steering
# badly. Measured: teleport to the pre-ramp pose at exactly yaw 0 and the
# bare policy climbs 2.5 m with +0.03 m of drift. Give it the heading error
# a Nav2 leg is allowed to finish with -- nav2_params.yaml sets
# yaw_goal_tolerance to 0.25 rad -- and the same policy drifts +0.58 m,
# because 2.5 m of climb at 0.25 rad IS 0.64 m of lateral. The policy holds
# a line; what it cannot do is correct one, having trained from a fixed
# spawn where it never had to.
#
# That is also why it is dangerous rather than merely inaccurate. Open
# loop, the outer lanes end up 1.17-1.18 m out on a platform that ends at
# 1.25 m, and 2 of those 8 climbs never summited at all.
#
# The gains were swept, not guessed. Linearising about the centreline with
# v = 0.4 m/s and the env's MAX_ANG = 0.5 rad/s,
#
#     y'' = -v*MAX_ANG*K_Y*y - MAX_ANG*K_YAW*y'
#     w_n = sqrt(v*MAX_ANG*K_Y),  zeta = MAX_ANG*K_YAW / (2*w_n)
#
# and the climb only lasts ~6 s, so bandwidth is the whole problem. Worst
# residual over both signs of a 0.25 rad start heading:
#
#     K_Y  K_YAW   w_n    zeta   worst drift
#     ---  -----   ----   ----   -----------
#     off  off      -      -       +0.582 m   (2/8 climbs failed)
#     1.2  1.6     0.49   0.82     +0.218 m
#     3.0  2.5     0.77   0.81     +0.053 m   <-- shipped
#     5.0  3.2     1.00   0.80     +0.061 m   (overshoots: sign flips)
#     8.0  4.0     1.26   0.79     +0.107 m
#
# The sign flip above 3.0 is the tell that this is a real minimum and not
# noise: past it the loop crosses the centreline before the climb ends.
# All 8 climbs reached the goal at every gain setting tried.
#
# LATERAL_CLAMP is a safety term rather than a tuning one: it bounds how far
# the correction can move the action away from the distribution the policy
# trained on. The clamp was swept too (0.4 / 0.8 / 1.2 / 2.0 moved the
# residual by 6 mm), which is what proved the limit was bandwidth and not
# authority. 0.8 sits just above the 0.625 peak the shipped gains actually
# ask for.
LATERAL_GAIN = 3.0      # action units per metre of drift
HEADING_GAIN = 2.5      # action units per radian of heading error
LATERAL_CLAMP = 0.8     # ceiling on the correction, in action units


def lateral_hold(action, y_err, yaw, gain=LATERAL_GAIN,
                 heading_gain=HEADING_GAIN, clamp=LATERAL_CLAMP):
    """
    Bias a policy action's yaw channel back toward the lane centreline.

    Pure, so the cases that matter — a correction of the right sign, a
    saturated policy action staying inside the action space, the clamp
    holding — are asserted without a simulator or a trained model.

    `action` is the policy's [linear, angular] in [-1, 1]; `y_err` is metres
    of drift from the lane the segment started in (observation index 1,
    positive to the left); `yaw` is heading error in radians, positive to
    the left. The linear channel is never touched: slowing down on a grade
    is how a skid-steer base loses traction, and speed is the policy's
    business.
    """
    correction = -(gain * float(y_err) + heading_gain * float(yaw))
    correction = max(-clamp, min(clamp, correction))
    angular = max(-1.0, min(1.0, float(action[1]) + correction))
    return [float(action[0]), angular]


def format_status(segment, step, progress, lateral, pitch, outcome):
    """
    One-line driver state, space-separated key=value for the panel.

    `lateral` is drift from the lane the segment started in. It is here
    because the mission depends on it: the four targets sit in lanes
    0.5 m apart on a ramp only 2.5 m wide, so arriving at the top in the
    wrong lane is a failed fetch, and the climb is the only place that
    drift can accumulate.
    """
    return (f'segment={segment} step={step} progress={progress:.2f} '
            f'lateral={lateral:+.2f} pitch={pitch:.3f} '
            f'outcome={outcome or "none"}')


class RampDriver(Node):
    """Runs one ramp segment on request and reports what it is doing."""

    def __init__(self):
        super().__init__('ramp_driver')
        self.declare_parameter('model', '')
        self.declare_parameter('cmd_vel_topic', RL_CMD_VEL_TOPIC)
        self.declare_parameter('descend_goal_x', 6.8)
        self.declare_parameter('status_topic', '/ramp/status')
        # Off reproduces the bare policy, which is how the +0.59 m baseline
        # was measured and how any claim that this helps stays falsifiable.
        self.declare_parameter('lateral_hold', True)
        self.declare_parameter('lateral_gain', LATERAL_GAIN)
        self.declare_parameter('heading_gain', HEADING_GAIN)
        self.declare_parameter('lateral_clamp', LATERAL_CLAMP)

        self._model_path = self.get_parameter('model').value
        self._descend_goal_x = float(
            self.get_parameter('descend_goal_x').value)

        self._env = None
        self._model = None
        self._lock = threading.Lock()
        self._abort = threading.Event()
        self._busy = False

        self.segment = 'idle'
        self.step = 0
        self.progress = 0.0
        self.lateral = 0.0
        self.pitch = 0.0
        self.outcome = None
        self._peak_correction = 0.0
        self._lateral_hold = bool(self.get_parameter('lateral_hold').value)

        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self.create_service(Trigger, '/ramp/climb', self._on_climb)
        self.create_service(Trigger, '/ramp/descend', self._on_descend)
        self.create_service(Trigger, '/ramp/stop', self._on_stop)
        self.create_timer(1.0 / STATUS_HZ, self._publish_status)

        self.get_logger().info(
            f'ramp_driver ready; policy {self._model_path or "<none>"} -> '
            f'{self.get_parameter("cmd_vel_topic").value}, lane hold '
            f'{"on" if self._lateral_hold else "OFF"}. Set /mission/mode to '
            f'"rl" or the arbiter will not forward this.')

    # ── plumbing ─────────────────────────────────────────────────────────
    def _publish_status(self):
        self._status_pub.publish(String(data=format_status(
            self.segment, self.step, self.progress, self.lateral,
            self.pitch, self.outcome)))

    def _ensure_env(self):
        """Build the env and load the policy on first use."""
        if self._env is None:
            self._env = CocoRampEnv(
                cmd_vel_topic=self.get_parameter('cmd_vel_topic').value,
                teleport=False)
        if self._model is None and self._model_path:
            from stable_baselines3 import PPO
            self._model = PPO.load(self._model_path)
        return self._env

    def _reject(self, why):
        self.get_logger().error(why)
        return Trigger.Response(success=False, message=why)

    # ── services ─────────────────────────────────────────────────────────
    def _on_climb(self, request, response):
        del request
        if self._busy:
            return self._reject(f'busy running {self.segment}')
        if not self._model_path:
            return self._reject(
                'no policy: start with -p model:=<path to the .zip>')
        threading.Thread(target=self._run_climb, daemon=True).start()
        response.success = True
        response.message = 'climb started; watch /ramp/status'
        return response

    def _on_descend(self, request, response):
        del request
        if self._busy:
            return self._reject(f'busy running {self.segment}')
        threading.Thread(target=self._run_descend, daemon=True).start()
        response.success = True
        response.message = 'descend started; watch /ramp/status'
        return response

    def _on_stop(self, request, response):
        del request
        self._abort.set()
        if self._env is not None:
            self._env._publish(0.0, 0.0)
        self.segment, self.outcome = 'idle', 'stopped'
        response.success = True
        response.message = 'stopping'
        return response

    # ── segments ─────────────────────────────────────────────────────────
    def _run_climb(self):
        with self._lock:
            self._busy = True
            self._abort.clear()
            self.segment, self.outcome, self.step = 'climb', None, 0
            try:
                # Re-read per segment, not once at start-up: the whole
                # value of an A/B knob is being able to run both arms of
                # the comparison against ONE simulator, and relaunching
                # between them changes the thing being measured.
                self._lateral_hold = bool(
                    self.get_parameter('lateral_hold').value)
                gain = float(self.get_parameter('lateral_gain').value)
                heading_gain = float(self.get_parameter('heading_gain').value)
                clamp = float(self.get_parameter('lateral_clamp').value)
                env = self._ensure_env()
                obs, _ = env.reset()
                # Peak correction over the segment, so "the lane hold did
                # something" is a number in the log rather than an
                # impression from watching the robot.
                self._peak_correction = 0.0
                while not self._abort.is_set():
                    action, _ = self._model.predict(obs, deterministic=True)
                    if self._lateral_hold:
                        # obs[1] is drift from the lane this segment started
                        # in: reset() rebases _y0 on the CURRENT pose, so it
                        # is zero at the first step by construction and the
                        # lane the sequencer navigated to is the datum.
                        held = lateral_hold(
                            action, y_err=float(obs[1]),
                            yaw=math.atan2(float(obs[2]), float(obs[3])),
                            gain=gain, heading_gain=heading_gain,
                            clamp=clamp)
                        self._peak_correction = max(
                            self._peak_correction,
                            abs(held[1] - float(action[1])))
                        action = held
                    obs, _, terminated, truncated, info = env.step(action)
                    self.step += 1
                    self.progress = float(obs[0])
                    self.lateral = float(obs[1])
                    self.pitch = float(obs[7])
                    if terminated or truncated:
                        self.outcome = info.get('outcome', 'unknown')
                        break
                else:
                    self.outcome = 'stopped'
                env._publish(0.0, 0.0)
            except Exception as exc:                      # noqa: BLE001
                # A segment that dies must not take the node with it: the
                # sequencer is waiting on /ramp/status and a dead node
                # looks identical to a slow climb.
                self.outcome = f'error: {exc}'
                self.get_logger().error(f'climb failed: {exc}')
            finally:
                self.segment = 'idle'
                self._busy = False
        self.get_logger().info(
            f'climb finished: {self.outcome} after {self.step} steps, '
            f'progress {self.progress:.2f} m, lateral {self.lateral:+.2f} m; '
            f'lane hold {"on" if self._lateral_hold else "OFF"}, peak yaw '
            f'correction {self._peak_correction:.3f} action units '
            f'({self._peak_correction * MAX_ANG:.3f} rad/s)')

    def _run_descend(self):
        with self._lock:
            self._busy = True
            self._abort.clear()
            self.segment, self.outcome, self.step = 'descend', None, 0
            try:
                env = self._ensure_env()
                # No reset(): the robot is already standing on the platform
                # where the climb left it, and reset() would settle physics
                # and rebase the episode for no reason.
                env._spin(0.2)
                deadline = time.time() + DESCEND_TIMEOUT
                while not self._abort.is_set():
                    pose = env._pose_msg().pose.pose
                    q = pose.orientation
                    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z))
                    iq = env._imu.orientation
                    roll, self.pitch = quat_to_rp(iq.x, iq.y, iq.z, iq.w)
                    lin, ang, done = descend_cmd(
                        yaw, pose.position.x, self._descend_goal_x)
                    self.progress = pose.position.x
                    self.step += 1
                    if done:
                        self.outcome = 'goal'
                        break
                    # Same tip threshold the env terminates on. Coasting
                    # down a slope on a pitching chassis is the one place
                    # this script can hurt the robot.
                    if abs(roll) > TIP_LIMIT or abs(self.pitch) > TIP_LIMIT:
                        self.outcome = 'tipped'
                        break
                    if time.time() > deadline:
                        self.outcome = 'timeout'
                        break
                    env._publish(lin, ang)
                    # Rate-limit deliberately. env._spin() returns as soon
                    # as any callback fires, not after its timeout, so a
                    # bare loop ran at ~570 Hz — flooding the arbiter and
                    # making the step counter meaningless. 20 Hz is well
                    # inside the arbiter's 0.3 s staleness window.
                    next_tick = time.time() + 1.0 / DESCEND_HZ
                    while time.time() < next_tick and rclpy.ok():
                        env._spin(0.01)
                else:
                    self.outcome = 'stopped'
                env._publish(0.0, 0.0)
            except Exception as exc:                      # noqa: BLE001
                self.outcome = f'error: {exc}'
                self.get_logger().error(f'descend failed: {exc}')
            finally:
                self.segment = 'idle'
                self._busy = False
        self.get_logger().info(
            f'descend finished: {self.outcome} at x={self.progress:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = RampDriver()
    # A DEDICATED executor, not rclpy.spin(). Both rclpy.spin() and
    # rclpy.spin_once() fall back to the GLOBAL executor when none is
    # given, and CocoRampEnv drives its own node with rclpy.spin_once()
    # from the segment worker thread. Sharing the global executor between
    # the two makes the very first env call die with "Executor is already
    # spinning" — which surfaces as the climb failing instantly, with the
    # robot sitting still and nothing obviously wrong.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy invalidates the context from its own SIGINT handler before
        # Python sees the signal, so Ctrl-C arrives as the latter.
        pass
    finally:
        if rclpy.ok():
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
