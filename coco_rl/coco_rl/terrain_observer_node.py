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
The ROS face of the terrain observer.

Imports :mod:`coco_rl.terrain_observer` and never the reverse — the
estimator stays pure so ``yard_env`` can reach it, and CLAUDE.md §2 stays
structural rather than aspirational.

**This node publishes and does not drive.** It adds no publisher to any
``cmd_vel`` topic, so ``cmd_vel_arbiter`` remains the sole publisher to
the controller, exactly as CLAUDE.md §4 requires. A controller that wants
terrain-aware gains subscribes to ``/terrain/state`` and goes on
publishing wherever it published before.

Interfaces
==========

Subscribes
----------
``/imu`` (``sensor_msgs/Imu``, **BEST_EFFORT**)
    Attitude, angular rate and linear acceleration. The reliability is
    not optional: the sensor topics in this project are BEST_EFFORT and a
    RELIABLE subscriber never matches, leaving the node **silently
    blind** — CLAUDE.md's trap table names this one. The flag comes from
    ``coco_config.robot.is_best_effort`` rather than from a literal here.

``/joint_states`` (``sensor_msgs/JointState``)
    Wheel velocities. The four wheel joints are the names
    ``coco_controllers.yaml`` gives ``diff_drive_controller``, read from
    there in spirit and listed below in fact.

``/diff_drive_controller/cmd_vel`` (``geometry_msgs/TwistStamped``)
    The commanded twist. Subscribed, never published.

Publishes
---------
``/terrain/state`` (``diagnostic_msgs/DiagnosticArray``, 10 Hz)
    Two statuses, ``coco: terrain grade`` and ``coco: terrain traction``.
    ``level`` carries validity — ``OK``, ``WARN`` when the estimate stands
    but is uncertain, ``STALE`` when an input has gone away — and the
    ``values`` carry the numbers. No custom message is introduced:
    ``DiagnosticArray`` already carries a per-item level and a labelled
    key/value payload, ``coco_config/diagnostics_node.py`` already uses
    it, and ``rqt_robot_monitor`` renders it for free.

Timing
------
``use_sim_time`` throughout, and the estimator's own arithmetic runs on
the **message stamps**, not on the node's wall clock. The observer
rejects a sample whose stamp has not advanced and withdraws its estimate
when one is stale, so a stalled ``/clock`` produces an explicit invalid
rather than a smoothly extrapolated fiction.

Failure behaviour
-----------------
Every one of these produces an explicit ``STALE`` or ``WARN`` with a
reason string, never a plausible number:

===========================  =====================================
no ``/imu`` yet              ``STALE``, "waiting for /imu"
no ``/joint_states`` yet     ``STALE``, "waiting for /joint_states"
IMU older than ``MAX_AGE``   ``STALE``, the observer's own reason
clock not advancing          ``STALE``, "clock did not advance"
below the speed floor        ``WARN``, traction withheld
turning                      ``WARN``, skid-steer scrub
normal load off its band     ``WARN``, wheel light or slamming
bound below the floor        ``WARN``, "not yet established"
grade and gravity disagree   ``WARN``, "estimator disagreement"
===========================  =====================================

The last is a genuine cross-check rather than a formality. At rest the
accelerometer's own longitudinal channel gives the tilt independently of
the orientation quaternion, ``asin(f_x / g)``, so the two can be compared
and a disagreement means one of them is wrong. It is only meaningful when
the body is not accelerating, so it is evaluated only there.
"""

import math

from coco_config.robot import WHEEL_RADIUS, is_best_effort

from coco_rl.sensor_model import quat_to_rpy
from coco_rl.terrain_observer import G, TerrainObserver

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from geometry_msgs.msg import TwistStamped

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu, JointState


# gazebo_models/urdf/coco_controllers.yaml, diff_drive_controller:
#   left_wheel_names  ["base_Revolute-3", "base_Revolute-4"]
#   right_wheel_names ["base_Revolute-1", "base_Revolute-2"]
WHEEL_JOINTS = ('base_Revolute-1', 'base_Revolute-2',
                'base_Revolute-3', 'base_Revolute-4')

PUBLISH_HZ = 10.0

# The disagreement check is only meaningful while the body is not
# accelerating: at rest asin(f_x/g) IS the tilt, under acceleration it is
# tilt plus a/g and the two are not separable from one sample.
QUIET_ACCEL = 0.3            # m/s^2
DISAGREE_LIMIT = math.radians(5.0)


def _kv(key, value):
    return KeyValue(key=key, value=value)


class TerrainObserverNode(Node):
    """Publishes the terrain estimate. Drives nothing."""

    def __init__(self):
        super().__init__('terrain_observer')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('cmd_vel_topic',
                               '/diff_drive_controller/cmd_vel')
        self.declare_parameter('status_topic', '/terrain/state')
        self.declare_parameter('mu_min', 0.35)
        self.declare_parameter('mu_max', 0.70)
        # The robot's level-ground attitude. A robot constant; left at NaN
        # it is learned from the first quiet samples instead, and the
        # estimate says `calibrated=False` until it has been.
        self.declare_parameter('flat_pitch', float('nan'))
        self.declare_parameter('flat_roll', 0.0)

        mu_range = (float(self.get_parameter('mu_min').value),
                    float(self.get_parameter('mu_max').value))
        self.observer = TerrainObserver(mu_range=mu_range)
        flat = float(self.get_parameter('flat_pitch').value)
        if not math.isnan(flat):
            self.observer.grade_est.calibrate_flat(
                flat, float(self.get_parameter('flat_roll').value))

        # BEST_EFFORT or the node matches nothing and goes silently blind.
        qos = QoSProfile(
            depth=5,
            reliability=(ReliabilityPolicy.BEST_EFFORT if is_best_effort()
                         else ReliabilityPolicy.RELIABLE))
        self._imu = None
        self._joints = None
        self._cmd = (0.0, 0.0)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._on_imu, qos)
        self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value,
            self._on_joints, 10)
        self.create_subscription(
            TwistStamped, self.get_parameter('cmd_vel_topic').value,
            self._on_cmd, 10)
        self._pub = self.create_publisher(
            DiagnosticArray, self.get_parameter('status_topic').value, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)

    # ── inputs ───────────────────────────────────────────────────────────
    def _on_imu(self, msg):
        self._imu = msg

    def _on_joints(self, msg):
        self._joints = msg

    def _on_cmd(self, msg):
        self._cmd = (msg.twist.linear.x, msg.twist.angular.z)

    def _wheel_speeds(self):
        """Wheel velocities in the order WHEEL_JOINTS lists, or None.

        Returns None rather than zeros when a name is absent: a missing
        encoder and a stationary wheel are different facts, and only one
        of them means the traction channel can run.
        """
        js = self._joints
        if js is None or not js.velocity:
            return None
        try:
            idx = [js.name.index(n) for n in WHEEL_JOINTS]
        except ValueError:
            return None
        if max(idx) >= len(js.velocity):
            return None
        return tuple(float(js.velocity[i]) for i in idx)

    # ── output ───────────────────────────────────────────────────────────
    def _publish(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._imu is None:
            msg.status = self._waiting('waiting for /imu')
            self._pub.publish(msg)
            return
        wheels = self._wheel_speeds()
        if wheels is None:
            msg.status = self._waiting('waiting for /joint_states')
            self._pub.publish(msg)
            return

        from coco_rl.terrain_observer import DeployableSignals
        i = self._imu
        stamp = i.header.stamp.sec + i.header.stamp.nanosec * 1e-9
        roll, pitch, yaw = quat_to_rpy(
            (i.orientation.w, i.orientation.x, i.orientation.y,
             i.orientation.z))
        est = self.observer.update(DeployableSignals(
            stamp=stamp, roll=roll, pitch=pitch, yaw=yaw,
            roll_rate=i.angular_velocity.x,
            pitch_rate=i.angular_velocity.y,
            yaw_rate=i.angular_velocity.z,
            accel_body=(i.linear_acceleration.x, i.linear_acceleration.y,
                        i.linear_acceleration.z),
            wheel_speeds=wheels,
            cmd_linear=self._cmd[0], cmd_angular=self._cmd[1],
            wheel_radius=WHEEL_RADIUS))
        msg.status = [self._grade_status(est, pitch),
                      self._traction_status(est)]
        self._pub.publish(msg)

    def _waiting(self, why):
        return [DiagnosticStatus(name=n, level=DiagnosticStatus.STALE,
                                 message=why)
                for n in ('coco: terrain grade', 'coco: terrain traction')]

    def _disagreement(self, est, pitch):
        """asin(f_x/g) against the quaternion's pitch, while quiet.

        Two independent routes to the same tilt. They can only be compared
        where the body is not accelerating, so everywhere else this
        returns None rather than a number that mixes tilt with a/g.
        """
        i = self._imu
        a_fwd = i.linear_acceleration.x + G * math.sin(pitch)
        if abs(a_fwd) > QUIET_ACCEL:
            return None
        ratio = max(-1.0, min(1.0, i.linear_acceleration.x / G))
        return abs(math.asin(ratio) - (-pitch))

    def _grade_status(self, est, pitch):
        s = DiagnosticStatus(name='coco: terrain grade')
        disagree = self._disagreement(est, pitch)
        if not est.grade_valid:
            s.level, s.message = DiagnosticStatus.STALE, est.reason
        elif disagree is not None and disagree > DISAGREE_LIMIT:
            s.level = DiagnosticStatus.WARN
            s.message = (f'estimator disagreement: attitude and gravity '
                         f'differ by {math.degrees(disagree):.1f} deg')
        elif not est.grade_calibrated:
            s.level = DiagnosticStatus.WARN
            s.message = 'flat-ground reference not yet taken'
        elif est.grade_confidence < 0.25:
            s.level = DiagnosticStatus.WARN
            s.message = 'body pitch is scattering too much to be the surface'
        else:
            s.level, s.message = DiagnosticStatus.OK, 'grade estimate valid'
        s.values = [
            _kv('grade_deg', f'{math.degrees(est.grade):.3f}'),
            _kv('camber_deg', f'{math.degrees(est.camber):.3f}'),
            _kv('confidence', f'{est.grade_confidence:.3f}'),
            _kv('roughness_deg', f'{math.degrees(est.grade_roughness):.3f}'),
            _kv('calibrated', str(bool(est.grade_calibrated))),
            _kv('disagreement_deg',
                '--' if disagree is None else f'{math.degrees(disagree):.3f}'),
            _kv('stamp', f'{est.stamp:.3f}'),
        ]
        return s

    def _traction_status(self, est):
        s = DiagnosticStatus(name='coco: terrain traction')
        if not est.traction_valid:
            s.level, s.message = DiagnosticStatus.WARN, est.reason
        elif not est.mu_established:
            s.level = DiagnosticStatus.WARN
            s.message = ('bound not yet established: nothing measured '
                         'beyond the a-priori floor')
        else:
            s.level = DiagnosticStatus.OK
            s.message = ('mu at its measured limit' if est.saturated
                         else 'mu bounded below by the observed demand')
        s.values = [
            _kv('tau', f'{est.tau:.4f}'),
            _kv('mu_lower', f'{est.mu_lower:.4f}'),
            _kv('mu_hat', f'{est.mu_hat:.4f}'),
            _kv('established', str(bool(est.mu_established))),
            _kv('saturated', str(bool(est.saturated))),
            _kv('deficit_mps2', f'{est.deficit:.3f}'),
            _kv('confidence', f'{est.traction_confidence:.3f}'),
        ]
        return s


def main(args=None):
    rclpy.init(args=args)
    node = TerrainObserverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
