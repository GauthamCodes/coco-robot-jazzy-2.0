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
        # The robot's level-ground attitude. A robot constant. Supply it
        # and the reference is installed at construction; leave it NaN and
        # the reference must instead be LEARNED, which needs
        # `declare_flat` below.
        self.declare_parameter('flat_pitch', float('nan'))
        self.declare_parameter('flat_roll', 0.0)
        # Operator knowledge: "the ground under the robot right now is
        # flat". `GradeEstimator` takes its reference only while this is
        # true AND the robot is quasi-stationary, so it is the switch that
        # makes the learned path reachable at all.
        #
        # It defaults FALSE, and the default is the honest one: the node
        # cannot tell flat ground from a slope — that is the very quantity
        # it is estimating — so asserting flatness is knowledge only an
        # operator has. Left false the observer runs, reports
        # `calibrated=False` and halves its own confidence, which is the
        # designed behaviour for "no reference has been measured".
        #
        # C2-M2.1's live gate found this unwired: the node never passed
        # the flag at all, so the learned path was dead code and the
        # comment here claimed it worked. Measured live: `calibrated`
        # stayed False for all 431 samples of a full 18 deg climb.
        self.declare_parameter('declare_flat', False)

        mu_range = (float(self.get_parameter('mu_min').value),
                    float(self.get_parameter('mu_max').value))
        self.observer = TerrainObserver(mu_range=mu_range)
        flat = float(self.get_parameter('flat_pitch').value)
        if not math.isnan(flat):
            self.observer.grade_est.calibrate_flat(
                flat, float(self.get_parameter('flat_roll').value))

        # BEST_EFFORT or the node matches nothing and goes silently blind.
        #
        # `is_best_effort` takes the TOPIC and looks it up in
        # `coco_config.robot.SENSOR_TOPICS`; called with no argument it
        # raises TypeError in the constructor and the node never starts.
        # That is what it did, and C2-M2.1's live gate is what found it —
        # the pure-core unit tests never construct the node, so nothing
        # off-line could have. Each subscription now takes the QoS its own
        # topic declares rather than sharing one profile: /imu is
        # best-effort and /joint_states is reliable, and a single profile
        # is only ever correct for one of them.
        imu_topic = self.get_parameter('imu_topic').value
        js_topic = self.get_parameter('joint_states_topic').value

        def _qos(topic, depth=5):
            return QoSProfile(
                depth=depth,
                reliability=(ReliabilityPolicy.BEST_EFFORT
                             if is_best_effort(topic)
                             else ReliabilityPolicy.RELIABLE))

        self._imu = None
        self._joints = None
        self._cmd = (0.0, 0.0)
        # The newest estimate, produced at the IMU's rate and published at
        # PUBLISH_HZ. Kept apart so the two rates cannot be confused again.
        self._est = None
        self._est_pitch = 0.0
        self.create_subscription(
            Imu, imu_topic, self._on_imu, _qos(imu_topic))
        self.create_subscription(
            JointState, js_topic, self._on_joints, _qos(js_topic, depth=10))
        self.create_subscription(
            TwistStamped, self.get_parameter('cmd_vel_topic').value,
            self._on_cmd, 10)
        self._pub = self.create_publisher(
            DiagnosticArray, self.get_parameter('status_topic').value, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)

    # ── inputs ───────────────────────────────────────────────────────────
    def _on_imu(self, msg):
        self._imu = msg
        # Estimate here, at 50 Hz, NOT in the publish timer. See _estimate.
        self._estimate()

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

    # ── estimation, at the IMU's own rate ────────────────────────────────
    def _estimate(self):
        """Fold the newest IMU sample in. Called from ``_on_imu``, 50 Hz.

        **The rate is the point.** An earlier version ran this from the
        10 Hz publish timer, which handed the observer samples exactly
        ``MAX_AGE`` apart and made every single estimate report
        ``stale input: 0.100 s > 0.100 s`` — the observer withdrawing
        itself, live, on a perfectly healthy robot. It never produced one
        valid estimate and the pure-core tests could not see it, because
        they drive the observer directly at 50 Hz.

        C2-M2.0 fixed the observer rate at 50 Hz and ``B3.observe`` says
        why in as many words: the traction channel's acceleration deficit
        is a transient a 10 Hz sample misses, so an accelerometer
        decimated to the control rate is a different sensor. Estimation
        belongs on the sensor's clock; publication belongs on the
        consumer's. They are separated here for that reason.
        """
        i = self._imu
        if i is None:
            return
        wheels = self._wheel_speeds()
        if wheels is None:
            return

        from coco_rl.terrain_observer import DeployableSignals
        stamp = i.header.stamp.sec + i.header.stamp.nanosec * 1e-9
        roll, pitch, yaw = quat_to_rpy(
            (i.orientation.w, i.orientation.x, i.orientation.y,
             i.orientation.z))
        self._est = self.observer.update(DeployableSignals(
            stamp=stamp, roll=roll, pitch=pitch, yaw=yaw,
            roll_rate=i.angular_velocity.x,
            pitch_rate=i.angular_velocity.y,
            yaw_rate=i.angular_velocity.z,
            accel_body=(i.linear_acceleration.x, i.linear_acceleration.y,
                        i.linear_acceleration.z),
            wheel_speeds=wheels,
            cmd_linear=self._cmd[0], cmd_angular=self._cmd[1],
            wheel_radius=WHEEL_RADIUS,
            on_declared_flat=bool(
                self.get_parameter('declare_flat').value)))
        self._est_pitch = pitch

    # ── output ───────────────────────────────────────────────────────────
    def _publish(self):
        """Publish the most recent estimate at ``PUBLISH_HZ``.

        Publishes only; it never advances the estimator. See ``_estimate``.
        """
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._imu is None:
            msg.status = self._waiting('waiting for /imu')
            self._pub.publish(msg)
            return
        if self._wheel_speeds() is None:
            msg.status = self._waiting('waiting for /joint_states')
            self._pub.publish(msg)
            return
        if self._est is None:
            msg.status = self._waiting('waiting for the first estimate')
            self._pub.publish(msg)
            return
        msg.status = [self._grade_status(self._est, self._est_pitch),
                      self._traction_status(self._est)]
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
        # Terminology, on the wire where there is no docstring to read.
        # `tau` is the TRACTION-DEMAND RATIO, `mu_lower` the bound it
        # proves, and `mu_sched_input` the number a gain schedule would
        # consume. None of them is a friction estimate: C2-M2.0 measured
        # that true mu is not identifiable from this robot's IMU and
        # encoders anywhere in the operating envelope.
        s.values = [
            _kv('tau_traction_demand', f'{est.tau:.4f}'),
            _kv('mu_lower_bound', f'{est.mu_lower:.4f}'),
            _kv('mu_sched_input', f'{est.mu_hat:.4f}'),
            _kv('note', 'tau is traction demand, not friction; '
                        'true mu is not identifiable'),
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
