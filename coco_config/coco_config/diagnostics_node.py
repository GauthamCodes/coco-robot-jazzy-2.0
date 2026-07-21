#!/usr/bin/env python3
"""
diagnostics_node.py
===================
Report simulation/robot health as a proper diagnostic_msgs/DiagnosticArray
on /diagnostics.

It reports what it actually measures — the observed publish rate of the
load-bearing sensor topics and how stale each one is — rather than a
fixed "OK". A topic that stops publishing shows up as STALE, and one
running below its nominal rate shows up as WARN, which is the failure
mode that matters here: a lagging /scan quietly degrades SLAM and Nav2
long before anything crashes.

This is the always-on counterpart to gazebo_models/scripts/verify_sim.py,
which runs the same checks once as a pass/fail gate. Rates here are
measured in wall time, so under an unlocked real-time factor expect them
to read high; verify_sim.py measures in sim time and is the right tool
when the RTF is not 1.
"""

from coco_config.robot import is_best_effort, nominal_hz

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan

WINDOW = 5.0          # s of history used for the rate estimate
STALE_AFTER = 2.0     # s without a message before a topic is STALE

BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

# Which topics this node reports on, and their message types. The nominal
# rates and QoS come from coco_config.robot, shared with
# gazebo_models/scripts/verify_sim.py so the two health checks cannot
# disagree when a sensor's <update_rate> changes.
_TYPES = {
    '/joint_states': JointState,
    '/scan': LaserScan,
    '/imu': Imu,
    '/diff_drive_controller/odom': Odometry,
}

# topic, type, nominal Hz, needs best-effort QoS
WATCHED = [
    (topic, msg_type, nominal_hz(topic), is_best_effort(topic))
    for topic, msg_type in _TYPES.items()
]


class TopicRate:
    """Sliding-window publish-rate estimate for one topic."""

    def __init__(self, node, topic, msg_type, qos):
        self._node = node
        self._times = []
        node.create_subscription(msg_type, topic, self._cb, qos)

    def _cb(self, _msg):
        now = self._node.get_clock().now().nanoseconds / 1e9
        self._times.append(now)
        cutoff = now - WINDOW
        while self._times and self._times[0] < cutoff:
            self._times.pop(0)

    def rate(self):
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        return (len(self._times) - 1) / span if span > 0 else 0.0

    def age(self, now):
        return float('inf') if not self._times else now - self._times[-1]


class DiagnosticsNode(Node):
    """Publish per-topic sensor health on /diagnostics."""

    def __init__(self):
        super().__init__('diagnostics_node')
        self._rates = {
            topic: TopicRate(self, topic, mtype,
                             BEST_EFFORT if best_effort else 10)
            for topic, mtype, _, best_effort in WATCHED
        }
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self._report)
        self.get_logger().info(
            f'diagnostics_node watching {len(WATCHED)} topics')

    def _report(self):
        now = self.get_clock().now().nanoseconds / 1e9
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        for topic, _, nominal, _ in WATCHED:
            probe = self._rates[topic]
            rate, age = probe.rate(), probe.age(now)
            status = DiagnosticStatus(name=f'coco: {topic}',
                                      hardware_id='coco')
            status.values = [
                KeyValue(key='rate_hz', value=f'{rate:.1f}'),
                KeyValue(key='nominal_hz', value=f'{nominal:.1f}'),
            ]
            if age > STALE_AFTER:
                status.level = DiagnosticStatus.STALE
                status.message = ('no messages yet' if age == float('inf')
                                  else f'stale ({age:.1f}s since last message)')
            elif rate < nominal * 0.5:
                status.level = DiagnosticStatus.WARN
                status.message = (f'{rate:.1f} Hz, well below the nominal '
                                  f'{nominal:.0f} Hz')
            elif rate > nominal * 1.5:
                # Caught a real one during development: a leftover
                # ros_gz_bridge left two publishers on /scan and /imu, which
                # doubled the observed rate. Duplicate publishers are a
                # misconfiguration worth surfacing, not a bonus.
                status.level = DiagnosticStatus.WARN
                status.message = (f'{rate:.1f} Hz, well above the nominal '
                                  f'{nominal:.0f} Hz — duplicate publisher, '
                                  f'or the real-time factor is unlocked')
            else:
                status.level = DiagnosticStatus.OK
                status.message = f'{rate:.1f} Hz'
            msg.status.append(status)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
