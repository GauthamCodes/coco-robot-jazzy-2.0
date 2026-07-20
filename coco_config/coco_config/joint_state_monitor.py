#!/usr/bin/env python3
"""
joint_state_monitor.py
======================
Watch /joint_states and warn when an arm or gripper joint approaches (or
exceeds) the limits declared in coco_robo2.xacro.

Publishes a diagnostic_msgs/DiagnosticStatus stream on /diagnostics so
the result is visible to standard tooling (rqt_robot_monitor,
diagnostic_aggregator) rather than only in the log. The limit maths is a
pure function in coco_config.joint_limits, unit-tested without a sim.
"""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from coco_config.joint_limits import ARM_LIMITS, limit_margin

STALE_AFTER = 2.0     # s without /joint_states before reporting STALE


class JointStateMonitor(Node):
    """Report arm joint headroom against the URDF limits."""

    def __init__(self):
        super().__init__('joint_state_monitor')
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self._report)
        self._last = None      # (stamp, {name: position})
        self.get_logger().info(
            f'joint_state_monitor watching {len(ARM_LIMITS)} joints')

    def _joint_state_cb(self, msg):
        self._last = (self.get_clock().now(),
                      dict(zip(msg.name, msg.position)))

    def _report(self):
        status = DiagnosticStatus(name='coco: arm joint limits',
                                  hardware_id='coco')
        if self._last is None:
            status.level = DiagnosticStatus.WARN
            status.message = 'no /joint_states received yet'
        else:
            stamp, positions = self._last
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if age > STALE_AFTER:
                status.level = DiagnosticStatus.STALE
                status.message = f'/joint_states stale ({age:.1f}s old)'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'all arm joints within limits'
                worst = []
                for name, pos in positions.items():
                    margin, near, beyond = limit_margin(name, pos)
                    if margin is None:
                        continue          # wheels etc. — not limit-checked
                    status.values.append(
                        KeyValue(key=name,
                                 value=f'{pos:.3f} rad (margin {margin:.3f})'))
                    if beyond:
                        status.level = DiagnosticStatus.ERROR
                        worst.append(f'{name} beyond limit ({pos:.3f})')
                    elif near and status.level != DiagnosticStatus.ERROR:
                        status.level = DiagnosticStatus.WARN
                        worst.append(f'{name} near limit ({pos:.3f})')
                if worst:
                    status.message = '; '.join(worst)

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.append(status)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMonitor()
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
