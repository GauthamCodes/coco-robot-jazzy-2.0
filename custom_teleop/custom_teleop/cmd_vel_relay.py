"""
Relay TwistStamped messages from /cmd_vel to /diff_drive_controller/cmd_vel.

Nav2 publishes its final velocity command on /cmd_vel (via the collision
monitor), while the ros2_control DiffDriveController subscribes on its own
namespaced topic. This relay bridges the two so the Nav2 parameter file can
stay close to stock.
"""

from geometry_msgs.msg import TwistStamped

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CmdVelRelay(Node):
    """Republishes /cmd_vel onto the DiffDriveController's topic."""

    def __init__(self):
        super().__init__('cmd_vel_relay')
        self._pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self._sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self._cb, 10)
        self.get_logger().info(
            'Relaying /cmd_vel -> /diff_drive_controller/cmd_vel')

    def _cb(self, msg):
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy's own SIGINT handler invalidates the context before Python
        # sees the signal, so Ctrl-C (e.g. tearing down nav.launch.py)
        # arrives as ExternalShutdownException, not KeyboardInterrupt.
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
