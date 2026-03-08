"""
teleop_wheels_node.py
=====================
Keyboard teleoperation for the Coco robot's 4-wheel differential-drive base.

The robot URDF uses two gazebo_ros_diff_drive plugins (front + rear axle),
both listening on /cmd_vel. Publishing a single Twist message here drives
all four wheels simultaneously.

Controls
--------
  w   : Drive forward
  s   : Drive backward
  a   : Turn left  (counter-clockwise)
  d   : Turn right (clockwise)
  x   : Full stop (zero velocity)
  h   : Print this help
  q   : Quit (sends stop before exiting)

Topics published
----------------
  /cmd_vel  (geometry_msgs/Twist)
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LINEAR_STEP = 0.1    # m/s per key press
ANGULAR_STEP = 0.2   # rad/s per key press
MAX_LINEAR = 1.0     # m/s
MAX_ANGULAR = 2.0    # rad/s

KEY_BINDINGS = {
    'w': (LINEAR_STEP, 0.0),
    's': (-LINEAR_STEP, 0.0),
    'a': (0.0, ANGULAR_STEP),
    'd': (0.0, -ANGULAR_STEP),
}


class TeleopWheels(Node):
    """ROS 2 node for keyboard-driven wheel base teleoperation."""

    def __init__(self):
        super().__init__('teleop_wheels')
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info(
            'Wheels Teleop Node started. Press "h" for controls, "q" to quit.'
        )

    # ── helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(value, hi))

    @staticmethod
    def _get_key(timeout: float = 0.1) -> str:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            return sys.stdin.read(1) if ready else ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _publish_twist(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)

    def _stop(self) -> None:
        self._publish_twist(0.0, 0.0)

    @staticmethod
    def _print_help() -> None:
        print(
            '\n── Wheel Teleop Controls ────────────────\n'
            '  w   : Forward\n'
            '  s   : Backward\n'
            '  a   : Turn left\n'
            '  d   : Turn right\n'
            '  x   : Full stop\n'
            '  h   : Show this help\n'
            '  q   : Quit\n'
            '─────────────────────────────────────────\n'
        )

    # ── main loop ────────────────────────────────────────────────────────────

    def run(self) -> None:
        linear = 0.0
        angular = 0.0

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                key = self._get_key()

                if key == 'q':
                    self.get_logger().info('Quitting wheel teleop.')
                    break
                elif key == 'h':
                    self._print_help()
                elif key == 'x':
                    linear = 0.0
                    angular = 0.0
                    self.get_logger().info('Full stop.')
                elif key in KEY_BINDINGS:
                    dl, da = KEY_BINDINGS[key]
                    linear += dl
                    angular += da
                elif key != '':
                    # Any other key: gradual deceleration
                    linear *= 0.5
                    angular *= 0.5

                linear = self._clamp(linear, -MAX_LINEAR, MAX_LINEAR)
                angular = self._clamp(angular, -MAX_ANGULAR, MAX_ANGULAR)

                self._publish_twist(linear, angular)

        except KeyboardInterrupt:
            self.get_logger().info('Wheel teleop interrupted.')
        finally:
            self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopWheels()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
