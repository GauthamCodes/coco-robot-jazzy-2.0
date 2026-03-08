"""
teleop_arm_node.py
==================
Keyboard teleoperation for the Coco robot manipulator arm.

Controls
--------
  w / s   : Shoulder joint (m_link1) +/-
  e / d   : Elbow joint    (m_link2) +/-
  r / f   : Gripper open / close
  SPACE   : Reset all joints to home position
  h       : Print this help
  q       : Quit

Topics published
----------------
  /m_link1_controller/commands          (std_msgs/Float64MultiArray)
  /m_link2_controller/commands          (std_msgs/Float64MultiArray)
  /m_link3_controller/commands          (std_msgs/Float64MultiArray)
  /m_link3_Revolute_9_controller/commands (std_msgs/Float64MultiArray)
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# Joint limits (radians) matching coco_arm_controller.yaml
SHOULDER_LIMITS = (-3.839724354, 0.0)
ELBOW_LIMITS = (-1.5, 0.0)
GRIP_LEFT_LIMITS = (-0.3, 1.047)
GRIP_RIGHT_LIMITS = (-1.047, 0.3)

# Home / reset position
HOME_SHOULDER = 0.0
HOME_ELBOW = -2.0   # NOTE: outside ELBOW_LIMITS — clamped to -1.5 on first publish
HOME_GRIP_LEFT = 0.0
HOME_GRIP_RIGHT = 0.0

STEP = 0.1  # radians per key press


class TeleopArm(Node):
    """ROS 2 node for keyboard-driven arm teleoperation."""

    def __init__(self):
        super().__init__('teleop_arm')

        self._shoulder_pub = self.create_publisher(
            Float64MultiArray, '/m_link1_controller/commands', 10
        )
        self._elbow_pub = self.create_publisher(
            Float64MultiArray, '/m_link2_controller/commands', 10
        )
        self._grip_left_pub = self.create_publisher(
            Float64MultiArray, '/m_link3_controller/commands', 10
        )
        self._grip_right_pub = self.create_publisher(
            Float64MultiArray, '/m_link3_Revolute_9_controller/commands', 10
        )

        self.get_logger().info(
            'Arm Teleop Node started. Press "h" for controls, "q" to quit.'
        )

    # ── helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(value, hi))

    @staticmethod
    def _get_key(timeout: float = 0.1) -> str:
        """Read one character from stdin without blocking."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            return sys.stdin.read(1) if ready else ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _publish(
        self,
        shoulder: float,
        elbow: float,
        grip_left: float,
        grip_right: float,
    ) -> None:
        self._shoulder_pub.publish(Float64MultiArray(data=[shoulder]))
        self._elbow_pub.publish(Float64MultiArray(data=[elbow]))
        self._grip_left_pub.publish(Float64MultiArray(data=[grip_left]))
        self._grip_right_pub.publish(Float64MultiArray(data=[grip_right]))

    @staticmethod
    def _print_help() -> None:
        print(
            '\n── Arm Teleop Controls ──────────────────\n'
            '  w / s   : Shoulder +/-\n'
            '  e / d   : Elbow    +/-\n'
            '  r / f   : Gripper  open/close\n'
            '  SPACE   : Reset to home position\n'
            '  h       : Show this help\n'
            '  q       : Quit\n'
            '─────────────────────────────────────────\n'
        )

    # ── main loop ────────────────────────────────────────────────────────────

    def run(self) -> None:
        shoulder = 0.0
        elbow = 0.0
        grip_left = 0.0
        grip_right = 0.0
        last_cmd = None

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                key = self._get_key()

                if key == 'q':
                    self.get_logger().info('Quitting arm teleop.')
                    break
                elif key == 'h':
                    self._print_help()
                elif key == 'w':
                    shoulder += STEP
                elif key == 's':
                    shoulder -= STEP
                elif key == 'e':
                    elbow += STEP
                elif key == 'd':
                    elbow -= STEP
                elif key == 'r':
                    grip_left += STEP
                    grip_right -= STEP
                elif key == 'f':
                    grip_left -= STEP
                    grip_right += STEP
                elif key == ' ':
                    shoulder = HOME_SHOULDER
                    elbow = HOME_ELBOW
                    grip_left = HOME_GRIP_LEFT
                    grip_right = HOME_GRIP_RIGHT
                    self.get_logger().info('Reset to home position.')

                # Apply joint limits
                shoulder = self._clamp(shoulder, *SHOULDER_LIMITS)
                elbow = self._clamp(elbow, *ELBOW_LIMITS)
                grip_left = self._clamp(grip_left, *GRIP_LEFT_LIMITS)
                grip_right = self._clamp(grip_right, *GRIP_RIGHT_LIMITS)

                # Only publish when state has changed
                cmd = (shoulder, elbow, grip_left, grip_right)
                if cmd != last_cmd:
                    self._publish(*cmd)
                    last_cmd = cmd

        except KeyboardInterrupt:
            self.get_logger().info('Arm teleop interrupted.')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopArm()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
