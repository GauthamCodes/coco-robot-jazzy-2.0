"""
Keyboard teleoperation for the Coco robot manipulator arm (Jazzy / Layer 1).

The arm and gripper are driven by JointTrajectoryControllers (see
coco_controllers.yaml), so this node publishes JointTrajectory messages
instead of the old per-joint Float64MultiArray forward commands.

Controls
--------
  w / s   : Shoulder joint (m_link1_Revolute-6) +/-
  e / d   : Elbow joint    (m_link2_Revolute-7) +/-
  r / f   : Gripper open / close
  SPACE   : Reset all joints to home position
  h       : Print this help
  q       : Quit

Topics published
----------------
  /arm_controller/joint_trajectory      (trajectory_msgs/JointTrajectory)
  /gripper_controller/joint_trajectory  (trajectory_msgs/JointTrajectory)
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = ['m_link1_Revolute-6', 'm_link2_Revolute-7']
GRIPPER_JOINTS = ['m_link3_Revolute-8', 'm_link3_Revolute-9']

# Joint limits (radians) matching coco_robo2.xacro
# shoulder: negative = raise arm, positive = reach down toward the floor
SHOULDER_LIMITS = (-3.84, 1.0)
ELBOW_LIMITS = (-1.6, 1.6)
GRIP_LIMITS = (-0.3, 1.047)   # grip1; grip2 mirrors with opposite sign

# Home / reset position
HOME_SHOULDER = 0.0
HOME_ELBOW = 0.0
HOME_GRIP = 0.0

STEP = 0.1            # radians per key press
MOVE_TIME = 0.3       # trajectory point time_from_start (seconds)


class TeleopArm(Node):
    """ROS 2 node for keyboard-driven arm teleoperation via JTC."""

    def __init__(self):
        super().__init__('teleop_arm')

        self._arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )
        self._grip_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10
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

    def _traj(self, joints, positions):
        msg = JointTrajectory()
        msg.joint_names = joints
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(seconds=MOVE_TIME).to_msg()
        msg.points = [pt]
        return msg

    def _publish(self, shoulder: float, elbow: float, grip: float) -> None:
        self._arm_pub.publish(self._traj(ARM_JOINTS, [shoulder, elbow]))
        # grip2 (Revolute-9) mirrors grip1 (Revolute-8) with opposite sign
        self._grip_pub.publish(self._traj(GRIPPER_JOINTS, [grip, -grip]))

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
        shoulder = HOME_SHOULDER
        elbow = HOME_ELBOW
        grip = HOME_GRIP
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
                    grip += STEP
                elif key == 'f':
                    grip -= STEP
                elif key == ' ':
                    shoulder, elbow, grip = HOME_SHOULDER, HOME_ELBOW, HOME_GRIP
                    self.get_logger().info('Reset to home position.')

                # Apply joint limits
                shoulder = self._clamp(shoulder, *SHOULDER_LIMITS)
                elbow = self._clamp(elbow, *ELBOW_LIMITS)
                grip = self._clamp(grip, *GRIP_LIMITS)

                # Only publish when state has changed
                cmd = (shoulder, elbow, grip)
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
