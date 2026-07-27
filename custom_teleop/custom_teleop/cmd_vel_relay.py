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
Relay TwistStamped messages from /cmd_vel to /diff_drive_controller/cmd_vel.

Nav2 publishes its final velocity command on /cmd_vel (via the collision
monitor), while the ros2_control DiffDriveController subscribes on its own
namespaced topic. This relay bridges the two so the Nav2 parameter file can
stay close to stock.

Both topics are parameters. The defaults keep a standalone Nav2 run
working exactly as before; during a mission, cmd_vel_arbiter is the sole
publisher to the controller, so the relay feeds it instead:

  ros2 run custom_teleop cmd_vel_relay --ros-args -p output_topic:=/cmd_vel_nav

which is what `nav.launch.py arbiter:=true` does.
"""

from geometry_msgs.msg import TwistStamped

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CmdVelRelay(Node):
    """Republishes /cmd_vel onto the DiffDriveController's topic."""

    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel')
        src = self.get_parameter('input_topic').value
        dst = self.get_parameter('output_topic').value
        self._pub = self.create_publisher(TwistStamped, dst, 10)
        self._sub = self.create_subscription(
            TwistStamped, src, self._cb, 10)
        self.get_logger().info(f'Relaying {src} -> {dst}')

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
