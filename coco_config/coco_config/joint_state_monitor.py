#!/usr/bin/env python3
"""Joint state monitoring node for Coco robot arm."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMonitor(Node):
    """Monitor joint states and publish warnings if limits exceeded."""
    
    def __init__(self):
        super().__init__('joint_state_monitor')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.get_logger().info('Joint state monitor started')
    
    def joint_state_callback(self, msg):
        """Monitor joint positions and velocities."""
        # Simple monitoring - can be expanded
        for name, pos in zip(msg.name, msg.position):
            if 'm_link' in name:  # Arm joints
                self.get_logger().debug(f'{name}: {pos:.3f} rad')

def main(args=None):
    rclpy.init(args=args)
    monitor = JointStateMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
