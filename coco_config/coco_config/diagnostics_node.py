#!/usr/bin/env python3
"""Diagnostics node for Coco robot health monitoring."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DiagnosticsNode(Node):
    """Simple diagnostics publisher for robot health status."""
    
    def __init__(self):
        super().__init__('diagnostics_node')
        self.publisher = self.create_publisher(String, '/diagnostics', 10)
        self.timer = self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info('Diagnostics node started')
    
    def publish_diagnostics(self):
        """Publish basic health status."""
        msg = String()
        msg.data = 'Robot status: OK'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
