"""
Teleoperation node for controlling the Coco robot wheels via keyboard.

This node provides keyboard-based control for the robot's differential drive base,
publishing TwistStamped messages to /diff_drive_controller/cmd_vel (the Jazzy
diff_drive_controller accepts TwistStamped only). Features include:
- Incremental velocity control with configurable steps
- Safety limits on linear and angular velocities
- Auto-stop timeout if no input received (1 second default)
- Emergency slowdown on unknown key press

Controls:
    w/s - Increase/decrease forward velocity
    a/d - Increase/decrease turning velocity  
    x   - Emergency stop (zero all velocities)
    q   - Quit and send final stop command
    
Safety Features:
    - Velocity clamping to MAX_LINEAR/MAX_ANGULAR
    - Auto-stop after TIMEOUT_SECONDS of no input
    - Guaranteed stop command on exit
    
Author: gautham
License: Apache-2.0
"""

import select, sys, termios, tty, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped

LINEAR_STEP, ANGULAR_STEP = 0.1, 0.2
MAX_LINEAR, MAX_ANGULAR   = 1.0, 2.0
TIMEOUT_SECONDS = 1.0  # Auto-stop if no input for 1 second
KEY_BINDINGS = {'w':(0.1,0.0),'s':(-0.1,0.0),'a':(0.0,0.2),'d':(0.0,-0.2)}

class TeleopWheels(Node):
    def __init__(self):
        super().__init__('teleop_wheels')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', qos)
        self._last_input_time = time.time()
        self._timeout_active = False
        self.get_logger().info("Wheels teleop ready: w/s=fwd/back  a/d=turn  x=stop  q=quit")
        self.get_logger().info(f"Safety: Auto-stop after {TIMEOUT_SECONDS}s of no input")

    def _get_key(self, t=0.1):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            r,_,_ = select.select([sys.stdin],[],[],t)
            return sys.stdin.read(1) if r else ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _send(self, lx, az):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = float(lx)
        m.twist.angular.z = float(az)
        self._pub.publish(m)

    def _check_timeout(self):
        """Auto-stop if no input received within timeout period"""
        if time.time() - self._last_input_time > TIMEOUT_SECONDS:
            if not self._timeout_active:
                self.get_logger().warn('Safety timeout! Auto-stopping robot.')
                self._timeout_active = True
            return True
        return False

    def run(self):
        lx = az = 0.0
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                k = self._get_key()
                
                # Check for timeout and auto-stop
                if self._check_timeout():
                    lx = az = 0.0
                
                if k == 'q': 
                    self.get_logger().info('Quitting wheels teleop')
                    break
                elif k == 'x': 
                    lx = az = 0.0
                    self._last_input_time = time.time()
                    self._timeout_active = False
                elif k in KEY_BINDINGS:
                    dl, da = KEY_BINDINGS[k]
                    lx += dl
                    az += da
                    self._last_input_time = time.time()
                    self._timeout_active = False
                elif k:
                    # Any other key: emergency slow-down (50% reduction)
                    lx *= 0.5
                    az *= 0.5
                    self._last_input_time = time.time()
                    self._timeout_active = False
                
                # Apply velocity limits
                lx = max(-MAX_LINEAR, min(MAX_LINEAR, lx))
                az = max(-MAX_ANGULAR, min(MAX_ANGULAR, az))
                self._send(lx, az)
        finally:
            self.get_logger().info('Sending final stop command')
            self._send(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    n = TeleopWheels()
    try: n.run()
    finally: n.destroy_node(); rclpy.shutdown()
