import select, sys, termios, tty
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist

LINEAR_STEP, ANGULAR_STEP = 0.1, 0.2
MAX_LINEAR, MAX_ANGULAR   = 1.0, 2.0
KEY_BINDINGS = {'w':(0.1,0.0),'s':(-0.1,0.0),'a':(0.0,0.2),'d':(0.0,-0.2)}

class TeleopWheels(Node):
    def __init__(self):
        super().__init__('teleop_wheels')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(Twist, '/cmd_vel', qos)
        print("Wheels teleop ready  w/s=fwd/back  a/d=turn  x=stop  q=quit")

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
        m = Twist(); m.linear.x = float(lx); m.angular.z = float(az)
        self._pub.publish(m)

    def run(self):
        lx = az = 0.0
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                k = self._get_key()
                if k == 'q': break
                elif k == 'x': lx = az = 0.0
                elif k in KEY_BINDINGS:
                    dl, da = KEY_BINDINGS[k]; lx += dl; az += da
                elif k: lx *= 0.5; az *= 0.5
                lx = max(-MAX_LINEAR, min(MAX_LINEAR, lx))
                az = max(-MAX_ANGULAR, min(MAX_ANGULAR, az))
                self._send(lx, az)
        finally:
            self._send(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    n = TeleopWheels()
    try: n.run()
    finally: n.destroy_node(); rclpy.shutdown()
