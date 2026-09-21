# Copyright 2026 Gautham Anil -- Apache-2.0.
"""
Record what reached the wheels while a browser drives COCO.

    python3 wheel_recorder.py out.jsonl

One JSON object per line, wall-clock `t`, so it lines up with the browser
scenario's own action log. Measurement only: it publishes nothing.

Traps it avoids, each paid for in this repo already:
  * /diff_drive_controller/cmd_vel carries TwistStamped from the arbiter;
    a Twist subscriber there is silently blind.
  * Sensor-ish topics are subscribed BEST_EFFORT, which matches both
    reliable and best-effort publishers; a RELIABLE subscriber matches
    only the former and receives nothing from the latter.
"""

import json
import signal
import sys
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
import tf2_ros

WHEEL = '/diff_drive_controller/cmd_vel'
TELEOP = '/cmd_vel_teleop'


class Recorder(Node):
    def __init__(self, path):
        super().__init__('p02_wheel_recorder')
        self.out = open(path, 'a', buffering=1)
        be = QoSProfile(depth=50,
                        reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TwistStamped, WHEEL,
                                 lambda m: self._twist('wheel', m), be)
        self.create_subscription(TwistStamped, TELEOP,
                                 lambda m: self._twist('teleop', m), be)
        self.create_subscription(String, '/mission/state', self._mission, 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom',
                                 self._odom, be)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self._last_odom = 0.0
        self._last_mission = None
        self.create_timer(1.0, self._graph)

    def write(self, **row):
        row['t'] = round(time.time(), 4)
        self.out.write(json.dumps(row) + '\n')

    def _twist(self, topic, msg):
        self.write(k=topic, lin=round(msg.twist.linear.x, 4),
                   ang=round(msg.twist.angular.z, 4))

    def _mission(self, msg):
        if msg.data != self._last_mission:
            self._last_mission = msg.data
            self.write(k='mission', line=msg.data)

    def _odom(self, msg):
        now = time.time()
        if now - self._last_odom < 0.1:
            return
        self._last_odom = now
        p = msg.pose.pose.position
        self.write(k='odom', x=round(p.x, 4), y=round(p.y, 4),
                   v=round(msg.twist.twist.linear.x, 4),
                   w=round(msg.twist.twist.angular.z, 4))

    def _graph(self):
        pubs = [f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                for i in self.get_publishers_info_by_topic(WHEEL)]
        teleop = [f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                  for i in self.get_publishers_info_by_topic(TELEOP)]
        localised = self.tf.can_transform('map', 'odom', rclpy.time.Time())
        self.write(k='graph', wheel_pubs=sorted(pubs),
                   teleop_pubs=sorted(teleop), localised=bool(localised))


def main():
    rclpy.init()
    node = Recorder(sys.argv[1])

    def stop(*_):
        node.out.flush()
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.out.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
