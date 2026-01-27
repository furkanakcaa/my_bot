#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pythonosc import dispatcher, osc_server
import threading

class OscTeleop(Node):
    def __init__(self):
        super().__init__('osc_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear = 0.0
        self.angular = 0.0

        disp = dispatcher.Dispatcher()
        disp.map("/oscControl/slider2Dy", self.linear_cb)
        disp.map("/oscControl/slider2Dx", self.angular_cb)

        self.server = osc_server.ThreadingOSCUDPServer(
            ("0.0.0.0", 5005),
            disp
        )

        threading.Thread(
            target=self.server.serve_forever,
            daemon=True
        ).start()

        self.timer = self.create_timer(0.05, self.publish_cmd)
        self.get_logger().info("OSC Teleop hazır (5005)")

    def linear_cb(self, addr, value):
        self.linear = float(value)

    def angular_cb(self, addr, value):
        self.angular = float(value)

    def publish_cmd(self):
        twist = Twist()
        twist.linear.x = self.linear * 1.0
        twist.angular.z = self.angular * 1.5
        self.pub.publish(twist)

def main():
    rclpy.init()
    node = OscTeleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
