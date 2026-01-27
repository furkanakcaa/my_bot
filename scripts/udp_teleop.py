import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

class UdpTeleop(Node):
    def __init__(self):
        super().__init__('udp_teleop')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 5005))
        self.sock.setblocking(False)

        self.get_logger().info('UDP Teleop listening on port 5005')

        self.timer = self.create_timer(0.02, self.read_udp)

    def read_udp(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            msg = data.decode().strip()

            linear, angular = map(float, msg.split(','))

            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular

            self.publisher.publish(twist)

        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().warn(f'Invalid data: {e}')

def main():
    rclpy.init()
    node = UdpTeleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()