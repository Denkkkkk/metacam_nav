import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

class TestaListener(Node):
    def __init__(self):
        super().__init__('testa_listener')
        self.subscription = self.create_subscription(
            Twist,
            '/tower/navigation/cmd_vel',
            self.callback,
            10
        )

    def callback(self, msg):
        self.get_logger().info(f"Received on testa: {msg.linear.x}")
        self.get_logger().info(f"Received on testa: {msg.angular.z}")
        data = str(msg.linear.x) + "#" + str(msg.angular.z)
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect(('0.0.0.0', 38977))
                self.get_logger().info("Sending message to System B")
                sock.sendall(data.encode('utf-8'))
        except socket.error as exc:
            self.get_logger().error(f"Caught exception socket.error : {exc}")

def main(args=None):
    rclpy.init(args=args)
    node = TestaListener()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
