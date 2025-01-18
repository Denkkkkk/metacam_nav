#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('host_receive_cmd_vel')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("CmdVelPublisher initialized. Waiting for incoming connections...")

    def publish_cmd_vel(self, linear_x, angular_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.x = angular_x
        twist.angular.z = angular_z
        self.publisher_.publish(twist)
        self.get_logger().info(f"Published linear.x: {linear_x}, angular.x: {angular_x}, angular.z: {angular_z}")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()

    # 建立服务器 Socket
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_ip = '0.0.0.0'
    server_port = 39000
    server_sock.bind((server_ip, server_port))
    server_sock.listen(1)
    node.get_logger().info(f"Server listening on {server_ip}:{server_port}")

    try:
        while rclpy.ok():
            node.get_logger().info("Waiting for a new TCP client...")
            
            # 等待客户端连接（阻塞）
            connection, client_address = server_sock.accept()
            node.get_logger().warn(f"Connection established with {client_address}")

            # 在当前连接上，循环读取数据，直到对方断开
            with connection:
                while rclpy.ok():
                    data = connection.recv(1024)
                    if not data:
                        node.get_logger().warn("Connection closed by client.")
                        break  # 跳出内部循环，回到 accept

                    # 解码数据并发布
                    data_str = data.decode('utf-8').strip()
                    try:
                        linear_x_str, angular_x_str, angular_z_str = data_str.split('#')
                        linear_x = float(linear_x_str)
                        angular_x = float(angular_x_str)
                        angular_z = float(angular_z_str)
                        node.publish_cmd_vel(linear_x, angular_x, angular_z)
                    except ValueError as e:
                        node.get_logger().error(f"Failed to parse data: '{data_str}', Error: {e}")

    except KeyboardInterrupt:
        node.get_logger().warn("Server interrupted by user.")

    finally:
        server_sock.close()
        node.get_logger().info("Server socket closed.")
        rclpy.shutdown()


if __name__ == '__main__':
    main()