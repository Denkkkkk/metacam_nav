import socket
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        # 创建一个话题发布者，发布到"/cmd_vel"话题，消息类型是Twist
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("CmdVelPublisher has been initialized")

    def publish_cmd_vel(self, linear_x, angular_z):
        # 创建一个Twist消息
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        # 发布消息
        self.publisher_.publish(twist)
        self.get_logger().info(f"Published linear.x: {linear_x}, angular.z: {angular_z}")

def receive_data(cmd_vel_publisher):
    # 创建一个 TCP/IP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        # 绑定到特定的地址和端口
        server_sock.bind(('0.0.0.0', 38977))  # 监听端口38977
        server_sock.listen(1)  # 最大连接数为1

        print("Server is listening on port 38977...")

        # 等待连接
        connection, client_address = server_sock.accept()

        with connection:
            print(f"Connection established with {client_address}")
            
            while True:
                # 持续接收数据（最大1024字节）
                data = connection.recv(1024)
                
                if not data:
                    # 如果没有接收到数据，退出
                    return
                
                # 解码接收到的数据
                data_str = data.decode('utf-8')
                print(f"Received data: {data_str}")

                # 解析数据并发布
                try:
                    # 假设数据格式为 "1.0#0.0"，我们将其分离
                    linear_x_str, angular_z_str = data_str.split('#')
                    linear_x = float(linear_x_str)
                    angular_z = float(angular_z_str)

                    # 发布到ROS 2话题
                    cmd_vel_publisher.publish_cmd_vel(linear_x, angular_z)
                except ValueError as e:
                    print(f"Failed to parse the data: {data_str}, Error: {e}")

def main():
    # 初始化ROS 2
    rclpy.init()

    # 创建发布者节点
    cmd_vel_publisher = CmdVelPublisher()
    while(rclpy.ok()):
        try:
            # 调用接收数据的函数
            receive_data(cmd_vel_publisher)
        except KeyboardInterrupt:
            print("Server has been stopped.")

    # 清理并关闭ROS 2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
