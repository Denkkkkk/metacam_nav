#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import socket

class CmdVelTcpClient:
    def __init__(self):
        rospy.init_node('docker_send', anonymous=True)

        # 服务器的 IP 与端口，视自己实际情况修改
        self.server_ip = '0.0.0.0'
        self.server_port = 39000

        # 创建 Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            rospy.loginfo(f"Trying to connect to {self.server_ip}:{self.server_port} ...")
            self.sock.connect((self.server_ip, self.server_port))
            rospy.loginfo("Connected successfully to System B.")
        except socket.error as e:
            rospy.logerr(f"Failed to connect: {e}")
            rospy.signal_shutdown("Cannot connect to server.")
            return

        # 订阅 /cmd_vel 话题
        rospy.Subscriber('/cmd_vel', Twist, self.callback)

        rospy.on_shutdown(self.close_socket)

    def callback(self, msg):
        # 拼接发送的数据
        data = f"{msg.linear.x}#{msg.angular.x}#{msg.angular.z}\n"
        try:
            self.sock.sendall(data.encode('utf-8'))
            # rospy.loginfo(f"Send data: {data}")
        except socket.error as exc:
            rospy.logerr(f"Socket error while sending data: {exc}")
            rospy.signal_shutdown("Socket error.")
            return

    def close_socket(self):
        rospy.loginfo("Shutting down node, closing socket.")
        try:
            self.sock.close()
        except Exception as e:
            rospy.logerr(f"Error closing socket: {e}")

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    node = CmdVelTcpClient()
    node.spin()