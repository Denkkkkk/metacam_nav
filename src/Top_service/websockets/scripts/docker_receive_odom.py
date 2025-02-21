#!/usr/bin/env python3
import socket
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3

def receivedata(pub):
    # 创建一个 TCP/IP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        # 绑定到特定的地址和端口
        server_sock.bind(('0.0.0.0', 38992)) 
        server_sock.listen(1)  # 最大连接数为1
        rospy.logwarn("Server is listening on port 38992...")

        # 等待连接
        connection, client_address = server_sock.accept()

        with connection:
            rospy.logwarn(f"Connection established with {client_address}")

            while not rospy.is_shutdown():
                # 持续接收数据（最大1024字节）
                data = connection.recv(1024)

                if data:
                    # 输出接收到的数据
                    received_str = data.decode('utf-8')
                    # rospy.logwarn(f"Received data: {received_str}")

                    # 解析接收到的字符串，假设格式为 "positionx#positiony#positionz#orientationx#orientationy#orientationz#orientationw#linearvelocityx#linearvelocityy#linearvelocityz#angularvelocityx#angularvelocityy#angularvelocityz"
                    try:
                        values = received_str.split('#')
                        positionx = float(values[0])
                        positiony = float(values[1])
                        positionz = float(values[2])
                        orientationx = float(values[3])
                        orientationy = float(values[4])
                        orientationz = float(values[5])
                        orientationw = float(values[6])
                        linearvelocityx = float(values[7])
                        linearvelocityy = float(values[8])
                        linearvelocityz = float(values[9])
                        angularvelocityx = float(values[10])
                        angularvelocityy = float(values[11])
                        angularvelocityz = float(values[12])

                        # 创建 Odometry 消息对象
                        odom_msg = Odometry()

                        # 设置位置信息websockets/sync/test_ros_docker_server.py
                        odom_msg.pose.pose.position = Point(positionx, positiony, positionz)
                        odom_msg.pose.pose.orientation = Quaternion(orientationx, orientationy, orientationz, orientationw)

                        # 设置速度信息
                        odom_msg.twist.twist.linear = Vector3(linearvelocityx, linearvelocityy, linearvelocityz)
                        odom_msg.twist.twist.angular = Vector3(angularvelocityx, angularvelocityy, angularvelocityz)

                        # 发布消息到 /Odometry
                        pub.publish(odom_msg)
                        rospy.logwarn(f"Published Odometry data to /Odometry topic")
                        # rospy.logwarn(f"Published to /Odometry: position = ({positionx}, {positiony}, {positionz}), "
                        #     f"orientation = ({orientationx}, {orientationy}, {orientationz}, {orientationw}), "
                        #     f"linear_velocity = ({linearvelocityx}, {linearvelocityy}, {linearvelocityz}), "
                        #     f"angular_velocity = ({angularvelocityx}, {angularvelocityy}, {angularvelocityz})")
                    except ValueError:
                        rospy.logwarn("Error: Received data format is incorrect.")
                else:
                    # 如果没有接收到数据，等待新的连接或数据
                    break

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('docker_receive_odom', anonymous=True)

    # 创建发布者，发布到/Odometry话题
    pub = rospy.Publisher('/Odometry', Odometry, queue_size=2)

    while not rospy.is_shutdown():
        receivedata(pub)

# import socket
# import rospy
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Point, Quaternion, Vector3
# import threading

# class OdometryReceiver:
#     def __init__(self):
#         # Initialize ROS node
#         rospy.init_node('docker_receive_odom', anonymous=True)

#         # Create Odometry publisher
#         self.pub_odom = rospy.Publisher('/Odometry', Odometry, queue_size=2)

#         # Initialize the socket server in a separate thread
#         self.server_thread = threading.Thread(target=self.receivedata)
#         self.server_thread.daemon = True  # Ensure it exits when the program ends
#         self.server_thread.start()

#     def receivedata(self):
#         # Create a TCP/IP socket
#         with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
#             # Bind to a specific address and port
#             server_sock.bind(('0.0.0.0', 38992))  # Listening on port 38992 for odometry data
#             server_sock.listen(1)  # Max connection count is 1
#             rospy.logwarn("Server is listening on port 38992...")

#             # Wait for a connection
#             connection, client_address = server_sock.accept()
#             with connection:
#                 rospy.logwarn(f"Connection established with {client_address}")

#                 while not rospy.is_shutdown():
#                     # Continuously receive data (maximum 1024 bytes)
#                     data = connection.recv(1024)

#                     if data:
#                         # Decode received data into string
#                         received_str = data.decode('utf-8')
#                         # Process the received odometry data and publish it to /Odometry topic
#                         self.process_odometry_data(received_str)

#                     else:
#                         # If no data is received, wait for new connection or data
#                         break

#     def process_odometry_data(self, received_str):
#         try:
#             # Parse received string into expected values
#             values = received_str.split('#')
#             positionx = float(values[0])
#             positiony = float(values[1])
#             positionz = float(values[2])
#             orientationx = float(values[3])
#             orientationy = float(values[4])
#             orientationz = float(values[5])
#             orientationw = float(values[6])
#             linearvelocityx = float(values[7])
#             linearvelocityy = float(values[8])
#             linearvelocityz = float(values[9])
#             angularvelocityx = float(values[10])
#             angularvelocityy = float(values[11])
#             angularvelocityz = float(values[12])

#             # Create Odometry message object
#             odom_msg = Odometry()

#             # Set position information
#             odom_msg.pose.pose.position = Point(positionx, positiony, positionz)
#             odom_msg.pose.pose.orientation = Quaternion(orientationx, orientationy, orientationz, orientationw)

#             # Set velocity information
#             odom_msg.twist.twist.linear = Vector3(linearvelocityx, linearvelocityy, linearvelocityz)
#             odom_msg.twist.twist.angular = Vector3(angularvelocityx, angularvelocityy, angularvelocityz)

#             # Publish message to /Odometry topic
#             self.pub_odom.publish(odom_msg)
#             rospy.logwarn(f"Published Odometry data to /Odometry topic")

#         except ValueError:
#             rospy.logwarn("Error: Received data format is incorrect.")

# if __name__ == '__main__':
#     # Create the OdometryReceiver object
#     receiver = OdometryReceiver()

#     # Keep the node running
#     rospy.spin()