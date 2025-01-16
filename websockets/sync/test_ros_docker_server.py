import socket
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3

def receivedata(pub):
    # 创建一个 TCP/IP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as serversock:
        # 绑定到特定的地址和端口
        serversock.bind(('0.0.0.0', 38979)) 
        serversock.listen(1)  # 最大连接数为1
    print("Server is listening on port 38979...")

    # 等待连接
    connection, client_address = server_sock.accept()

    with connection:
        print(f"Connection established with {client_address}")

        while not rospy.is_shutdown():
            # 持续接收数据（最大1024字节）
            data = connection.recv(1024)

            if data:
                # 输出接收到的数据
                received_str = data.decode('utf-8')
                print(f"Received data: {received_str}")

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

                    # 设置位置信息
                    odom_msg.pose.pose.position = Point(positionx, positiony, positionz)
                    odom_msg.pose.pose.orientation = Quaternion(orientationx, orientationy, orientationz, orientationw)

                    # 设置速度信息
                    odom_msg.twist.twist.linear = Vector3(linearvelocityx, linearvelocityy, linearvelocityz)
                    odom_msg.twist.twist.angular = Vector3(angularvelocityx, angularvelocityy, angularvelocityz)

                    # 发布消息到 /Odometry
                    pub.publish(odom_msg)
                    print(f"Published to /Odometry: position = ({positionx}, {positiony}, {positionz}), "
                          f"orientation = ({orientationx}, {orientationy}, {orientationz}, {orientationw}), "
                          f"linear_velocity = ({linearvelocityx}, {linearvelocityy}, {linearvelocityz}), "
                          f"angular_velocity = ({angularvelocityx}, {angularvelocityy}, {angularvelocityz})")
                except ValueError:
                    print("Error: Received data format is incorrect.")
            else:
                # 如果没有接收到数据，等待新的连接或数据
                break

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('datareceiver', anonymous=True)

    # 创建发布者，发布到/Odometry话题
    pub = rospy.Publisher('/Odometry', Odometry, queue_size=2)

    while not rospy.is_shutdown():
        receivedata(pub)
