import socket
import rospy
from geometry_msgs.msg import Twist

def receive_data(pub):
    # 创建一个 TCP/IP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        # 绑定到特定的地址和端口
        server_sock.bind(('0.0.0.0', 38979)) 
        server_sock.listen(1)  # 最大连接数为1

        print("Server is listening on port 38979...")

        # 等待连接
        connection, client_address = server_sock.accept()

        with connection:
            print(f"Connection established with {client_address}")
            
            while not rospy.is_shutdown():
                # 持续接收数据（最大1024字节）pub
                data = connection.recv(1024)
                
                if data:
                    # 输出接收到的数据
                    received_str = data.decode('utf-8')
                    print(f"Received data: {received_str}")
                    
                    # 解析接收到的字符串，假设格式为 "1.0#0.0"
                    try:
                        linear_str, angular_str = received_str.split('#')
                        linear_x = float(linear_str)
                        angular_z = float(angular_str)
                        
                        # 创建 Twist 消息对象
                        twist_msg = Twist()
                        twist_msg.linear.x = linear_x
                        twist_msg.angular.z = angular_z
                        
                        # 发布消息到 /cmd_vel
                        pub.publish(twist_msg)
                        print(f"Published to /cmd_vel: linear.x = {linear_x}, angular.z = {angular_z}")
                    except ValueError:
                        print("Error: Received data format is incorrect.")
                else:
                    # 如果没有接收到数据，等待新的连接或数据
                    break

if __name__ == '__main__':
        # 初始化ROS节点
    rospy.init_node('data_receiver', anonymous=True)
    # 创建发布者，发布到/cmd_vel话题
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
    while True:
        receive_data(pub)
