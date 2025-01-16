import socket
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

def receivedata(pub_cloud):
    # Create a TCP/IP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        # Bind to a specific address and port
        server_sock.bind(('0.0.0.0', 38993))  # Listening on port 38993 for point cloud data
        server_sock.listen(1)  # Max connection count is 1
        rospy.loginfo("Server is listening on port 38993...")

        # Wait for a connection
        connection, client_address = server_sock.accept()

        with connection:
            rospy.loginfo(f"Connection established with {client_address}")

            while not rospy.is_shutdown():
                # Continuously receive data (maximum 1024 bytes)
                data = connection.recv(1024)

                if data:
                    # Decode received data into string
                    received_str = data.decode('utf-8')
                    rospy.loginfo(f"Received point cloud data: {received_str}")

                    # Process the received point cloud data and publish it to /cloud_registered topic
                    try:
                        # Convert received data into point cloud format
                        points = list(map(float, received_str.split('#')))  # Split and convert to float
                        point_data = [points[i:i+3] for i in range(0, len(points), 3)]  # Group into (x, y, z) tuples

                        # Create a PointCloud2 message
                        header = Header()
                        header.stamp = rospy.Time.now()
                        header.frame_id = "map"  # Assuming the point cloud is in the "base_link" frame

                        # Create the point cloud data (assuming 32-bit float XYZ format)
                        pc_data = pc2.create_cloud_xyz32(header, point_data)

                        # Publish PointCloud2 message to /cloud_registered topic
                        pub_cloud.publish(pc_data)
                        rospy.loginfo(f"Published PointCloud2 data to /cloud_registered topic")

                    except Exception as e:
                        rospy.logwarn(f"Error processing point cloud data: {e}")
                else:
                    # If no data is received, wait for new connection or data
                    break

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('pointcloud_receiver', anonymous=True)

    # Create PointCloud2 publisher
    pub_cloud = rospy.Publisher('/cloud_registered', PointCloud2, queue_size=2)

    # Call the function to listen and process point cloud data
    while not rospy.is_shutdown():
        receivedata(pub_cloud)
