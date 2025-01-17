#!/usr/bin/env python3
import socket
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

def receivedata(pub_cloud):
    # Create a TCP/IP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        # Bind to a specific address and port
        server_sock.bind(('0.0.0.0', 38994))  # Listening on port 38994 for point cloud data
        server_sock.listen(1)  # Max connection count is 1
        rospy.logwarn("Server is listening on port 38994...")

        # Wait for a connection
        connection, client_address = server_sock.accept()

        with connection:
            rospy.logwarn(f"Connection established with {client_address}")

            while not rospy.is_shutdown():
                # Continuously receive data (maximum 1024 bytes)
                data = connection.recv(1024*100)

                if data:
                    # Decode received data into string
                    received_str = data.decode('utf-8')
                    # print(f"Received data: {received_str}")

                    # Process the received point cloud data and publish it to /cloud_registered topic
                    try:
                        # Split received string into list and filter out non-numeric values
                        points = []
                        for item in received_str.split('#'):
                            try:
                                points.append(float(item))
                            except ValueError:
                                rospy.logwarn(f"Skipping invalid value: {item}")  # Log and skip invalid values
                        
                        if len(points) % 3 != 0:
                            rospy.logwarn("Received point cloud data is incomplete. Skipping.")
                            continue  # Skip this set if the data is incomplete

                        # Group points into (x, y, z) tuples
                        point_data = [points[i:i+3] for i in range(0, len(points), 3)]  # Group into (x, y, z) tuples
                        
                        # Count the number of points
                        num_points = len(point_data)
                        rospy.logwarn(f"Received {num_points} points in the point cloud")

                        # Create a PointCloud2 message
                        header = Header()
                        header.stamp = rospy.Time.now()
                        header.frame_id = "map"  # Assuming the point cloud is in the "base_link" frame

                        # Create the point cloud data (assuming 32-bit float XYZ format)
                        pc_data = pc2.create_cloud_xyz32(header, point_data)

                        # Publish PointCloud2 message to /cloud_registered topic
                        pub_cloud.publish(pc_data)
                        rospy.logwarn(f"Published PointCloud2 data to /cloud_registered topic")

                    except Exception as e:
                        rospy.logwarn(f"Error processing point cloud data: {e}")
                else:
                    # If no data is received, wait for new connection or data
                    break

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('docker_receive_cloud', anonymous=True)

    # Create PointCloud2 publisher
    pub_cloud = rospy.Publisher('/cloud_registered', PointCloud2, queue_size=2)

    # Call the function to listen and process point cloud data
    while not rospy.is_shutdown():
        receivedata(pub_cloud)
