#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # Import the Odometry message type
from sensor_msgs.msg import PointCloud2  # Import the PointCloud2 message type
import socket
import struct
import std_msgs.msg

class TestaListener(Node):
    def __init__(self):
        super().__init__('host_send')

        # Subscription to the '/tower/mapping/odometry' topic using Odometry message type
        self.subscription_odometry = self.create_subscription(
            Odometry,  # Odometry message type
            '/tower/mapping/odometry',  # Topic name
            self.callback_odometry,
            10
        )

        # Subscription to the '/tower/mapping/cloud' topic using PointCloud2 message type
        self.subscription_cloud = self.create_subscription(
            PointCloud2,  # PointCloud2 message type
            '/tower/mapping/cloud',  # New topic name
            self.callback_cloud,
            10
        )

    def callback_odometry(self, msg):
        # Extracting various fields from the Odometry message
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        position_z = msg.pose.pose.position.z
        orientation_x = msg.pose.pose.orientation.x
        orientation_y = msg.pose.pose.orientation.y
        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w
        linear_velocity_x = msg.twist.twist.linear.x
        linear_velocity_y = msg.twist.twist.linear.y
        linear_velocity_z = msg.twist.twist.linear.z
        angular_velocity_x = msg.twist.twist.angular.x
        angular_velocity_y = msg.twist.twist.angular.y
        angular_velocity_z = msg.twist.twist.angular.z
        
        # Log the received data for debugging
        self.get_logger().info(f"Position: ({position_x}, {position_y}, {position_z})")
        self.get_logger().info(f"Orientation: ({orientation_x}, {orientation_y}, {orientation_z}, {orientation_w})")
        # self.get_logger().info(f"Linear Velocity: ({linear_velocity_x}, {linear_velocity_y}, {linear_velocity_z})")
        # self.get_logger().info(f"Angular Velocity: ({angular_velocity_x}, {angular_velocity_y}, {angular_velocity_z})")
        
        # Prepare data to send over socket
        data = f"{position_x}#{position_y}#{position_z}#" \
               f"{orientation_x}#{orientation_y}#{orientation_z}#{orientation_w}#" \
               f"{linear_velocity_x}#{linear_velocity_y}#{linear_velocity_z}#" \
               f"{angular_velocity_x}#{angular_velocity_y}#{angular_velocity_z}"
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect(('0.0.0.0', 38992))  # Connect to System B on port 38979
                self.get_logger().info("Sending message to System B")
                sock.sendall(data.encode('utf-8'))  # Send the data as UTF-8 encoded string
        except socket.error as exc:
            self.get_logger().error(f"Caught exception socket.error: {exc}")

    def callback_cloud(self, msg):
        # Extract the raw point cloud data and its metadata
        points = self.extract_points_from_pointcloud(msg)
        
        # Log some information about the point cloud (e.g., number of points)
        self.get_logger().info(f"Received PointCloud2 with {len(points)} points")
        
        # Prepare the point cloud data as a string
        point_data = '#'.join(map(str, points))
        points_1 = []
        for item in point_data.split('#'):
            try:
                points_1.append(float(item))
            except ValueError:
                print("Skipping invalid value: ",item)  # Log and skip invalid values
        if len(points_1) % 3 != 0:
           print("Received point cloud data is incomplete. Skipping.")
           return  # Skip this set if the data is incomplete
        # Group points_1 into (x, y, z) tuples
        point_data1 = [points_1[i:i+3] for i in range(0, len(points_1), 3)]  # Group into (x, y, z) tuples
        # Count the number of points
        num_points = len(point_data1)
        print("send ",num_points," points in the point cloud")
        
        
        # print("Sending PointCloud2 data:", point_data)
        # Send the point cloud data over the socket
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect(('0.0.0.0', 38994))  # Connect to System B
                self.get_logger().info("Sending PointCloud2 data to System B")
                sock.sendall(point_data.encode('utf-8'))  # Send the data as UTF-8 encoded string
        except socket.error as exc:
            self.get_logger().error(f"Caught exception socket.error: {exc}")

    def extract_points_from_pointcloud(self, msg):
        # This function extracts 3D points from a PointCloud2 message.
        # Assume that the point cloud has x, y, z fields (in a typical format).
        points = []
        for data in self.get_pointcloud_data(msg):
            points.append(data[0])  # X
            points.append(data[1])  # Y
            points.append(data[2])  # Z
        return points

    def get_pointcloud_data(self, msg):
        # Parse the PointCloud2 message to extract point data.
        # This is just a simplified example. In a real case, you'd need to unpack the data.
        import sensor_msgs_py.point_cloud2 as pc2
        
        point_list = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        return list(point_list)

def main(args=None):
    rclpy.init(args=args)
    node = TestaListener()
    rclpy.spin(node)

    # Destroy the node explicitly after the spin is finished
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
