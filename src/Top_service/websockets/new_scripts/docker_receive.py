#!/usr/bin/env python
import rospy
import asyncio
import websockets.client as websockets
import json
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

# Receive and publish Odometry data via WebSocket
async def receive_data_odom(pub_odom):
    ws_url = "ws://0.0.0.0:38123"  # WebSocket URL for odometry data
    try:
        # Connect to WebSocket server
        async with websockets.connect(ws_url) as websocket:
            rospy.logwarn(f"Connected to WebSocket server at {ws_url}")

            while not rospy.is_shutdown():
                # Wait for incoming message
                data = await websocket.recv()

                if data:
                    # Decode received data
                    received_data = json.loads(data)

                    try:
                        # Parse the received data (Odometry)
                        position = received_data["position"]
                        orientation = received_data["orientation"]
                        linear_velocity = received_data["linear_velocity"]
                        angular_velocity = received_data["angular_velocity"]

                        # Create Odometry message
                        odom_msg = Odometry()
                        odom_msg.pose.pose.position = Point(*position)
                        odom_msg.pose.pose.orientation = Quaternion(*orientation)
                        odom_msg.twist.twist.linear = Vector3(*linear_velocity)
                        odom_msg.twist.twist.angular = Vector3(*angular_velocity)

                        # Publish the Odometry message
                        pub_odom.publish(odom_msg)
                        rospy.logwarn(f"Published Odometry data to /Odometry topic")

                    except KeyError:
                        rospy.logwarn(f"Error: Missing expected keys in received odometry data.")

    except Exception as e:
        rospy.logwarn(f"Error in WebSocket connection: {e}")


# Receive and publish PointCloud2 data via WebSocket
async def receive_data_cloud(pub_cloud):
    ws_url = "ws://0.0.0.0:38124"  # WebSocket URL for point cloud data
    try:
        # Connect to WebSocket server
        async with websockets.connect(ws_url) as websocket:
            rospy.logwarn(f"Connected to WebSocket server at {ws_url}")

            while not rospy.is_shutdown():
                # Wait for incoming message
                data = await websocket.recv()

                if data:
                    # Decode received data
                    received_data = json.loads(data)

                    try:
                        # Process point cloud data
                        point_data = received_data["points"]

                        # Convert to (x, y, z) tuples
                        point_data_tuples = [(p["x"], p["y"], p["z"]) for p in point_data]

                        # Create PointCloud2 message
                        header = Header()
                        header.stamp = rospy.Time.now()
                        header.frame_id = "map"  # Assuming point cloud is in "map" frame

                        pc_data = pc2.create_cloud_xyz32(header, point_data_tuples)

                        # Publish the PointCloud2 message
                        pub_cloud.publish(pc_data)
                        rospy.logwarn(f"Published PointCloud2 data to /cloud_registered topic")

                    except KeyError:
                        rospy.logwarn(f"Error: Missing expected keys in received point cloud data.")

    except Exception as e:
        rospy.logwarn(f"Error in WebSocket connection: {e}")


def main():
    # Initialize ROS node
    rospy.init_node('docker_receive_data', anonymous=True)

    # Create publishers for odometry and point cloud
    pub_odom = rospy.Publisher('/Odometry', Odometry, queue_size=2)
    pub_cloud = rospy.Publisher('/cloud_registered', PointCloud2, queue_size=2)

    # Start WebSocket listeners for both odometry and point cloud data
    loop = asyncio.get_event_loop()
    loop.create_task(receive_data_odom(pub_odom))
    loop.create_task(receive_data_cloud(pub_cloud))

    # Run the event loop
    loop.run_forever()


if __name__ == '__main__':
    main()