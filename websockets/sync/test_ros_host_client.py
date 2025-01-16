import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # Import the Odometry message type
import socket

class TestaListener(Node):
    def __init__(self):
        super().__init__('testalistener')
        # Subscription to the '/tower/mapping/odometry' topic using Odometry message type
        self.subscription = self.create_subscription(
            Odometry,  # Change to Odometry
            '/tower/mapping/odometry',  # New topic name
            self.callback,
            10
        )

    def callback(self, msg):
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
        self.get_logger().info(f"Linear Velocity: ({linear_velocity_x}, {linear_velocity_y}, {linear_velocity_z})")
        self.get_logger().info(f"Angular Velocity: ({angular_velocity_x}, {angular_velocity_y}, {angular_velocity_z})")
        
        # Prepare data to send over socket
        data = f"{position_x}#{position_y}#{position_z}#" \
               f"{orientation_x}#{orientation_y}#{orientation_z}#{orientation_w}#" \
               f"{linear_velocity_x}#{linear_velocity_y}#{linear_velocity_z}#" \
               f"{angular_velocity_x}#{angular_velocity_y}#{angular_velocity_z}"
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect(('0.0.0.0', 38991))  # Connect to System B on port 38979
                self.get_logger().info("Sending message to System B")
                sock.sendall(data.encode('utf-8'))  # Send the data as UTF-8 encoded string
        except socket.error as exc:
            self.get_logger().error(f"Caught exception socket.error: {exc}")

def main(args=None):
    rclpy.init(args=args)
    node = TestaListener()
    rclpy.spin(node)

    # Destroy the node explicitly after the spin is finished
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
