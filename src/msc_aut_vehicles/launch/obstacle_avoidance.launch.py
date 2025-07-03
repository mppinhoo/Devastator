import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            'lidar_topic',  # Replace 'lidar_topic' with your actual lidar topic
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',  # Publish to the command velocity topic
            10)

    def lidar_callback(self, msg):
        # Process lidar data to detect obstacles
        # For simplicity, let's assume lidar data is an array of ranges
        obstacle_threshold = 0.1

        for range_value in msg.ranges:
            if range_value < obstacle_threshold:  # Define your obstacle threshold
                # If obstacle detected, stop the robot by publishing zero velocities
                cmd_vel_msg = Twist()
                self.publisher.publish(cmd_vel_msg)
                return  # Exit the loop after detecting the first obstacle

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)
    obstacle_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('msc_aut_vehicles')  # Replace 'your_package_name' with your actual package name

    # Create the launch description
    ld = LaunchDescription()

    # Add the ObstacleDetector node
    obstacle_detector_node = Node(
        package='your_package_name',  # Replace 'your_package_name' with your actual package name
        executable='obstacle_detector_node',  # The name of the executable for your ObstacleDetector node
        output='screen',  # Output to screen for debugging
        parameters=[{
            'lidar_topic': '/lidar_topic',  # Replace '/lidar_topic' with the actual lidar topic
            'obstacle_threshold': 1.0  # Define the obstacle threshold
        }]
    )

    # Add the node to the launch description
    ld.add_action(obstacle_detector_node)

    return ld
