#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # Import Odometry message
import csv
import datetime

class VelocityComparator(Node):
    def __init__(self):
        super().__init__('velocity_comparator')

        # Subscribe to /cmd_vel topic
        self.cmd_vel_sub = self.create_subscription(Twist, '/diff_cont/cmd_vel_unstamped', self.cmd_vel_callback, 10)
        # Subscribe to /diff_cont/odom topic
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)

        # Open CSV file and write header
        self.csv_file = open("velocity_comparison.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["Time (HH:MM:SS.mmm)", 
                                  "Cmd_Linear_X", "Cmd_Angular_Z", 
                                  "Odom_Linear_X", "Odom_Angular_Z"])

        # Store latest values
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.odom_linear_x = 0.0
        self.odom_angular_z = 0.0

        self.get_logger().info("Velocity Comparator Node has started")

    def cmd_vel_callback(self, msg):
        """Callback for /cmd_vel topic."""
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z
        self.log_data()

    def odom_callback(self, msg):
        """Callback for /diff_cont/odom topic."""
        self.odom_linear_x = msg.twist.twist.linear.x
        self.odom_angular_z = msg.twist.twist.angular.z
        self.log_data()

    def log_data(self):
        """Log the current velocity data to CSV."""
        timestamp = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]  # Trim to milliseconds
        self.csv_writer.writerow([
            timestamp,
            self.cmd_linear_x, self.cmd_angular_z,
            self.odom_linear_x, self.odom_angular_z
        ])
        self.csv_file.flush()  # Ensure data is written immediately

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VelocityComparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
