#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv
import datetime
import threading

class VelocityComparator(Node):
    def __init__(self):
        super().__init__('velocity_comparator')

        # Subscribe to topics
        self.cmd_vel_sub = self.create_subscription(Twist, '/diff_cont/cmd_vel_unstamped', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)

        # Open CSV file and write header
        self.csv_file = open("velocity_comparison.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["Time (HH:MM:SS.mmm)", "Cmd_Linear_X", "Cmd_Angular_Z", "Odom_Linear_X", "Odom_Angular_Z"])

        # Store the latest received cmd_vel and odom data
        self.latest_cmd_vel = None
        self.latest_odom = None

        # RAM buffer for messages
        self.data_buffer = []

        # Start a background timer to flush data every 1 second
        self.create_timer(5.0, self.flush_to_csv)

        self.get_logger().info("Velocity Comparator Node has started")

    def cmd_vel_callback(self, msg):
        """Callback for /cmd_vel topic."""
        # Store the latest cmd_vel message
        timestamp = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.latest_cmd_vel = [timestamp, msg.linear.x, msg.angular.z, None, None]
        self.try_to_log_data()

    def odom_callback(self, msg):
        """Callback for /diff_cont/odom topic."""
        # Store the latest odom message
        timestamp = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.latest_odom = [timestamp, None, None, msg.twist.twist.linear.x, msg.twist.twist.angular.z]
        self.try_to_log_data()

    def try_to_log_data(self):
        """Check if both cmd_vel and odom data are available, then log."""
        if self.latest_cmd_vel is not None and self.latest_odom is not None:
            # Combine both data sets into one
            self.latest_cmd_vel[3] = self.latest_odom[3]
            self.latest_cmd_vel[4] = self.latest_odom[4]
            # Add the combined data to the buffer
            self.data_buffer.append(self.latest_cmd_vel)
            # Clear the data after logging
            self.latest_cmd_vel = None
            self.latest_odom = None

    def flush_to_csv(self):
        """Flushes stored data to CSV every second."""
        if self.data_buffer:
            self.csv_writer.writerows(self.data_buffer)
            self.csv_file.flush()
            self.data_buffer = []  # Clear buffer after writing

    def destroy_node(self):
        """Ensures all data is saved before shutting down."""
        self.flush_to_csv()  # Final flush
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
