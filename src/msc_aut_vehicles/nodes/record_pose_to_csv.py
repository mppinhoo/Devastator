#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv
import datetime
import numpy as np

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')
        
        # Subscribe to the /pose topic with PoseWithCovarianceStamped type
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,  # Changed to PoseWithCovarianceStamped
            '/pose',
            self.pose_callback,
            10
        )

        # Open CSV file for writing
        self.csv_file = open('pose_log.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header row
        self.csv_writer.writerow(['Time (HH:MM:SS.sss)', 'X', 'Y', 'Quaternion X', 'Quaternion X', 'Quaternion X', 'Quaternion X', 'Yaw'])

    def pose_callback(self, msg):
        # Convert timestamp to float seconds
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Convert to human-readable time (HH:MM:SS.sss)
        human_time = datetime.datetime.fromtimestamp(timestamp).strftime('%H:%M:%S.%f')[:-3]  # Trim to milliseconds
        
        # Extract pose data from PoseWithCovarianceStamped
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_x = msg.pose.pose.orientation.x
        orientation_y = msg.pose.pose.orientation.y
        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w

        siny_cosp = 2 * (orientation_w * orientation_z + orientation_x * orientation_y)
        cosy_cosp = 1 - 2 * (orientation_y * orientation_y + orientation_z * orientation_z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Write to CSV
        self.csv_writer.writerow([human_time, x, y, orientation_x, orientation_y, orientation_z, orientation_w, yaw])
        #self.get_logger().info(f'Logged Pose: Time={human_time}, X={x}, Y={y}, Yaw={yaw}')

    def destroy_node(self):
        # Close file on shutdown
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PoseLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
