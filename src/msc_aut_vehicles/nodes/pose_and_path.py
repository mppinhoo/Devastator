#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
import csv
import datetime
import numpy as np

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10
        )

        self.plan_sub = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10
        )

        # Store the latest plan
        self.latest_plan = None

        # Open CSV file for writing
        self.csv_file = open('pose_log.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header row
        self.csv_writer.writerow([
            'Time (HH:MM:SS.sss)', 'X', 'Y',
            'Quaternion X', 'Quaternion Y', 'Quaternion Z', 'Quaternion W', 'Yaw',
            'Plan (x0,y0; x1,y1; ...)'
        ])

    def plan_callback(self, msg):
        self.latest_plan = msg

    def pose_callback(self, msg):
        # Timestamp
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        human_time = datetime.datetime.fromtimestamp(timestamp).strftime('%H:%M:%S.%f')[:-3]

        # Pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Yaw
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Plan path as string: "x0,y0; x1,y1; ..."
        if self.latest_plan is not None:
            plan_str = '; '.join(
                f"{pose.pose.position.x:.2f},{pose.pose.position.y:.2f}"
                for pose in self.latest_plan.poses
            )
        else:
            plan_str = 'None'

        # Write to CSV
        self.csv_writer.writerow([human_time, x, y, qx, qy, qz, qw, yaw, plan_str])
        # self.get_logger().info(f'Logged at {human_time} with plan of {len(self.latest_plan.poses) if self.latest_plan else 0} points')

    def destroy_node(self):
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
