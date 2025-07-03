#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import csv
import os
from datetime import datetime
import numpy as np

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)

        self.plan_sub = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10)

        self.waypoint_reached_sub = self.create_subscription(
            Bool,
            '/waypoint_reached',
            self.waypoint_reached_callback,
            10)

        self.csv_file = 'waypoint_timings.csv'
        self.waypoint_counter = 0
        self.awaiting_plan = False
        self.goal_time = None
        self.plan_time = None
        self.waypoint_reached_time = None
        self.path_poses = []
        self.current_goal_pose = None

        # Start clean
        if os.path.isfile(self.csv_file):
            os.remove(self.csv_file)

        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "waypoint number",
                "goal (x, y)",
                "goal yaw (rad)",  # ✅ radians
                "goal pose time",
                "plan time",
                "waypoint reached time",
                "path poses"
            ])

    def goal_callback(self, msg):
        self.waypoint_counter += 1
        self.awaiting_plan = True
        self.goal_time = self.stamp_to_str(msg.header.stamp)

        pos = msg.pose.position
        ori = msg.pose.orientation
        yaw = self.quaternion_to_yaw(ori)
        self.current_goal_pose = {
            "x": round(pos.x, 3),
            "y": round(pos.y, 3),
            "yaw_rad": round(yaw, 4)  # 🔢 precision to 4 decimals
        }

    def plan_callback(self, msg):
        if self.awaiting_plan and self.goal_time is not None:
            self.plan_time = self.stamp_to_str(msg.header.stamp)
            self.path_poses = self.collect_path_poses(msg)
            self.awaiting_plan = False

    def waypoint_reached_callback(self, msg):
        if msg.data and self.waypoint_counter > 0:
            self.waypoint_reached_time = self.stamp_to_str(self.get_clock().now().to_msg())
            self.log_to_csv(
                self.waypoint_counter,
                self.current_goal_pose,
                self.goal_time,
                self.plan_time,
                self.waypoint_reached_time,
                self.path_poses
            )

    def log_to_csv(self, waypoint, goal_pose, goal_time, plan_time, reached_time, path_poses):
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                waypoint,
                f"{goal_pose['x']}, {goal_pose['y']}",
                goal_pose['yaw_rad'],
                goal_time,
                plan_time,
                reached_time,
                path_poses
            ])

    def collect_path_poses(self, path_msg):
        return "; ".join(
            f"({round(p.pose.position.x, 2)}, {round(p.pose.position.y, 2)})"
            for p in path_msg.poses
        )

    def quaternion_to_yaw(self, orientation):
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy**2 + qz**2)
        return np.arctan2(siny_cosp, cosy_cosp)

    def stamp_to_str(self, stamp):
        total_time = stamp.sec + stamp.nanosec * 1e-9
        return datetime.fromtimestamp(total_time).strftime('%H:%M:%S.%f')[:-3]

def main(args=None):
    rclpy.init(args=args)
    node = WaypointLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
