#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler
import time
import math

WAYPOINT_FILE = '/home/mpinho/proj_ws/waypoints/waypoints.csv'

class WaypointExecutor(Node):
    def __init__(self):
        super().__init__('waypoint_executor')

        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.waypoint_reached_pub = self.create_publisher(Bool, '/waypoint_reached', 10)

        self.goals = self.load_goals_from_file()
        self.current_goal_index = 0

        self._action_client.wait_for_server()
        self.get_logger().info("Action server ready. Starting navigation.")
        self.send_next_goal()

    def load_goals_from_file(self):
        goals = []
        try:
            with open(WAYPOINT_FILE, 'r') as f:
                for line in f:
                    x, y, yaw = map(float, line.strip().split(','))
                    goals.append((x, y, yaw))
            self.get_logger().info(f"Loaded {len(goals)} waypoints.")
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")
        return goals

    def send_next_goal(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info("All waypoints completed.")
            rclpy.shutdown()
            return

        x, y, yaw = self.goals[self.current_goal_index]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.goal_pub.publish(pose)
        self.get_logger().info(f"Published goal {self.current_goal_index+1} to /goal_pose.")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"Sending goal {self.current_goal_index+1}/{len(self.goals)} to ({x:.2f}, {y:.2f})")

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected.")
            return
        self.get_logger().info("Goal accepted.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        if status == 4:  # SUCCESS
            self.get_logger().info(f"Goal {self.current_goal_index+1} succeeded.")
            waypoint_reached_msg = Bool()
            waypoint_reached_msg.data = True
            self.waypoint_reached_pub.publish(waypoint_reached_msg)
            self.current_goal_index += 1
            time.sleep(2)
            self.send_next_goal()

        elif status == 6:  # ABORTED
            self.get_logger().warn(f"Goal {self.current_goal_index + 1} was aborted. Retrying...")
            self.send_next_goal()

        else:
            self.get_logger().warn(f"Goal {self.current_goal_index + 1} failed with status: {status}. Skipping...")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointExecutor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
