#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler
import random
import math
import time
from std_msgs.msg import Bool  # Import Bool message type
from nav2_msgs.action._navigate_to_pose import NavigateToPose_Result


N_WAYPOINTS = 5 # Number of random goals to generate
MIN_DISTANCE = 0.8  # Minimum distance between consecutive waypoints (meters)
X_MIN = 0.4
X_MAX = 3.0
Y_MIN = 0.0
Y_MAX = 3.0

class RandomWaypointNavigator(Node):
    def __init__(self):
        super().__init__('random_waypoint_navigator')

        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.waypoint_reached_pub = self.create_publisher(Bool, '/waypoint_reached', 10)  # Publisher for waypoint reached signal

        self.costmap_received = False
        self.free_cells = []
        self.goals = []
        self.current_goal_index = 0

        self.get_logger().info("Waiting for costmap...")

    def costmap_callback(self, msg):
        if not self.costmap_received:
            self.costmap_received = True
            self.get_logger().info("Received costmap.")
            self.costmap = msg
            self.extract_free_space()
            self.generate_random_goals(N_WAYPOINTS)

            self._action_client.wait_for_server()
            self.get_logger().info("Action server ready. Starting navigation.")
            self.send_next_goal()

    def extract_free_space(self):
        self.free_cells.clear()
        data = self.costmap.data
        width = self.costmap.info.width
        resolution = self.costmap.info.resolution
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y

        for i, value in enumerate(data):
            if value != -1 and value < 80:
                mx = i % width
                my = i // width
                x = origin_x + mx * resolution
                y = origin_y + my * resolution
                
                # Only include the cell if it's within the defined bounds
                if X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX:
                    self.free_cells.append((x, y))

        self.get_logger().info(f"Found {len(self.free_cells)} free cells.")

    def euclidean_distance(self, p1, p2):
        """Calculates Euclidean distance between two points."""
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def generate_random_goals(self, n):
        self.goals.clear()
        random.seed(42)

        if not self.free_cells:
            self.get_logger().error("No free cells available to generate goals.")
            return

        # Start from a random free cell
        x, y = random.choice(self.free_cells)
        yaw = random.uniform(-math.pi, math.pi)
        self.goals.append((x, y, yaw))

        while len(self.goals) < n:
            last_x, last_y, _ = self.goals[-1]

            # Filter free cells at least MIN_DISTANCE from the previous one
            candidates = [
                (cx, cy) for (cx, cy) in self.free_cells
                if self.euclidean_distance((last_x, last_y), (cx, cy)) >= MIN_DISTANCE
            ]

            if not candidates:
                self.get_logger().warn(f"Could only generate {len(self.goals)} goals out of requested {n}.")
                break

            x, y = random.choice(candidates)
            yaw = random.uniform(-math.pi, math.pi)
            self.goals.append((x, y, yaw))

        self.get_logger().info(f"Generated {len(self.goals)} random goals.")

    def send_next_goal(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info("All waypoints completed.")
            rclpy.shutdown()
            return

        x, y, yaw = self.goals[self.current_goal_index]

        # Create PoseStamped with timestamp NOW
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

        # Publish goal to /goal_pose at the exact time it is being used
        self.goal_pub.publish(pose)
        self.get_logger().info(f"Published goal {self.current_goal_index+1} to /goal_pose.")

        # Send to NavigateToPose action server
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"Sending goal {self.current_goal_index+1}/{len(self.goals)} to ({x:.2f}, {y:.2f})")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg
            # feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(f"Distance remaining: {feedback.feedback.distance_remaining:.2f} m")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected.")
            # Do NOT increment or call send_next_goal here
            self.handle_goal_result(success=False)
            return

        self.get_logger().info("Goal accepted.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:
            self.get_logger().info(f"Goal {self.current_goal_index+1} succeeded.")
            waypoint_reached_msg = Bool()
            waypoint_reached_msg.data = True
            self.waypoint_reached_pub.publish(waypoint_reached_msg)
            self.current_goal_index += 1
            time.sleep(2)  # Optional delay
            self.send_next_goal()

        elif status == 6:  # ABORTED
            self.get_logger().warn(f"Goal {self.current_goal_index + 1} was aborted. Retrying...")
            self.send_next_goal()  # retry same goal without incrementing

        else:
            self.get_logger().warn(
                f"Goal {self.current_goal_index + 1} ended with unexpected status: {status}. Skipping..."
            )

        

def main(args=None):
    rclpy.init(args=args)
    node = RandomWaypointNavigator()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
