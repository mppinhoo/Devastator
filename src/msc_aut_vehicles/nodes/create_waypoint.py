#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import random
import math
import os

# Config
N_WAYPOINTS = 10
MIN_DISTANCE = 1.0
X_MIN = -3.0
X_MAX = 3.0
Y_MIN = -1.0
Y_MAX = 1.5
WAYPOINT_FILE = '/home/mpinho/proj_ws/waypoints/waypoints.csv'

class WaypointGenerator(Node):
    def __init__(self):
        super().__init__('waypoint_generator')

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )

        self.costmap_received = False
        self.free_cells = []
        self.goals = []

        self.get_logger().info("Waiting for costmap...")

    def costmap_callback(self, msg):
        if self.costmap_received:
            return
        self.costmap_received = True
        self.get_logger().info("Received costmap.")

        self.costmap = msg
        self.extract_free_space()
        self.generate_random_goals(N_WAYPOINTS)
        self.save_goals_to_file()

        self.get_logger().info("Waypoint generation complete. Shutting down.")
        rclpy.shutdown()

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
                if X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX:
                    self.free_cells.append((x, y))

        self.get_logger().info(f"Found {len(self.free_cells)} free cells.")

    def euclidean_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def generate_random_goals(self, n):
        self.goals.clear()
        #random.seed(32)

        if not self.free_cells:
            self.get_logger().error("No free cells available.")
            return

        x, y = random.choice(self.free_cells)
        yaw = random.uniform(-math.pi, math.pi)
        self.goals.append((x, y, yaw))

        while len(self.goals) < n:
            last_x, last_y, _ = self.goals[-1]
            candidates = [
                (cx, cy) for (cx, cy) in self.free_cells
                if self.euclidean_distance((last_x, last_y), (cx, cy)) >= MIN_DISTANCE
            ]
            if not candidates:
                self.get_logger().warn(f"Only generated {len(self.goals)} of {n} goals.")
                break
            x, y = random.choice(candidates)
            yaw = random.uniform(-math.pi, math.pi)
            self.goals.append((x, y, yaw))

        self.get_logger().info(f"Generated {len(self.goals)} waypoints.")

    def save_goals_to_file(self):
        try:
            with open(WAYPOINT_FILE, 'w') as f:
                for x, y, yaw in self.goals:
                    f.write(f"{x},{y},{yaw}\n")
            self.get_logger().info(f"Saved waypoints to {WAYPOINT_FILE}")
        except Exception as e:
            self.get_logger().error(f"Error saving waypoints: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointGenerator()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
