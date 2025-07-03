#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import skfuzzy as fuzz
import skfuzzy.control as ctrl

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Replace '/lidar_topic' with the actual lidar topic
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel_stop', 10)

        self.cmd_vel_joy_msg = Twist()  # Joystick control command
        self.cmd_vel_msg = Twist()      # To publish adjusted command

        # Constants for distances
        self.StopDist = 0.18
        self.Ddist = 0.35  # Danger distance
        self.Sdist = 0.8  # Safety distance
        self.MIN_LIDAR_RANGE = 0.1  # Minimum valid lidar range

        # Fuzzy logic setup
        self.setup_fuzzy_logic()

    def setup_fuzzy_logic(self):
        # Fuzzy Variables
        self.obstacle_distance1 = ctrl.Antecedent(np.arange(0, 2, 0.01), 'obstacle_distance1')
        self.obstacle_distance2 = ctrl.Antecedent(np.arange(0, 2, 0.01), 'obstacle_distance2')
        self.obstacle_distance3 = ctrl.Antecedent(np.arange(0, 2, 0.01), 'obstacle_distance3')
        self.speed = ctrl.Consequent(np.arange(0, 1.1, 0.01), 'speed')
        self.steering = ctrl.Consequent(np.arange(-1.0, 1.0, 0.01), 'steering')

        # Membership functions for distance
        self.obstacle_distance1['near'] = fuzz.trimf(self.obstacle_distance1.universe, [0, 0, 0.5])
        self.obstacle_distance1['medium'] = fuzz.trimf(self.obstacle_distance1.universe, [0.2, 0.75, 1.2])
        self.obstacle_distance1['far'] = fuzz.trimf(self.obstacle_distance1.universe, [0.8, 1.5, 1.5])

        self.obstacle_distance2['near'] = fuzz.trimf(self.obstacle_distance2.universe, [0, 0, 0.5])
        self.obstacle_distance2['medium'] = fuzz.trimf(self.obstacle_distance2.universe, [0.2, 0.75, 1.2])
        self.obstacle_distance2['far'] = fuzz.trimf(self.obstacle_distance2.universe, [0.8, 1.5, 1.5])

        self.obstacle_distance3['near'] = fuzz.trimf(self.obstacle_distance3.universe, [0, 0, 0.5])
        self.obstacle_distance3['medium'] = fuzz.trimf(self.obstacle_distance3.universe, [0.2, 0.75, 1.2])
        self.obstacle_distance3['far'] = fuzz.trimf(self.obstacle_distance3.universe, [0.8, 1.5, 1.5])

        # Membership functions for speed
        self.speed['stop'] = fuzz.trimf(self.speed.universe, [0, 0, 0.1])
        self.speed['slow'] = fuzz.trimf(self.speed.universe, [0.05, 0.3, 0.6])
        self.speed['normal'] = fuzz.trimf(self.speed.universe, [0.4, 1.0, 1.0])

        # Membership functions for steering
        self.steering['left'] = fuzz.trimf(self.steering.universe, [-1.0, -0.5, 0])
        self.steering['straight'] = fuzz.trimf(self.steering.universe, [-0.2, 0, 0.2])
        self.steering['right'] = fuzz.trimf(self.steering.universe, [0, 0.5, 1.0])

        # Fuzzy rules
        rule1 = ctrl.Rule(self.obstacle_distance1['near'] | self.obstacle_distance2['near'] | self.obstacle_distance3['near'], self.speed['stop'])
        rule2 = ctrl.Rule(self.obstacle_distance1['medium'] | self.obstacle_distance2['medium'], self.speed['slow'])
        rule3 = ctrl.Rule(self.obstacle_distance3['far'], self.speed['normal'])
        rule4 = ctrl.Rule(self.obstacle_distance1['near'], self.steering['left'])
        rule5 = ctrl.Rule(self.obstacle_distance2['near'], self.steering['right'])
        rule6 = ctrl.Rule(self.obstacle_distance3['far'], self.steering['straight'])

        # Control systems
        self.speed_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
        self.steering_ctrl = ctrl.ControlSystem([rule4, rule5, rule6])

        self.speed_sim = ctrl.ControlSystemSimulation(self.speed_ctrl)
        self.steering_sim = ctrl.ControlSystemSimulation(self.steering_ctrl)

    def lidar_callback(self, msg):
        all_ranges = msg.ranges

        # Calculate indices for 3/8 and 5/8 of the ranges
        num_ranges = len(all_ranges)
        start_index1 = int(60 * num_ranges / 240)
        end_index1 = int(180 * num_ranges / 240)
        start_index2 = int(75 * num_ranges / 240)
        end_index2 = int(165 * num_ranges / 240)
        start_index3 = int(90 * num_ranges / 240)
        end_index3 = int(150 * num_ranges / 240)

        ranges1 = all_ranges[start_index1:end_index1]
        ranges2 = all_ranges[start_index2:end_index2]
        ranges3 = all_ranges[start_index3:end_index3]

        # Filter out invalid ranges (values below MIN_LIDAR_RANGE)
        valid_ranges1 = [r for r in ranges1 if r > self.MIN_LIDAR_RANGE]
        valid_ranges2 = [r for r in ranges2 if r > self.MIN_LIDAR_RANGE]
        valid_ranges3 = [r for r in ranges3 if r > self.MIN_LIDAR_RANGE]

        # If no valid ranges, return
        if not valid_ranges1 or not valid_ranges2 or not valid_ranges3:
            self.get_logger().warn("No valid lidar data available.")
            return

        # Find the minimum valid range in each section
        min_range1 = min(valid_ranges1)
        min_range2 = min(valid_ranges2)
        min_range3 = min(valid_ranges3)

        # Debugging print statements to see the min range values
        self.get_logger().info(f"Min Range 1: {min_range1}, Min Range 2: {min_range2}, Min Range 3: {min_range3}")

        # Apply fuzzy logic to the lidar distances
        self.speed_sim.input['obstacle_distance1'] = min_range1
        self.speed_sim.input['obstacle_distance2'] = min_range2
        self.speed_sim.input['obstacle_distance3'] = min_range3
        self.steering_sim.input['obstacle_distance1'] = min_range1
        self.steering_sim.input['obstacle_distance2'] = min_range2
        self.steering_sim.input['obstacle_distance3'] = min_range3

        # Compute fuzzy logic
        self.speed_sim.compute()
        self.steering_sim.compute()

        # Get fuzzy outputs for speed and steering
        adjusted_speed = self.speed_sim.output['speed']
        adjusted_steering = self.steering_sim.output['steering']

        # Debugging print statements for fuzzy outputs
        self.get_logger().info(f"Adjusted Speed: {adjusted_speed}, Adjusted Steering: {adjusted_steering}")

        # Apply joystick input (linear.x and angular.z) and fuzzy adjustments
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.cmd_vel_joy_msg.linear.x + adjusted_speed  # Joystick input + adjustment from fuzzy
        cmd_vel_msg.angular.z = self.cmd_vel_joy_msg.angular.z + adjusted_steering  # Joystick input + adjustment from fuzzy

        # Publish the adjusted command
        self.publisher.publish(cmd_vel_msg)

    def joy_callback(self, msg):
        self.cmd_vel_joy_msg = msg

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
