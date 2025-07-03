#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from control_msgs.msg import DynamicJointState  # Updated to import DynamicJointState
import csv
import datetime

class WheelVelocityLogger(Node):
    def __init__(self):
        super().__init__('wheel_velocity_logger')

        # Declare and get parameters
        self.declare_parameter('wheel_separation', 0.193)
        self.declare_parameter('wheel_radius', 0.021)
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # Subscribe to topics
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_joy', self.cmd_vel_callback, 10)
        # Subscribe to /dynamic_joint_states topic with DynamicJointState type
        self.joint_state_sub = self.create_subscription(DynamicJointState, '/dynamic_joint_states', self.joint_states_callback, 10)

        # Open CSV file and write header
        self.csv_file = open("wheel_velocities.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["Time (HH:MM:SS.mmm)", "Desired Left", "Desired Right", "Measured Left", "Measured Right"])

        # Store latest actual velocities
        self.actual_left = 0.0
        self.actual_right = 0.0

        self.get_logger().info("Wheel Velocity Logger Node has started")

    def cmd_vel_callback(self, msg):
        # Compute expected wheel velocities
        v_x = msg.linear.x
        omega_z = msg.angular.z
        left_wheel_velocity = (v_x - (omega_z * self.wheel_separation / 2.0)) / self.wheel_radius
        right_wheel_velocity = (v_x + (omega_z * self.wheel_separation / 2.0)) / self.wheel_radius

        # Get current time with milliseconds
        timestamp = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]  # Trim to milliseconds

        # Log to CSV
        self.csv_writer.writerow([
            timestamp,
            left_wheel_velocity, right_wheel_velocity,
            self.actual_left, self.actual_right
        ])
        self.csv_file.flush()  # Ensure data is written immediately

    def joint_states_callback(self, msg):
        # Extract actual wheel velocities from the DynamicJointState message
        for i, joint_name in enumerate(msg.joint_names):
            if joint_name == "left_wheel_joint":
                left_velocity_index = msg.interface_values[i].interface_names.index("velocity")
                self.actual_left = msg.interface_values[i].values[left_velocity_index]
            
            if joint_name == "right_wheel_joint":
                right_velocity_index = msg.interface_values[i].interface_names.index("velocity")
                self.actual_right = msg.interface_values[i].values[right_velocity_index]

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WheelVelocityLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
