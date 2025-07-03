#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv
from datetime import datetime


class VelocityLogger(Node):
    def __init__(self):
        super().__init__('velocity_logger')

        self.create_subscription(Twist, '/diff_cont/cmd_vel_unstamped', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_joy', self.cmd_vel_joy_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_stop', self.cmd_vel_stop_callback, 10)

        self.csv_file = open("velocity_comparison.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "Log_Time", "Topic", "Msg_Timestamp", "Linear_X", "Angular_Z"
        ])
        self.get_logger().info("Velocity Logger initialized")

    def get_now_str(self):
        return datetime.now().strftime('%H:%M:%S.%f')[:-3]

    def write_row(self, topic_name: str, linear_x: float, angular_z: float):
        row = [
            self.get_now_str(),
            topic_name,
            self.get_now_str(),  # Could use ROS time here if needed
            linear_x,
            angular_z
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def cmd_vel_callback(self, msg):
        self.write_row("cmd_vel", msg.linear.x, msg.angular.z)

    def odom_callback(self, msg):
        self.write_row("odom", msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def cmd_vel_joy_callback(self, msg):
        self.write_row("joy", msg.linear.x, msg.angular.z)

    def cmd_vel_stop_callback(self, msg):
        self.write_row("stop", msg.linear.x, msg.angular.z)

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VelocityLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
