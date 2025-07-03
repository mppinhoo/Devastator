#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JoyFeedbackArray, JoyFeedback, Joy
from std_msgs.msg import Int32, Float32MultiArray  # Added for zone and range publishing

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.subscription_joy = self.create_subscription(
            Twist,
            '/cmd_vel_joy',
            self.joy_callback,
            10
        )

        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.joy_input_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_input_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel_stop', 10)
        self.vibration_publisher = self.create_publisher(JoyFeedbackArray, '/joy/set_feedback', 10)
        self.zone_publisher = self.create_publisher(Int32, '/zone_id', 10)  # Zone publisher
        self.min_ranges_publisher = self.create_publisher(Float32MultiArray, '/min_ranges', 10)  # Min ranges publisher

        self.cmd_vel_joy_msg = Twist()
        self.cmd_vel_msg = Twist()
        self.use_cmd_vel_joy = False

        self.StopDist = 0.18
        self.Ddist = 0.35
        self.Sdist = 0.8

    def publish_zone(self, zone_id):
        msg = Int32()
        msg.data = zone_id
        self.zone_publisher.publish(msg)

    def publish_min_ranges(self, r1, r2, r3):
        msg = Float32MultiArray()
        msg.data = [r1, r2, r3]
        self.min_ranges_publisher.publish(msg)

    def lidar_callback(self, msg):
        all_ranges = msg.ranges
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

        min_range1 = float('inf')
        min_index1 = -1
        min_range2 = float('inf')
        min_index2 = -1
        min_range3 = float('inf')
        min_index3 = -1

        for i, value in enumerate(ranges1):
            if not math.isnan(value) and 0.05 < value < min_range1:
                min_range1 = value
                min_index1 = i

        for i, value in enumerate(ranges2):
            if not math.isnan(value) and 0.05 < value < min_range2:
                min_range2 = value
                min_index2 = i

        for i, value in enumerate(ranges3):
            if not math.isnan(value) and 0.05 < value < min_range3:
                min_range3 = value
                min_index3 = i

        if min_index1 == -1:
            self.get_logger().warn("No valid range data received.")
            self.publish_zone(0)
            self.publish_min_ranges(0.0, 0.0, 0.0)
            return

        # Publish current minimum ranges
        self.publish_min_ranges(min_range1, min_range2, min_range3)

        angle_increment = msg.angle_increment
        min_angle1 = min_index1 * angle_increment - math.pi/3
        min_angle2 = min_index2 * angle_increment - math.pi/4
        min_angle3 = min_index3 * angle_increment - math.pi/6

        cmd_vel_msg = Twist()

        flag = 0
        if self.use_cmd_vel_joy:
            current_cmd_vel_msg = self.cmd_vel_joy_msg
            flag = 1
        else:
            current_cmd_vel_msg = self.cmd_vel_msg

        variable = 0

        if min_range1 < self.StopDist:
            self.publish_zone(1)

            if (current_cmd_vel_msg.linear.x > 0.02 or current_cmd_vel_msg.angular.z > 0.02) and -math.pi/3 < min_angle1 < math.pi/3:
                
                cmd_vel_msg.linear.x = 0.0

                # if min_angle1 > 0:
                #     cmd_vel_msg.angular.z = min((-math.pi + min_angle3) * 0.5, current_cmd_vel_msg.angular.z)
                # elif min_angle1 < 0:
                #     cmd_vel_msg.angular.z = max((math.pi + min_angle3) * 0.5, current_cmd_vel_msg.angular.z)
                # else:
                #     cmd_vel_msg.angular.z = current_cmd_vel_msg.angular.z

                cmd_vel_msg.angular.z = current_cmd_vel_msg.angular.z * 0.2

                self.publisher.publish(cmd_vel_msg)

            variable = 1

        elif self.StopDist < min_range2 < self.Ddist and variable != 1:
            self.publish_zone(2)

            if current_cmd_vel_msg.linear.x > 0.05 and -math.pi/4 < min_angle2 < math.pi/4:
                cmd_vel_msg.linear.x = current_cmd_vel_msg.linear.x * min_range2

                if min_angle2 > 0:
                    cmd_vel_msg.angular.z = min((-math.pi/2 + min_angle3) * 1.5, current_cmd_vel_msg.angular.z)
                elif min_angle2 < 0:
                    cmd_vel_msg.angular.z = max((math.pi/2 + min_angle3) * 1.5, current_cmd_vel_msg.angular.z)
                else:
                    cmd_vel_msg.angular.z = current_cmd_vel_msg.angular.z

                self.publisher.publish(cmd_vel_msg)

            variable = 2

        elif self.Ddist < min_range3 < self.Sdist and variable == 0 and flag == 1:
            self.publish_zone(3)

            if current_cmd_vel_msg.linear.x > 0.05 and -math.pi/6 < min_angle3 < math.pi/6:
                cmd_vel_msg.linear.x = current_cmd_vel_msg.linear.x * min_range3

                if min_angle3 > 0:
                    cmd_vel_msg.angular.z = min((-math.pi/4 + min_angle3) * 1.5, current_cmd_vel_msg.angular.z)
                elif min_angle3 < 0:
                    cmd_vel_msg.angular.z = max((math.pi/4 + min_angle3) * 1.5, current_cmd_vel_msg.angular.z)
                else:
                    cmd_vel_msg.angular.z = current_cmd_vel_msg.angular.z

                self.publisher.publish(cmd_vel_msg)
        else:
            self.publish_zone(0)

    def joy_input_callback(self, msg):
        self.l1_button_pressed = msg.buttons[9] == 1
        self.use_cmd_vel_joy = self.l1_button_pressed

    def joy_callback(self, msg):
        self.cmd_vel_joy_msg = msg

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg

    def publish_vibration(self, intensity):
        vibration_msg = JoyFeedbackArray()
        vibration_msg.array.append(JoyFeedback(type=1, id=0, intensity=intensity))
        self.vibration_publisher.publish(vibration_msg)

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
