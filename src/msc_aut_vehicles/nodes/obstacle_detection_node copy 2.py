#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sensor_msgs.msg import JoyFeedbackArray, JoyFeedback, Joy

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Replace '/lidar_topic' with the actual lidar topic
            self.lidar_callback,
            10
        )
        self.subscription_joy = self.create_subscription(
            Twist,
            '/cmd_vel_joy',  # Replace '/cmd_vel_joy' with the actual joystick cmd_vel topic
            self.joy_callback,
            10
        )
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',  # Add subscription for /cmd_vel topic
            self.cmd_vel_callback,
            10
        )

        self.joy_input_subscription = self.create_subscription(
            Joy,
            '/joy',  # Replace with the actual topic name for gamepad input
            self.joy_input_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel_stop', 10)
        self.vibration_publisher = self.create_publisher(JoyFeedbackArray, '/joy/set_feedback', 10)

        self.cmd_vel_joy_msg = Twist()
        self.cmd_vel_msg = Twist()
        self.use_cmd_vel_joy = False  # Flag to track which cmd_vel to use

         # Constants
        self.StopDist = 0.18
        self.Ddist = 0.35 # Danger distance
        self.Sdist = 0.8 # Safety distance
    
    def lidar_callback(self, msg):

        all_ranges = msg.ranges

        num_ranges = len(all_ranges)

        # Calculate indices for 3/8 and 5/8 of the ranges
        start_index1 = int(60 * num_ranges / 240)
        end_index1 = int(180 * num_ranges / 240)
        start_index2 = int(75 * num_ranges / 240)
        end_index2 = int(165 * num_ranges / 240)
        start_index3 = int(90 * num_ranges / 240)
        end_index3 = int(150 * num_ranges / 240)

        # Extract the relevant section of the ranges
        ranges1 = all_ranges[start_index1:end_index1]
        ranges2 = all_ranges[start_index2:end_index2]
        ranges3 = all_ranges[start_index3:end_index3]


        # Initialize min_range to infinity and min_index to -1
        min_range1 = float('inf')
        min_index1 = -1
        min_range2 = float('inf')
        min_index2 = -1
        min_range3 = float('inf')
        min_index3 = -1

        # Iterate through the ranges to find the minimum non-NaN range
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

        # Ensure a valid min_index is found
        if min_index1 == -1:
            self.get_logger().warn("No valid range data received.")
            return

        # min_range = min(msg.ranges)
        #min_index = msg.ranges.index(min_range)

        angle_increment = msg.angle_increment
        min_angle1 = min_index1 * angle_increment - math.pi/3
        min_angle2 = min_index2 * angle_increment - math.pi/4
        min_angle3 = min_index3 * angle_increment - math.pi/6

        cmd_vel_msg = Twist()

        # print("min_range1 = ", min_range1, flush=True)
        # print("min_angle1 = ", min_angle1, flush=True)
        # print("--------------------------")
        # print("min_range2 = ", min_range2, flush=True)
        # print("min_angle2 = ", min_angle2, flush=True)
        # print("--------------------------")
        # print("min_range3 = ", min_range3, flush=True)
        # print("min_angle3 = ", min_angle3, flush=True)
        #print("--------------------------")
        #print(self.use_cmd_vel_joy, flush=True)
        

        flag = 0
        # Choose the appropriate cmd_vel message based on priority
        if self.use_cmd_vel_joy:
            current_cmd_vel_msg = self.cmd_vel_joy_msg
            flag = 1
        else:
            current_cmd_vel_msg = self.cmd_vel_msg

        variable = 0

        if min_range1 < self.StopDist:
            if current_cmd_vel_msg.linear.x > 0.05 and -math.pi/3 < min_angle1 < math.pi/3:
            
                cmd_vel_msg.linear.x = current_cmd_vel_msg.linear.x * min_range1 * 0.5
                
                if min_angle1 > 0:
                    cmd_vel_msg.angular.z = min((-math.pi + min_angle3) * 0.5, current_cmd_vel_msg.angular.z)
                elif min_angle1 < 0:
                    cmd_vel_msg.angular.z = max((math.pi + min_angle3) * 0.5, current_cmd_vel_msg.angular.z)
                else:
                    cmd_vel_msg.angular.z = current_cmd_vel_msg.angular.z

                self.publisher.publish(cmd_vel_msg)

            else:
                # No obstacle within the specified minimum range, do not publish cmd_vel
                pass

            variable = 1

        elif self.StopDist < min_range2 < self.Ddist and variable !=1:
            if current_cmd_vel_msg.linear.x > 0.05 and -math.pi/4 < min_angle2 < math.pi/4:

                cmd_vel_msg.linear.x = current_cmd_vel_msg.linear.x * min_range2 
                
                if min_angle2 > 0:
                    cmd_vel_msg.angular.z = min((-math.pi/2 + min_angle3) * 1.0, current_cmd_vel_msg.angular.z)
                elif min_angle2 < 0:
                    cmd_vel_msg.angular.z = max((math.pi/2 + min_angle3) * 1.0, current_cmd_vel_msg.angular.z)
                else:
                    cmd_vel_msg.angular.z = current_cmd_vel_msg.angular.z

                self.publisher.publish(cmd_vel_msg)

            else:
                # No obstacle within the specified minimum range, do not publish cmd_vel
                pass

            variable = 2

        elif self.Ddist < min_range3 < self.Sdist and variable == 0 and flag == 1:
            if current_cmd_vel_msg.linear.x > 0.05 and -math.pi/6 < min_angle3 < math.pi/6:

                cmd_vel_msg.linear.x = current_cmd_vel_msg.linear.x * min_range3

                if min_angle3 > 0:
                    cmd_vel_msg.angular.z = min((-math.pi/4 + min_angle3) * 0.5, current_cmd_vel_msg.angular.z)
                elif min_angle3 < 0:
                    cmd_vel_msg.angular.z = max((math.pi/4 + min_angle3) * 0.5, current_cmd_vel_msg.angular.z)
                else:
                    cmd_vel_msg.angular.z = current_cmd_vel_msg.angular.z

                self.publisher.publish(cmd_vel_msg)

            else:
                # No obstacle within the specified minimum range, do not publish cmd_vel
                pass

        else:
            pass

        variable = 0
        flag = 0
        
    def joy_input_callback(self, msg):
        """Handle incoming gamepad input and set the L1 button state."""
        self.l1_button_pressed = msg.buttons[9] == 1  # Assuming L1 button is at index 4
        self.use_cmd_vel_joy = self.l1_button_pressed


    def joy_callback(self, msg):
        self.cmd_vel_joy_msg = msg
        #self.use_cmd_vel_joy = True  # Set the flag to use cmd_vel_joy

    def cmd_vel_callback(self, msg):
        #if not self.use_cmd_vel_joy:  # Only update if not using cmd_vel_joy
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
