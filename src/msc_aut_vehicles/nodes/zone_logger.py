#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray
import csv
from datetime import datetime

class ZoneLogger(Node):
    def __init__(self):
        super().__init__('zone_logger')

        self.zone_id = None
        self.min_ranges = None  # Initialize as None to differentiate from empty or incomplete

        # Open CSV file and write header
        self.csv_file = open('zone_log.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'zone_id', 'min_range1', 'min_range2', 'min_range3'])

        self.create_subscription(Int32, '/zone_id', self.zone_callback, 10)
        self.create_subscription(Float32MultiArray, '/min_ranges', self.ranges_callback, 10)

    def zone_callback(self, msg):
        self.zone_id = msg.data
        self.try_log()

    def ranges_callback(self, msg):
        self.min_ranges = list(msg.data)  # Safely convert to list
        self.try_log()

    def try_log(self):
        if self.zone_id is not None and self.min_ranges is not None and len(self.min_ranges) == 3:
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]  # Time only, no date
            row = [timestamp, self.zone_id] + self.min_ranges
            self.csv_writer.writerow(row)
            self.csv_file.flush()
            #self.get_logger().info(f"Logged: {row}")
            self.zone_id = None
            self.min_ranges = None

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZoneLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
