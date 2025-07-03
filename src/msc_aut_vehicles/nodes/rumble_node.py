import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JoyFeedback

class RumbleController(Node):
    def __init__(self):
        super().__init__('rumble_controller')
        self.publisher = self.create_publisher(JoyFeedback, 'joy/set_feedback', 10)

    def rumble(self, intensity):
        message = JoyFeedback()
        message.type = JoyFeedback.TYPE_RUMBLE
        message.id = 0  # You can adjust the ID if needed
        message.intensity = intensity  # Adjust intensity (0.0 to 1.0)
        self.publisher.publish(message)
        self.get_logger().info('Rumble command published')

def main(args=None):
    rclpy.init(args=args)
    rumble_controller = RumbleController()
    # Adjust the intensity value as needed (0.0 to 1.0)
    rumble_controller.rumble(1.0)  # Example: set intensity to maximum
    rclpy.spin(rumble_controller)
    rumble_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
