import rclpy
from rclpy.node import Node
from core_msgs.msg import Pixhawk  # ✅ make sure the msg package name matches your actual one
from core.utils.config import Topic
import random
import math
import time


class MockPixhawkPublisher(Node):
    def __init__(self):
        super().__init__('mock_pixhawk_publisher')

        # Create publisher using your Topic config helper
        self.publisher = Topic.pixhawk.createPublisher(self)

        # Timer publishes every 1 second
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_mock_data)

        # Simulate a small area near some coordinates
        self.base_lat = -6.2088  # Jakarta example
        self.base_lon = 106.8456
        self.counter = 0

    def publish_mock_data(self):
        msg = Pixhawk()

        # Create mock coordinates that drift slowly
        msg.lat = self.base_lat + math.sin(self.counter / 20.0) * 0.0005
        msg.lon = self.base_lon + math.cos(self.counter / 20.0) * 0.0005
        msg.alt = 5.0 + random.uniform(-0.2, 0.2)  # altitude variation
        msg.msg_spd = random.uniform(0.5, 3.0)     # simulated speed in m/s
        msg.msg_heading = int((self.counter * 10) % 360)  # heading in degrees

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Publishing Pixhawk -> "
            f"lat: {msg.lat:.6f}, lon: {msg.lon:.6f}, alt: {msg.alt:.2f}, "
            f"spd: {msg.msg_spd:.2f}, heading: {msg.msg_heading}"
        )

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockPixhawkPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
