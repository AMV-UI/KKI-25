import rclpy
from rclpy.node import Node
from core_msgs.msg import Pixhawk
from core.utils.config import Topic, PxMode
from std_msgs.msg import String
import random
import math

class MockPixhawkPublisher(Node):
    def __init__(self):
        super().__init__('mock_pixhawk_publisher')

        self.publisher = Topic.pixhawk.createPublisher(self)

        # Timer publishes every 1 second
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_mock_data)

        # Base southern-most latitude and eastern-most longitude
        self.base_lat = -6.2088  # southern-most
        self.base_lon = 106.8456  # eastern-most

        # Convert 25 meters to degrees
        self.delta_lat = 25 / 111320  # ~0.000224 degrees
        self.delta_lon = 25 / (111320 * math.cos(math.radians(self.base_lat)))  # ~0.000284 degrees

        self.pxmode_publisher = Topic.pxmode.createPublisher(self)
        self.node_msg = String()

        self.counter = 0

    def publish_mock_data(self):
        msg = Pixhawk()

        if self.counter == 0:
            self.node_msg.data = PxMode.HOLD
            self.pxmode_publisher.publish(self.node_msg)
            self.counter = self.counter + 1
            return  
        elif self.counter <= 2:
            # First message is exactly base point
            self.node_msg.data = PxMode.AUTO
            self.pxmode_publisher.publish(self.node_msg)
            msg.lat = self.base_lat
            msg.lon = self.base_lon
        else:
            # Randomize within 25m x 25m square
            msg.lat = self.base_lat + random.uniform(0, self.delta_lat)
            msg.lon = self.base_lon - random.uniform(0, self.delta_lon)  # subtract to go west

        msg.alt = 5.0 + random.uniform(-0.2, 0.2)
        msg.msg_spd = random.uniform(0.5, 3.0)
        msg.msg_heading = int(random.uniform(0, 360))

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
