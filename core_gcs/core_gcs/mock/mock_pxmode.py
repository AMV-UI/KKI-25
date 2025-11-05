import rclpy
from rclpy.node import Node
from core_msgs.msg import Pixhawk
from core.utils.config import Topic, PxMode
from std_msgs.msg import String
import random
import math

class MockPxModePublisher(Node):
    def __init__(self):
        super().__init__('mock_pxmode_publisher')

        self.pxmode_publisher = Topic.pxmode.createPublisher(
            self
        )
        
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_mock_data)

    def publish_mock_data(self):
        node_msg = String()
        node_msg.data = PxMode.AUTO
        self.pxmode_publisher.publish(node_msg)
        self.get_logger().info(
            f"Publishing PXMODE!!! "
        )
def main(args=None):
    rclpy.init(args=args)
    node = MockPxModePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
