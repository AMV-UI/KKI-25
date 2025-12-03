# talker.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from core.utils.config import AutoState, Box, Camera, NodeConfig, Topic

class MockPublish(Node):
    def __init__(self):
        super().__init__('mock_mission')
        self.publisher = Topic.camera_processed.createPublisher(self)
        self.counter = 1
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = str(self.counter)
        self.publisher.publish(msg)
        self.counter = self.counter + 1
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MockPublish()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

