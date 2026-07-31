#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from core.utils.config import Topic, MissionStatus
from time import sleep

class ParameterBlackboard(Node):    
    def __init__(self):
        # Initialize this class as an rclpy Node
        super().__init__('parameter_blackboard')

        # Use self as the node for publishers/subscribers
        self.arena = "A"
        self.st_speed = 1.0
        self.tn_speed = 1.0
        self.dock_lat = 0.0
        self.dock_lon = 0.0
        self.mission_type = MissionStatus.BUOY

        self.setup()

    def setup(self):
        # Important: delete === gay
        # sleep(5.0)# Wait for other nodes to be ready
        # Create publishers and retain references so they aren't garbage collected
        self.arena_pub = Topic.arena.createPublisher(self)
        self.st_speed_pub = Topic.st_speed.createPublisher(self)
        self.tn_speed_pub = Topic.tn_speed.createPublisher(self)
        self.mission_type_pub = Topic.mission_type.createPublisher(self)

        # Now publish
        self.arena_pub.publish(String(data=self.arena))
        self.st_speed_pub.publish(Float64(data=self.st_speed))
        self.tn_speed_pub.publish(Float64(data=self.tn_speed))
        self.mission_type_pub.publish(String(data=self.mission_type))

        self.get_logger().info(
            f'<> [ParameterBlackboard] Published initial parameters '
            f'{self.arena}, {self.st_speed}, {self.tn_speed}, '
            f'{self.dock_lat}, {self.dock_lon}, {self.mission_type}'
        )

        self.arena_sub = Topic.arena.createSubscriber(self, self._arena_cb)
        self.st_speed_sub = Topic.st_speed.createSubscriber(self, self._st_speed_cb)
        self.tn_speed_sub = Topic.tn_speed.createSubscriber(self, self._tn_speed_cb)
        self.mission_type_sub = Topic.mission_type.createSubscriber(self, self._mission_type_cb)
    
    def _mission_type_cb(self, msg: String):
        self.mission_type = msg.data

    def _arena_cb(self, msg: String):
        self.arena = msg.data
    
    def _st_speed_cb(self, msg: Float64):
        self.st_speed = msg.data
    
    def _tn_speed_cb(self, msg: Float64):
        self.tn_speed = msg.data
    
def main(args=None):
    rclpy.init(args=args)
    node = ParameterBlackboard()
    
    try:
        node.get_logger().info(f'Spinning... Param')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Parameter Blackboard')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
