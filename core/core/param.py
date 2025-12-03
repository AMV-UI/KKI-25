#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from core.utils.config import Topic

class ParameterBlackboard(Node):    
    def __init__(self, node = Node):
        super().__init__(
            'parameter_blackboard',
        )

        self.node = node

        self.arena = "B"
        self.st_speed = 1.0
        self.tn_speed = 1.0
        self.dock_lat = 0.0
        self.dock_lon = 0.0

        self.setup()

    def setup(self):
        self.arena_pub = Topic.arena.createPublisher(self.node).publish(String(data=self.arena))
        self.st_speed_pub = Topic.st_speed.createPublisher(self.node).publish(Float64(data=self.st_speed))
        self.tn_speed_pub = Topic.tn_speed.createPublisher(self.node).publish(Float64(data=self.tn_speed))
        self.dock_lat_pub = Topic.dock_lat.createPublisher(self.node).publish(Float64(data=self.dock_lat))
        self.dock_lon_pub = Topic.dock_lon.createPublisher(self.node).publish(Float64(data=self.dock_lon))

        self.arena_sub = Topic.arena.createSubscriber(
            self,
            self._arena_cb
        )
        self.st_speed_sub = Topic.st_speed.createSubscriber(
            self,
            self._st_speed_cb
        )
        self.tn_speed_sub = Topic.tn_speed.createSubscriber(
            self,
            self._tn_speed_cb
        )
        self.dock_lat_sub = Topic.dock_lat.createSubscriber(
            self,
            self._dock_lat_cb
        )
        self.dock_lon_sub = Topic.dock_lon.createSubscriber(
            self,
            self._dock_lon_cb
        )

    def _arena_cb(self, msg: String):
        self.arena = msg.data
    
    def _st_speed_cb(self, msg: Float64):
        self.st_speed = msg.data
    
    def _tn_speed_cb(self, msg: Float64):
        self.tn_speed = msg.data
    
    def _dock_lat_cb(self, msg: Float64):
        self.dock_lat = msg.data

    def _dock_lon_cb(self, msg: Float64):
        self.dock_lon = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = ParameterBlackboard(rclpy.create_node('param'))
    
    try:
        node.get_logger().info('Spinning... Param')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Parameter Blackboard')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()