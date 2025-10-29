import py_trees
from rclpy.node import Node
from py_trees.behaviour import Behaviour


class BaseBehavior(Behaviour):
    """Base class for all behaviors that need ROS node access"""
    
    def __init__(self, name: str):
        super(BaseBehavior, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key("ros_node", access=py_trees.common.Access.READ)
                
    @property
    def node(self) -> Node:
        """Access to the ROS node"""
        return self.blackboard.ros_node
