import py_trees
from rclpy.node import Node
from core.utils.config import BT

class BaseBehavior(py_trees.behaviour.Behaviour):
    """Base class for all behaviors that need ROS node access and blackboard functionality"""
    
    def __init__(self, name: str, keys_to_register=None):
        super(BaseBehavior, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key("ros_node", access=BT.read)
        
        if keys_to_register:
            for key, access in keys_to_register.items():
                self.blackboard.register_key(key, access=access)
                
    @property
    def node(self) -> Node:
        """Access to the ROS node"""
        return self.blackboard.ros_node