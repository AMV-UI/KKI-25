import py_trees
from rclpy.node import Node
from py_trees.behaviour import Behaviour


class BaseBehavior(Behaviour):
    """Base class for all behaviors that need ROS node access (no blackboard)"""
    
    def __init__(self, name: str, node: Node = None):
        super(BaseBehavior, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key("ros_node", access=py_trees.common.Access.WRITE)
    
    @classmethod
    def set_ros_node(cls, node: Node):
        """Set the ROS node on the blackboard for all behaviors to access"""
        blackboard = py_trees.blackboard.Client(name="global")
        blackboard.register_key("ros_node", access=py_trees.common.Access.WRITE)
        blackboard.ros_node = node
                
    # keep property for compatibility
    @property
    def node(self) -> Node:
        return self._node

    @node.setter
    def node(self, value: Node) -> None:
        self._node = value
