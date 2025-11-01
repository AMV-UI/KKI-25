import py_trees
from rclpy.node import Node
from py_trees.behaviour import Behaviour


class BaseBehavior(Behaviour):
    """Base class for all behaviors that need ROS node access (no blackboard)"""
    
    def __init__(self, name: str, node: Node = None):
        super(BaseBehavior, self).__init__(name)
        # store the ROS node directly instead of using the py_trees blackboard
        self.node: Node = node
                
    # keep property for compatibility
    @property
    def node(self) -> Node:
        return self._node

    @node.setter
    def node(self, value: Node) -> None:
        self._node = value
