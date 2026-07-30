import py_trees
from tree_builders import sequence_builder


class MissionTreeBuilder:
    """Builder class responsible for constructing the behavior tree"""

    def __init__(self, ros_node):
        self.ros_node = ros_node

    def build(self) -> py_trees.trees.BehaviourTree:
        root = sequence_builder.build(
            [
                # BehaviorA("namehere"),
                # BehaviorB("namethere"),
            ]
        )
        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(node=self.ros_node)
        return tree

    def get_tree_structure(self) -> str:
        """Return a string representation of the tree structure for debugging"""
        root = self._create_root()
        return py_trees.console.ascii_tree(root)

    def get_tree_ascii(self) -> str:
        """Return a DOT representation of the tree for visualization"""
        root = self._create_root()
        return py_trees.display.render_dot_tree(root)
