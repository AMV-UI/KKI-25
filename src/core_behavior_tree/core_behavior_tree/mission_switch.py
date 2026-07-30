#!/usr/bin/env python3

import time
import threading
import rclpy
import rclpy.executors
from rclpy.node import Node
import py_trees
import py_trees.display

from .tree.mission_tree_builder import MissionTreeBuilder


class BehaviorTreeNode(Node):
    """Main ROS2 node that runs the behavior tree"""

    def __init__(self):
        super().__init__("behavior_tree_node")

        tree_builder = MissionTreeBuilder(self)
        self.tree = tree_builder.build()

        self.tree.setup()
        self.tick_period = 0.1  # 10 Hz for prod / sim
        # self.tick_period = 2 #2hz for dev
        self.timer = self.create_timer(self.tick_period, self.tick_tree)

        self.setup_visualization()

        self.get_logger().info("Behavior tree initialized and ready")

    def tick_tree(self):
        """Called periodically to tick the behavior tree"""
        self.tree.tick()

    def setup_visualization(self):
        """Setup visualization of the behavior tree"""
        py_trees.display.render_dot_tree(self.tree.root)
        self.get_logger().info("Behavior tree visualization generated")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    bt_node = BehaviorTreeNode()

    # MultiThreadedExecutor for ascii tree rendering
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(bt_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():
            # bt_node.tick_tree()
            # print(py_trees.display.ascii_tree(bt_node.tree.root, show_status=True))
            time.sleep(0.1)
            # os.system('clear')
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()


if __name__ == "__main__":
    main()
