#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float64, Bool
import py_trees
import py_trees.console as console
import py_trees.display
import time
import random
from enum import Enum
import threading    
from utils.config import Node as NodeConfig, PxMode, Direction, Param, Topic, BT


# Simplified enums and constants
class PxMode(Enum):
    HOLD = 0
    MANUAL = 1

class Topic:
    heading_deg = "heading_deg"
    detected = "detected"
    mission = "mission"
    dsc = "dsc"
    pxmode = "pxmode"
    state_dst = "state_dst"

class Param:
    TRACK = "track"

class NodeConfig:
    mission = "mission_node"

# Simplified frame counter
class FrameCounter:
    def __init__(self, threshold):
        self.count = 0
        self.threshold = threshold
        
    def is_started(self):
        self.count += 1
        return True
        
    def is_enough(self):
        return self.count >= self.threshold
        
    def reset(self):
        self.count = 0

# FindMode class from original code (simplified)
class FindMode:
    def __init__(self):
        self.initial_heading = -1
        self.current_heading = -1
        self.threshold = 90
        self.range_low = -1
        self.range_high = -1
        self.find_status = "Left"  # Default
        self.GO_LEFT = -200
        self.GO_RIGHT = 200
        self.dir_map = {
            "Right": self.GO_RIGHT,
            "Left": self.GO_LEFT,
        }
        
        # Simplified track parameter
        self._track = "A"

    def set_track(self, track):
        self._track = track
        self.find_status = "Left" if track == "A" else "Right"

    def get_track(self):
        return self._track

    def set_initial_heading(self, heading):
        self.initial_heading = heading

    def set_range(self, direction):
        # Simplified ranges
        if self.get_track() == "A":
            self.range_low = 50
            self.range_high = 230
        else:
            self.range_low = 130
            self.range_high = 310

    def get_heading(self, raw_heading):
        heading = raw_heading - self.initial_heading
        if heading < 0:
            heading = 360 + heading
        return heading

    def get_state(self, raw_heading):
        self.current_heading = self.get_heading(raw_heading)
        
        # Simplified direction logic
        in_range = (self.range_low < self.current_heading < self.range_high)
        if not in_range:
            if abs(self.current_heading - self.range_low) < abs(self.current_heading - self.range_high):
                self.find_status = "Right"
            else:
                self.find_status = "Left"
                
        return self.dir_map[self.find_status]

# Mini simulator for sensors
class MiniSimulator(Node):
    def __init__(self):
        super().__init__('mini_simulator')
        
        # Publishers to simulate sensors
        self.heading_pub = self.create_publisher(Float64, Topic.heading_deg, 10)
        self.detected_pub = self.create_publisher(Bool, Topic.detected, 10)
        self.pxmode_pub = self.create_publisher(UInt8, Topic.pxmode, 10)
        
        # Simulation variables
        self.heading = 0.0
        self.detection_probability = 0.2
        self.detected = False
        self.detection_toggle_timer = 0
        
        # Start simulation timer
        self.timer = self.create_timer(0.1, self.simulate)
        
        # Initial PX mode
        msg = UInt8()
        msg.data = PxMode.HOLD.value
        self.pxmode_pub.publish(msg)
        
        # Parameters
        self.declare_parameter(Param.TRACK, "A")
        
    def simulate(self):
        # Simulate heading changes
        self.heading = (self.heading + random.uniform(-5, 5)) % 360.0
        heading_msg = Float64()
        heading_msg.data = self.heading
        self.heading_pub.publish(heading_msg)
        
        # Simulate detection with random toggling
        self.detection_toggle_timer += 1
        if self.detection_toggle_timer >= 20:
            if random.random() < self.detection_probability:
                self.detected = not self.detected
                detected_msg = Bool()
                detected_msg.data = self.detected
                self.detected_pub.publish(detected_msg)
                self.get_logger().info(f"Detection status changed to: {self.detected}")
            self.detection_toggle_timer = 0
        
        # Always publish current detection state
        detected_msg = Bool()
        detected_msg.data = self.detected
        self.detected_pub.publish(detected_msg)

# Initialize Blackboard behavior
class InitializeBlackboard(py_trees.behaviour.Behaviour):
    def __init__(self):
        super(InitializeBlackboard, self).__init__(name="Initialize Blackboard")
        self.blackboard = py_trees.blackboard.Client(name="Mission")
        self.blackboard.register_key("px_heading", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("detected", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("dsc", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("pxmode", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("mission_status", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("frame_counter", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("frame_counter_manuver", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("manuver_detected", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("isChange", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("find_mode", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("buoy_visited_count", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("last_detected_time", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("maneuvering_complete", access=py_trees.common.Access.WRITE)
        
    def update(self):
        self.blackboard.px_heading = -1
        self.blackboard.detected = False
        self.blackboard.dsc = float(160)
        self.blackboard.pxmode = PxMode.HOLD.value
        self.blackboard.mission_status = "RUNNING"
        self.blackboard.frame_counter = FrameCounter(2)
        self.blackboard.frame_counter_manuver = FrameCounter(3)
        self.blackboard.manuver_detected = False
        self.blackboard.isChange = False
        self.blackboard.find_mode = FindMode()
        self.blackboard.buoy_visited_count = 0
        self.blackboard.last_detected_time = time.time()
        self.blackboard.maneuvering_complete = False
        return py_trees.common.Status.SUCCESS

class PrintBlackboard(py_trees.behaviour.Behaviour):
    def __init__(self):
        super(PrintBlackboard, self).__init__(name="Print Blackboard")
        self.blackboard = py_trees.blackboard.Client(name="BlackboardMonitor")
        
        self.blackboard.register_key("px_heading", access=py_trees.common.Access.READ)
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("pxmode", access=py_trees.common.Access.READ)
        
        
    def update(self):
        # Use py_trees console instead of self.logger
        # print(console.green + "Blackboard contents:" + console.reset)
        # print(f"  px_heading: {self.blackboard.px_heading}")
        # print(f"  detected: {self.blackboard.detected}")
        # print(f"  pxmode: {self.blackboard.pxmode}")
        # # Print other keys as needed
        # return py_trees.common.Status.RUNNING

        blackboard = py_trees.blackboard.Blackboard()
        print(blackboard)


# Sensor subscribers
class HeadingSubscriber(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super(HeadingSubscriber, self).__init__(name="Heading Subscriber")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="HeadingSubscriber")
        self.blackboard.register_key("px_heading", access=py_trees.common.Access.WRITE)
        
    def setup(self, **kwargs):
        self.subscription = self.node.create_subscription(
            Float64,
            Topic.heading_deg,
            self.heading_callback,
            10
        )
        return True
        
    def heading_callback(self, msg):
        self.blackboard.px_heading = msg.data
        
    def update(self):
        return py_trees.common.Status.RUNNING

class DetectedSubscriber(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super(DetectedSubscriber, self).__init__(name="Detected Subscriber")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="DetectedSubscriber")
        self.blackboard.register_key("detected", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("last_detected_time", access=py_trees.common.Access.WRITE)
        
    def setup(self, **kwargs):
        self.subscription = self.node.create_subscription(
            Bool,
            Topic.detected,
            self.detected_callback,
            10
        )
        return True
        
    def detected_callback(self, msg):
        if msg.data and not self.blackboard.detected:
            # Detection just became true
            self.blackboard.last_detected_time = time.time()
        self.blackboard.detected = msg.data
        
    def update(self):
        return py_trees.common.Status.RUNNING

class PXModeSubscriber(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super(PXModeSubscriber, self).__init__(name="PX Mode Subscriber")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="PXModeSubscriber")
        self.blackboard.register_key("pxmode", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("px_heading", access=py_trees.common.Access.READ)
        self.blackboard.register_key("find_mode", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("buoy_visited_count", access=py_trees.common.Access.WRITE)
        
    def setup(self, **kwargs):
        self.subscription = self.node.create_subscription(
            UInt8,
            Topic.pxmode,
            self.pxmode_callback,
            10
        )
        return True
        
    def pxmode_callback(self, msg):
        old_pxmode = self.blackboard.pxmode
        self.blackboard.pxmode = msg.data
        
        # Reset mission if PX mode changes
        if old_pxmode != PxMode.HOLD.value and msg.data == PxMode.HOLD.value:
            self.blackboard.find_mode.set_initial_heading(self.blackboard.px_heading)
            self.blackboard.buoy_visited_count = 0
        
    def update(self):
        return py_trees.common.Status.RUNNING

# Main ROS2 Node
class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__('behavior_tree_node')
        
        # Create behavior tree
        self.tree = self.create_tree()
        
        # Setup periodic timer to tick the tree
        self.timer = self.create_timer(0.1, self.tick_tree)
        
        # Optional: setup visualization
        self.setup_visualization()
    
    def tick_tree(self):
        # Manually tick the tree since we're not using py_trees_ros
        self.tree.tick()
        
    # In the BehaviorTreeNode.create_tree method:
    def create_tree(self):
        # Root
        root = py_trees.composites.Parallel(
            name="Root",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
        )
        
        # Sensors parallel (always running)
        sensors = py_trees.composites.Parallel(
            name="Sensors",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
        )
        
        init_blackboard = InitializeBlackboard()
        
        # Add sensor subscribers
        heading_sub = HeadingSubscriber(self)
        detected_sub = DetectedSubscriber(self)
        pxmode_sub = PXModeSubscriber(self)
        
        # Create blackboard watcher (no self parameter needed)
        watcher = PrintBlackboard()
        
        # Add all behaviors to the sensors group
        sensors.add_children([heading_sub, detected_sub, pxmode_sub, watcher])
        
        root.add_children([init_blackboard, sensors])
        
        behavior_tree = py_trees.trees.BehaviourTree(root)
        return behavior_tree

    def setup_visualization(self):
        # Optional: setup tree visualization
        py_trees.display.render_dot_tree(self.tree.root)
        

def main(args=None):
    rclpy.init(args=args)
    
    # Start the simulator node
    simulator_node = MiniSimulator()
    
    # Start the behavior tree node
    bt_node = BehaviorTreeNode()
    
    # Use multithreading to run both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(simulator_node)
    executor.add_node(bt_node)
    
    # Create and start the thread for the executor
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    
    # Clean shutdown
    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()