#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import py_trees
import py_trees.console as console
from std_msgs.msg import UInt8, Bool, Float64
# from core_msgs.msg import Option, AutoControl, StateObject, Pixhawk
from core.utils.config import Node as NodeConfig, PxMode, Direction, Param, Topic, BT
from core.utils.converter import autocontrolToString
from core.utils.frame_counter import FrameCounter
from core.utils.factory import MissionFactory, ParamFactory

import threading 
##FindMode dari script lama
class FindMode:
    def __init__(self):
        self.initial_heading = -1
        self.current_heading = -1
        self.threshold = 90
        self.range_low = -1
        self.range_high = -1

        ##TODO: Fix ParamFactory usage
        # self.find_status = "Left" if ParamFactory().get_param(Param.TRACK) == "A" else "Right"
        
        self.find_status = "Left"
        self.GO_LEFT = -200
        self.GO_RIGHT = 200
        self.dir_map = {
            "Right": self.GO_RIGHT,
            "Left": self.GO_LEFT,
        }

    def set_initial_heading(self, heading):
        self.initial_heading = heading

    ##TODO: Fix ParamFactory usage
    # def set_range(self, direction):
    #     if ParamFactory().get_param(Param.TRACK) == "A":
    #         self.range_low = Direction.A[direction] - self.threshold
    #         self.range_high = Direction.A[direction] + self.threshold
    #     else:
    #         self.range_low = Direction.B[direction] - self.threshold
    #         self.range_high = Direction.B[direction] + self.threshold
    #
    def get_heading(self, raw_heading):
        heading = raw_heading - self.initial_heading
        if heading < 0:
            heading = 360 + heading
        return heading

    def get_state(self, raw_heading):
        self.current_heading = self.get_heading(raw_heading)
        # Handle out-of-bounds cases
        lb = abs(self.current_heading - self.range_low)
        if self.range_low == 0:
            lb = min(lb, abs(self.current_heading - 360))

        ub = abs(self.current_heading - self.range_high)
        if self.range_high == 0:
            ub = min(ub, abs(self.current_heading - 360))

        if (
            not (self.range_low if self.range_low == 360 else 0)
            < self.current_heading
            < (self.range_high if self.range_high != 0 else 360)
        ):
            if lb < ub:
                self.find_status = "Right"
            else:
                self.find_status = "Left"

        return self.dir_map[self.find_status]


class InitializeBlackboard(py_trees.behaviour.Behaviour):
    """
    Initialize the blackboard with default values
    
    Perbedaan blackboard dengan topic konvensional karena
    dia bisa menyimpan state meskipun "publishernya" berhenti sehingga krusial
    dalam melakukan decision making.
    """
    
    def __init__(self):
        super(InitializeBlackboard, self).__init__(name="InitBlackboard")
        self.blackboard = py_trees.blackboard.Client(name="Init")

        #iterasi untuk register ke blackboard untuk semua nilai konfigurasi di class BT.ALL pada di core.utils.config
        for key, value in vars(BT.ALL).items():
            if not key.startswith("__") and isinstance(value, tuple):
                self.blackboard.register_key(value[0], access=value[1])

    def update(self):

        #init constants
        self.blackboard.px_heading = -1
        self.blackboard.detected = False
        self.blackboard.dsc = float(160)
        self.blackboard.pxmode = PxMode.HOLD
        self.blackboard.mission_status = "RUNNING"
        self.blackboard.frame_counter = FrameCounter(2)
        self.blackboard.frame_counter_manuver = FrameCounter(3)
        self.blackboard.manuver_detected = False
        self.blackboard.isChange = False
        self.blackboard.find_mode = FindMode()
        self.blackboard.image = None
        self.blackboard.camera_bottom = None
        self.blackboard.current_mission = "-"  # Initialize with a default value
        print(self.blackboard)        
        return BT.success 

class PrintBlackboard(py_trees.behaviour.Behaviour):
    def __init__(self):
        super(PrintBlackboard, self).__init__(name="Print Blackboard")
        self.blackboard = py_trees.blackboard.Client(name="BlackboardMonitor")
        
        #iterasi untuk register ke blackboard untuk semua nilai konfigurasi di class BT.ALL pada di core.utils.config
        for key, value in vars(BT.ALL).items():
            if not key.startswith("__") and isinstance(value, tuple):
                self.blackboard.register_key(value[0], access=value[1])
        
    def update(self):
        for key, value in vars(BT.ALL).items():
            if not key.startswith("__") and isinstance(value, tuple):
                print(self.blackboard.get(value[0]))
        return BT.running 


# Sensor subscribers
class HeadingSubscriber(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super(HeadingSubscriber, self).__init__(name="Heading Subscriber")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="HeadingSubscriber")
        self.blackboard.register_key("px_heading", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.subscription = Topic.heading_deg.create_subscriber(self)        

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
        self.subscription = Topic.detected.create_subscriber(self)

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
        self.subscription = Topic.pxmode.create_subsciber(self)

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

# # Main ROS2 Node
class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__('behavior_tree_node')

        self.tree = self.create_tree()
        self.timer = self.create_timer(0.1, self.tick_tree)
        self.setup_visualization()

    def tick_tree(self):
        self.tree.tick()

    def create_tree(self):
        root = py_trees.composites.Parallel(
            name="Root",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
        )

        sensors = py_trees.composites.Parallel(
            name="Sensors",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
        )

        init_blackboard = InitializeBlackboard()

        heading_sub = HeadingSubscriber(self)
        detected_sub = DetectedSubscriber(self)
        pxmode_sub = PXModeSubscriber(self)

        watcher = PrintBlackboard()

        sensors.add_children([heading_sub, detected_sub, pxmode_sub, watcher])
        
        root.add_children([init_blackboard, sensors])

        behavior_tree = py_trees.trees.BehaviourTree(root)
        return behavior_tree

    def setup_visualization(self):
        py_trees.display.render_dot_tree(self.tree.root)


def main(args=None):
    rclpy.init(args=args)

    # simulator_node = MiniSimulator()

    # Start the behavior tree node
    bt_node = BehaviorTreeNode()
    #
    test_node = InitializeBlackboard()

    # Use multithreading to run both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(simulator_node)
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
