#!/usr/bin/env python3

import os
from py_trees.common import Status
import rclpy
import rclpy.executors
from rclpy.node import Node
import time
import random
import py_trees
import py_trees.console as console
import rclpy.qos
from std_msgs.msg import UInt8, Bool, Float64
# from core_msgs.msg import Option, AutoControl, StateObject, Pixhawk
from core.utils.config import Node as NodeConfig, PxMode, Direction, Param, Topic, BT
from core.utils.converter import autocontrolToString
from core.utils.frame_counter import FrameCounter
from core.utils.factory import MissionFactory, ParamFactory, TopicFactory

import threading

class TopicToBlackBoard(py_trees.behaviour.Behaviour):
    def __init__(self, topic_name : str, topic_factory : TopicFactory):
        super(TopicToBlackBoard, self).__init__(topic_name + "_subscriber")
        self.topic_name = topic_name
        self.topic_factory = topic_factory
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(topic_name, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("ros_node", access=py_trees.common.Access.READ)
        self.node = self.blackboard.ros_node
    
    def setup(self, **kwargs):
        self.subscriber = self.topic_factory.createSubscriber(self.node, self.subscriber_callback)
        return True

    def subscriber_callback(self, msg):
        self.blackboard.set(name=self.topic_name, value=msg.data)
 
    def update(self):
        return py_trees.common.Status.SUCCESS

class ManuallyMoveBoat(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super(ManuallyMoveBoat, self).__init__(name)
        self.disconnect_manual_mode_after_this_amount_of_ticks = 10
    
    def setup(self, **kwargs) -> None:
        # Setup microcontroller / motor publishers here
        pass
    
    def update(self):
        if (self.disconnect_manual_mode_after_this_amount_of_ticks < 0):
            return py_trees.common.Status.SUCCESS
        self.disconnect_manual_mode_after_this_amount_of_ticks -= 1
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status: Status) -> None:
        self.disconnect_manual_mode_after_this_amount_of_ticks = 10

class BaseTestMission(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super(BaseTestMission, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key("ros_node", access=py_trees.common.Access.READ)
        self.blackboard.register_key("mission_counter", access=py_trees.common.Access.READ)
        
        self.blackboard.register_key(BT.ALL.px_heading[0], access=py_trees.common.Access.READ)
        self.node : Node = self.blackboard.ros_node
    
    def setup(self, **kwargs) -> None:
        qos = rclpy.qos.QoSProfile(
            depth=10,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )
        self.mission_counter_publisher = self.node.create_publisher(UInt8, "/mission_counter", qos)
    
    def update(self) -> Status:
        stuff = [Status.SUCCESS, Status.FAILURE, Status.RUNNING]
        status : Status = stuff[round(self.blackboard.get("px_heading"))]
        match status:
            case Status.SUCCESS:
                self.node.get_logger().info(f"MISSION {self.blackboard.mission_counter} SUCCESS, CONTINUING :)")
                msg = UInt8()
                msg.data = self.blackboard.mission_counter + 1
                self.mission_counter_publisher.publish(msg)
            case Status.RUNNING:
                self.node.get_logger().info(f"still doing mission {self.blackboard.mission_counter} ...")
        return status

class FallbackAction(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super(FallbackAction, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key("mission_counter", access=py_trees.common.Access.READ)
        self.blackboard.register_key("ros_node", access=py_trees.common.Access.READ)
        self.node : Node = self.blackboard.ros_node
        
    def update(self) -> Status:
        self.node.get_logger().info(f"Mission {self.blackboard.mission_counter} Failure, falling back...")
        return Status.FAILURE
    
##FindMode dari script lama
class FindMode:
    def __init__(self, node):
        self.initial_heading = -1
        self.current_heading = -1
        self.threshold = 90
        self.range_low = -1
        self.range_high = -1
        self.node = node
        
        # Instance of ParamFactory refers to a single parameter (name & type)
        # self.param_track = ParamFactory(Param.TRACK, str)
        # Each node "has-a" relationship with parameter
        # self.param_track.createParam(self.node, "A") # createParam(node, defaultValue)
        # self.find_status = "Left" if self.param_track.getParam(self.node) == "A" else "Right"
        self.GO_LEFT = -200
        self.GO_RIGHT = 200
        self.dir_map = {
            "Right": self.GO_RIGHT,
            "Left": self.GO_LEFT,
        }

    def set_initial_heading(self, heading):
        self.initial_heading = heading

    def set_range(self, direction):
        if self.param_track.getParam(self.node) == "A":
            self.range_low = Direction.A[direction] - self.threshold
            self.range_high = Direction.A[direction] + self.threshold
        else:
            self.range_low = Direction.B[direction] - self.threshold
            self.range_high = Direction.B[direction] + self.threshold
    
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
    
    def __init__(self, node):
        super(InitializeBlackboard, self).__init__(name="InitBlackboard")
        self.blackboard = py_trees.blackboard.Client(name="Init")
        self.node = node
        # Set behavior tree ros node to blackboard, to ensure behaviors have access to ROS
        self.blackboard.register_key("ros_node", access=py_trees.common.Access.WRITE)
        self.blackboard.ros_node = node
        # Temporary blackboard variable for testing behavior tree
        self.blackboard.register_key("mission_counter", access=py_trees.common.Access.WRITE)
        self.blackboard.mission_counter = 0
        # iterasi untuk register ke blackboard untuk semua nilai konfigurasi di class BT.ALL pada di core.utils.config
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
        self.blackboard.find_mode = FindMode(self.node)
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
        '''
        for key, value in vars(BT.ALL).items():
            if not key.startswith("__") and isinstance(value, tuple):
                print(self.blackboard.get(value[0]))
        '''
        #print(self.blackboard)
        #os.system('cls||clear')
        
        return BT.running 

# # Main ROS2 Node
class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__('behavior_tree_node')

        self.tree = self.create_tree()
        self.timer = self.create_timer(0.1, self.tick_tree)
        self.setup_visualization()
        self.tree.setup()

    def tick_tree(self):
        self.tree.tick()

    def create_tree(self):
        
        
        
        root = py_trees.composites.Parallel(
            name="Root",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
        )
        init = py_trees.composites.Sequence(
            name="Init",
            memory=True
        )
        tasks = py_trees.composites.Selector(
            name="Tasks",
            memory=False
        )
        watcher = PrintBlackboard()
        root.add_children([init, tasks, watcher])
        
        init_blackboard = InitializeBlackboard(self)
        init_heading_deg_subscriber = TopicToBlackBoard("px_heading", Topic.heading_deg)
        init.add_children([init_blackboard, init_heading_deg_subscriber])
        
        
        mission_sequence = py_trees.composites.Sequence(name="Mission Sequence", memory=False)
        mission1_selector = py_trees.composites.Selector(name="mission1Succeed?", memory=True)
        mission2_selector = py_trees.composites.Selector(name="mission2Succeed?", memory=True)
        mission3_selector = py_trees.composites.Selector(name="mission3Succeed?", memory=True)
        mission_sequence.add_children([mission1_selector, mission2_selector, mission3_selector])
        
        mission1 = BaseTestMission("mission1")
        mission1_fallback = FallbackAction("mission1 Fallback")
        mission1_selector.add_children([mission1, mission1_fallback])
        
        mission2 = BaseTestMission("mission2")
        mission2_fallback = FallbackAction("mission2 Fallback")
        mission2_selector.add_children([mission2, mission2_fallback])
        
        mission3 = BaseTestMission("mission3")
        mission3_fallback = FallbackAction("mission3 Fallback")
        mission3_selector.add_children([mission3, mission3_fallback])
        
        pxmode_manual = ManuallyMoveBoat("ManualMovement")
        
        def check_pxmode(blackboard):
            return True if blackboard.pxmode == "manual" else False
        
        isManualMoving = py_trees.decorators.EternalGuard(
            "is manual moving?",
            pxmode_manual,
            check_pxmode,
            blackboard_keys={"pxmode"}
        )
        
        
        
        tasks.add_children([isManualMoving, mission_sequence, py_trees.behaviours.Running("Idle")])

        behavior_tree = py_trees.trees.BehaviourTree(root)
        return behavior_tree

    def setup_visualization(self):
        py_trees.display.render_dot_tree(self.tree.root)


def main(args=None):
    rclpy.init(args=args)

    bt_node = BehaviorTreeNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(bt_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():
            print(py_trees.display.ascii_tree(bt_node.tree.root, show_status=True))
            time.sleep(0.1)
            os.system('cls||clear')
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()
