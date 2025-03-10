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

# Simplified enums and constants
class PxMode(Enum):
    HOLD = 0
    MANUAL = 1

class AutoControl:
    MISSION_FIND_STEP_ONE = 0
    MISSION_STEP_ONE = 1
    MISSION_FIND_STEP_TWO = 2
    MISSION_STEP_TWO = 3
    MISSION_FIND_STEP_THREE = 4
    MISSION_STEP_THREE = 5
    MISSION_MANUVER = 6
    MISSION_POSITION_GREEN_BOX = 7

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
        self.blackboard.register_key("current_mission", access=py_trees.common.Access.WRITE)

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
        self.blackboard.current_mission = AutoControl.MISSION_FIND_STEP_ONE
        return py_trees.common.Status.SUCCESS

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
        
    def setup(self, **kwargs):
        self.subscription = self.node.create_subscription(
            Bool,
            Topic.detected,
            self.detected_callback,
            10
        )
        return True
        
    def detected_callback(self, msg):
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
        self.blackboard.register_key("current_mission", access=py_trees.common.Access.WRITE)
        
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
            self.blackboard.current_mission = AutoControl.MISSION_FIND_STEP_ONE
        
    def update(self):
        return py_trees.common.Status.RUNNING

# Mission Behaviors
class CheckPXMode(py_trees.behaviour.Behaviour):
    def __init__(self):
        super(CheckPXMode, self).__init__(name="Check PX Mode")
        self.blackboard = py_trees.blackboard.Client(name="CheckPXMode")
        self.blackboard.register_key("pxmode", access=py_trees.common.Access.READ)
        
    def update(self):
        if self.blackboard.pxmode == PxMode.HOLD.value:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class MissionFindStepOne(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super(MissionFindStepOne, self).__init__(name="Mission Find Step One")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionFindStepOne")
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("frame_counter", access=py_trees.common.Access.READ)
        self.blackboard.register_key("px_heading", access=py_trees.common.Access.READ)
        self.blackboard.register_key("find_mode", access=py_trees.common.Access.READ)
        self.blackboard.register_key("current_mission", access=py_trees.common.Access.WRITE)
        
        # Publishers
        self.mission_pub = node.create_publisher(
            UInt8,
            Topic.mission,
            10
        )
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst,
            10
        )
        
    def initialise(self):
        self.node.get_logger().info(f"<=> [{NodeConfig.mission}] Entering improc for mission mencari buoy tahap 1")
        
    def update(self):
        # Set direction
        dsc_state = 160.0  # Default value
        self.state_dst_pub.publish(Float64(data=dsc_state))
        
        if self.blackboard.detected:
            self.blackboard.frame_counter.is_started()
            if self.blackboard.frame_counter.is_enough():
                self.node.get_logger().info(f"<=> [{NodeConfig.mission}] {self.blackboard.detected} Detecting...")
                self.blackboard.frame_counter.reset()
                
                # Set next mission
                self.blackboard.current_mission = AutoControl.MISSION_STEP_ONE
                msg = UInt8()
                msg.data = AutoControl.MISSION_STEP_ONE
                self.mission_pub.publish(msg)
                return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.frame_counter.reset()
            
        # Set initial heading if valid
        if self.blackboard.px_heading >= 0:
            self.blackboard.find_mode.set_initial_heading(self.blackboard.px_heading)
            
        return py_trees.common.Status.RUNNING

class MissionStepOne(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super(MissionStepOne, self).__init__(name="Mission Step One")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionStepOne")
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("frame_counter", access=py_trees.common.Access.READ)
        self.blackboard.register_key("current_mission", access=py_trees.common.Access.WRITE)
        
        # Publishers
        self.mission_pub = node.create_publisher(
            UInt8,
            Topic.mission,
            10
        )
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst,
            10
        )
        
    def initialise(self):
        self.node.get_logger().info(f"<=> [{NodeConfig.mission}] Entering improc for mission step 1")
        
    def update(self):
        dsc_state = 160.0  # Default value
        
        if not self.blackboard.detected:
            self.blackboard.frame_counter.is_started()
            if self.blackboard.frame_counter.is_enough():
                self.blackboard.frame_counter.reset()
                
                # Set next mission
                self.blackboard.current_mission = AutoControl.MISSION_FIND_STEP_TWO
                msg = UInt8()
                msg.data = AutoControl.MISSION_FIND_STEP_TWO
                self.mission_pub.publish(msg)
                self.node.get_logger().info(f"<=> [{NodeConfig.mission}] Moving to mission find step 2")
                return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.frame_counter.reset()
            
        self.state_dst_pub.publish(Float64(data=dsc_state))
        return py_trees.common.Status.RUNNING

class MissionFindStepTwo(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super(MissionFindStepTwo, self).__init__(name="Mission Find Step Two")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionFindStepTwo")
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("frame_counter", access=py_trees.common.Access.READ)
        self.blackboard.register_key("px_heading", access=py_trees.common.Access.READ)
        self.blackboard.register_key("find_mode", access=py_trees.common.Access.READ)
        self.blackboard.register_key("current_mission", access=py_trees.common.Access.WRITE)
        
        # Publishers
        self.mission_pub = node.create_publisher(
            UInt8,
            Topic.mission,
            10
        )
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst,
            10
        )
        
    def initialise(self):
        self.node.get_logger().info(f"<=> [{NodeConfig.mission}] Entering improc for mission mencari buoy tahap 2")
        self.blackboard.find_mode.set_range(1)
        
    def update(self):
        if self.blackboard.px_heading >= 0:
            heading_state = self.blackboard.find_mode.get_state(self.blackboard.px_heading)
        else:
            heading_state = 160.0
            
        if self.blackboard.detected:  # Tower Found
            self.blackboard.frame_counter.is_started()
            if self.blackboard.frame_counter.is_enough():
                self.blackboard.frame_counter.reset()
                
                # Set next mission
                self.blackboard.current_mission = AutoControl.MISSION_STEP_TWO
                msg = UInt8()
                msg.data = AutoControl.MISSION_STEP_TWO
                self.mission_pub.publish(msg)
                self.node.get_logger().info(f"<=> [{NodeConfig.mission}] Moving to mission step 2")
                return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.frame_counter.reset()
                
        self.node.get_logger().info(f"[{NodeConfig.mission}] Current Heading: {self.blackboard.px_heading}")
        self.state_dst_pub.publish(Float64(data=heading_state))
        return py_trees.common.Status.RUNNING

class MissionStepTwo(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super(MissionStepTwo, self).__init__(name="Mission Step Two")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionStepTwo")
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("frame_counter", access=py_trees.common.Access.READ)
        self.blackboard.register_key("current_mission", access=py_trees.common.Access.WRITE)
        
        # Publishers
        self.mission_pub = node.create_publisher(
            UInt8,
            Topic.mission,
            10
        )
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst,
            10
        )
        
    def initialise(self):
        self.node.get_logger().info(f"<=> [{NodeConfig.mission}] Entering improc for mission buoy tahap 2")
        
    def update(self):
        dsc_state = 160.0  # Default value
        
        if not self.blackboard.detected:
            self.blackboard.frame_counter.is_started()
            if self.blackboard.frame_counter.is_enough():
                self.blackboard.frame_counter.reset()
                
                # Set next mission
                self.blackboard.current_mission = AutoControl.MISSION_FIND_STEP_THREE
                msg = UInt8()
                msg.data = AutoControl.MISSION_FIND_STEP_THREE
                self.mission_pub.publish(msg)
                self.node.get_logger().info(f"<=> [{NodeConfig.mission}] Moving to mission find step 3")
                return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.frame_counter.reset()
            
        self.state_dst_pub.publish(Float64(data=dsc_state))
        return py_trees.common.Status.RUNNING

class MissionFindStepThree(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super(MissionFindStepThree, self).__init__(name="Mission Find Step Three")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionFindStepThree")
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("frame_counter", access=py_trees.common.Access.READ)
        self.blackboard.register_key("px_heading", access=py_trees.common.Access.READ)
        self.blackboard.register_key("find_mode", access=py_trees.common.Access.READ)
        self.blackboard.register_key("current_mission", access=py_trees.common.Access.WRITE)
        
        # Publishers
        self.mission_pub = node.create_publisher(
            UInt8,
            Topic.mission,
            10
        )
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst,
            10
        )
        
    def initialise(self):
        self.node.get_logger().info(f"<=> [{NodeConfig.mission}] Entering improc for mission mencari buoy tahap 3")
        self.blackboard.find_mode.set_range(2)
        
    def update(self):
        if self.blackboard.px_heading >= 0:
            heading_state = self.blackboard.find_mode.get_state(self.blackboard.px_heading)
        else:
            heading_state = 160.0
        
        if self.blackboard.detected:
            self.blackboard.frame_counter.is_started()
            if self.blackboard.frame_counter.is_enough():
                self.blackboard.frame_counter.reset()
                
                # Set next mission
                self.blackboard.current_mission = AutoControl.MISSION_STEP_THREE
                msg = UInt8()
                msg.data = AutoControl.MISSION_STEP_THREE
                self.mission_pub.publish(msg)
                self.node.get_logger().info(f"<=> [{NodeConfig.mission}] Moving to mission step 3")
                return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.frame_counter.reset()
                
        self.state_dst_pub.publish(Float64(data=heading_state))
        return py_trees.common.Status.RUNNING

class MissionStepThree(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super(MissionStepThree, self).__init__(name="Mission Step Three")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionStepThree")
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("frame_counter", access=py_trees.common.Access.READ)
        self.blackboard.register_key("isChange", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("find_mode", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("current_mission", access=py_trees.common.Access.WRITE)
        
        # Publishers
        self.mission_pub = node.create_publisher(
            UInt8,
            Topic.mission,
            10
        )
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst,
            10
        )
        
    def initialise(self):
        self.node.get_logger().info(f"<=> [{NodeConfig.mission}] Entering improc for mission buoy tahap 3")
        
    def update(self):
        # Toggle track
        if self.blackboard.find_mode.get_track() == "A" and not self.blackboard.isChange:
            self.blackboard.find_mode.set_track("B")
            self.blackboard.isChange = True
            self.node.get_logger().info("Switching track to B")
        elif self.blackboard.find_mode.get_track() == "B" and not self.blackboard.isChange:
            self.blackboard.find_mode.set_track("A") 
            self.blackboard.isChange = True
            self.node.get_logger().info("Switching track to A")
            
        dsc_state = 160.0  # Default value
        
        if not self.blackboard.detected:
            self.blackboard.frame_counter.is_started()
            if self.blackboard.frame_counter.is_enough():
                self.blackboard.frame_counter.reset()
                
                # Set next mission
                self.blackboard.current_mission = AutoControl.MISSION_MANUVER
                msg = UInt8()
                msg.data = AutoControl.MISSION_MANUVER
                self.mission_pub.publish(msg)
                self.node.get_logger().info(f"<=> [{NodeConfig.mission}] Moving to mission manuver")
                return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.frame_counter.reset()
            
        self.state_dst_pub.publish(Float64(data=dsc_state))
        return py_trees.common.Status.RUNNING

class MissionManuver(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super(MissionManuver, self).__init__(name="Mission Manuver")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionManuver")
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("frame_counter_manuver", access=py_trees.common.Access.READ)
        self.blackboard.register_key("manuver_detected", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("find_mode", access=py_trees.common.Access.READ)
        self.blackboard.register_key("current_mission", access=py_trees.common.Access.WRITE)
        
        # Publishers
        self.mission_pub = node.create_publisher(
            UInt8,
            Topic.mission,
            10
        )
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst,
            10
        )
        
    def initialise(self):
        self.node.get_logger().info(f"<=> [{NodeConfig.mission}] Entering improc for mission manuver")
        
    def update(self):
        if self.blackboard.detected:
            self.blackboard.manuver_detected = True
            # Manuver direction
            if self.blackboard.find_mode.get_track() == "A":
                # Left
                self.state_dst_pub.publish(Float64(data=-200))
                self.node.get_logger().info("Maneuvering left")
            else:
                # Right
                self.state_dst_pub.publish(Float64(data=200))
                self.node.get_logger().info("Maneuvering right")
        elif self.blackboard.manuver_detected == True and not self.blackboard.detected:
            self.blackboard.frame_counter_manuver.is_started()
            # Continue manuver
            if self.blackboard.find_mode.get_track() == "A":
                self.state_dst_pub.publish(Float64(data=-200))
            else:
                self.state_dst_pub.publish(Float64(data=200))

            if self.blackboard.frame_counter_manuver.is_enough():
                # Set next mission
                self.blackboard.current_mission = AutoControl.MISSION_POSITION_GREEN_BOX
                msg = UInt8()
                msg.data = AutoControl.MISSION_POSITION_GREEN_BOX
                self.mission_pub.publish(msg)
                self.node.get_logger().info(f"<=> [{NodeConfig.mission}] Moving to green box positioning")
                return py_trees.common.Status.SUCCESS
        else:
            # Default behavior when nothing is detected yet
            self.state_dst_pub.publish(Float64(data=160.0))

        return py_trees.common.Status.RUNNING

class MissionPositionGreenBox(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super(MissionPositionGreenBox, self).__init__(name="Position Green Box")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionPositionGreenBox")
        
        # Publishers
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst,
            10
        )
        
    def initialise(self):
        self.node.get_logger().info(f"<=> [{NodeConfig.mission}] Positioning green box - MISSION COMPLETE")
        
    def update(self):
        # Final positioning
        self.state_dst_pub.publish(Float64(data=0.0))
        return py_trees.common.Status.SUCCESS

# Mission selector behavior
class MissionSelector(py_trees.behaviour.Behaviour):
    def __init__(self, name="Mission Selector"):
        super(MissionSelector, self).__init__(name=name)
        self.blackboard = py_trees.blackboard.Client(name="MissionSelector")
        self.blackboard.register_key("current_mission", access=py_trees.common.Access.READ)
        
    def update(self):
        # This behavior is used to select the appropriate mission based on current_mission
        # It's mainly for visualization purposes in the behavior tree
        return py_trees.common.Status.SUCCESS




# Main ROS2 Node
class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__('behavior_tree_node')
        
        # Create behavior tree
        self.tree = self.create_tree()
        
        # Setup periodic timer to tick the tree
        self.timer = self.create_timer(0.1, self.tick_tree)
        
        # Optional: setup visualization (modified for ROS2)
        self.setup_visualization()
    
    def tick_tree(self):
        # Manually tick the tree since we're not using py_trees_ros
        self.tree.tick()
        
    def create_tree(self):
        # Root
        root = py_trees.composites.Parallel(
            name="Root",
            policy=py_trees.common.ParallelPolicy
        )

        
        # Sensors parallel
        sensors = py_trees.composites.Parallel(
            name="Sensors",
            policy=py_trees.common.ParallelPolicy
        )
        
        # Initialize blackboard
        init_blackboard = InitializeBlackboard()
        
        # Add sensor subscribers
        heading_sub = HeadingSubscriber(self)
        detected_sub = DetectedSubscriber(self)
        pxmode_sub = PXModeSubscriber(self)
        
        sensors.add_children([heading_sub, detected_sub, pxmode_sub])
        
        # Mission control sequence
        mission_control = py_trees.composites.Sequence(name="Mission Control", memory=True)
        
        # Check PX mode condition
        check_pxmode = CheckPXMode()
        
        # Mission behaviors
        mission_selection = py_trees.composites.Selector(name="Mission Selection", memory=True)
        
        # Create mission behaviors
        find_step_one = MissionFindStepOne(self)
        step_one = MissionStepOne(self)
        find_step_two = MissionFindStepTwo(self)
        step_two = MissionStepTwo(self)
        find_step_three = MissionFindStepThree(self)
        step_three = MissionStepThree(self)
        manuver = MissionManuver(self)
        position_green_box = MissionPositionGreenBox(self)
        
        # Create conditions for missions using blackboard variables
        
        # We need to create blackboard clients for conditions
        bb_current_mission = py_trees.blackboard.Client(name="CurrentMission")
        bb_current_mission.register_key("current_mission", access=py_trees.common.Access.READ)
        
        # Mission Find Step One
        is_find_step_one = py_trees.decorators.EternalGuard(
            name="Is Find Step One",
            child=find_step_one,
            condition=lambda: bb_current_mission.current_mission == AutoControl.MISSION_FIND_STEP_ONE
        )
        
        # Mission Step One
        is_step_one = py_trees.decorators.EternalGuard(
            name="Is Step One",
            child=step_one,
            condition=lambda: bb_current_mission.current_mission == AutoControl.MISSION_STEP_ONE
        )
        
        # The rest of the mission guards follow the same pattern
        is_find_step_two = py_trees.decorators.EternalGuard(
            name="Is Find Step Two",
            child=find_step_two,
            condition=lambda: bb_current_mission.current_mission == AutoControl.MISSION_FIND_STEP_TWO
        )
        
        is_step_two = py_trees.decorators.EternalGuard(
            name="Is Step Two",
            child=step_two,
            condition=lambda: bb_current_mission.current_mission == AutoControl.MISSION_STEP_TWO
        )
        
        is_find_step_three = py_trees.decorators.EternalGuard(
            name="Is Find Step Three",
            child=find_step_three,
            condition=lambda: bb_current_mission.current_mission == AutoControl.MISSION_FIND_STEP_THREE
        )
        
        is_step_three = py_trees.decorators.EternalGuard(
            name="Is Step Three",
            child=step_three,
            condition=lambda: bb_current_mission.current_mission == AutoControl.MISSION_STEP_THREE
        )
        
        is_manuver = py_trees.decorators.EternalGuard(
            name="Is Manuver",
            child=manuver,
            condition=lambda: bb_current_mission.current_mission == AutoControl.MISSION_MANUVER
        )
        
        is_position_green_box = py_trees.decorators.EternalGuard(
            name="Is Position Green Box",
            child=position_green_box,
            condition=lambda: bb_current_mission.current_mission == AutoControl.MISSION_POSITION_GREEN_BOX
        )
        
        # Add all mission conditions to mission selection
        mission_selection.add_children([
            is_find_step_one,
            is_step_one,
            is_find_step_two,
            is_step_two,
            is_find_step_three,
            is_step_three,
            is_manuver,
            is_position_green_box
        ])
        
        # Add check_pxmode and mission_selection to mission_control
        mission_control.add_children([check_pxmode, mission_selection])
        
        # Add init_blackboard, sensors, and mission_control to root
        root.add_children([init_blackboard, sensors, mission_control])
        
        # return root
        return root

    def setup_visualization(self):
        # Export ke bentuk png dan svg buat visualizaton tree
        py_trees.display.render_dot_tree(self.tree)
        


# Main function
def main(args=None):
    rclpy.init(args=args)
    
    # Create nodes
    simulator_node = MiniSimulator()
    behavior_tree_node = BehaviorTreeNode()
    
    # Use MultiThreadedExecutor to run nodes concurrently
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(simulator_node)
    executor.add_node(behavior_tree_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown nodes
        simulator_node.destroy_node()
        behavior_tree_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()