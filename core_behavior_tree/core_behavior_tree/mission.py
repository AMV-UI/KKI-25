#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import py_trees
import py_trees_ros
import py_trees.console as console
from std_msgs.msg import UInt8, Bool, Float64
from core_msgs.msg import Option, AutoControl, StateObject, Pixhawk
from core.utils.config import Node as NodeConfig, PxMode, Direction, Param, Topic, BT
from core.utils.converter import autocontrolToString
from core.utils.frame_counter import FrameCounter
from core.utils.factory import MissionFactory


##FindMode dari script lama
class FindMode:
    def __init__(self):
        self.initial_heading = -1
        self.current_heading = -1
        self.threshold = 90
        self.range_low = -1
        self.range_high = -1
        self.find_status = "Left" if rclpy.get_param(Param.TRACK) == "A" else "Right"
        self.GO_LEFT = -200
        self.GO_RIGHT = 200
        self.dir_map = {
            "Right": self.GO_RIGHT,
            "Left": self.GO_LEFT,
        }

    def set_initial_heading(self, heading):
        self.initial_heading = heading

    def set_range(self, direction):
        if rclpy.get_param(Param.TRACK) == "A":
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


## Perhatikan input dia pakai py_trees.blackboard.Client bukan behavior, jadi gabisa pakai MissionFactory
class BlackboardROS(py_trees.blackboard.Client):
    """Blackboard for storing shared data between behaviors"""
    
    def __init__(self):
        super(BlackboardROS, self).__init__(name="MissionBlackboard")

        #iterasi ke semua nilai konfigurasi di class BT.ALL pada di core.utils.config
        for key, value in vars(BT.ALL).items():
            if not key.startswith("__") and isinstance(value, tuple):
                self.register_key(value[0], access=value[1])



## Init dengan default val
class InitializeBlackboard(py_trees.behaviour.Behaviour):
    """Initialize the blackboard with default values"""
    
    def __init__(self):
        super(InitializeBlackboard, self).__init__(name="Initialize Blackboard")
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
        return py_trees.common.Status.SUCCESS


class HeadingSubscriber(MissionFactory.BaseMission):
    """Subscribe to heading topic and update blackboard"""
    
    def __init__(self, node):
        super(HeadingSubscriber, self).__init__(name="Heading Subscriber")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="HeadingSubscriber")
        self.blackboard.register_key(*BT.ALL.px_heading)
        
    def setup(self, **kwargs):
        self.subscription = Topic.heading_deg.createSubscriber(self.node, self.heading_callback)
        return True
        
    def heading_callback(self, msg):
        self.blackboard.px_heading = msg.data
        
    def update(self):
        return py_trees.common.Status.RUNNING


class DetectedSubscriber(py_trees.behaviour.Behaviour):
    """Subscribe to detected topic and update blackboard"""
    
    def __init__(self, node):
        super(DetectedSubscriber, self).__init__(name="Detected Subscriber")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="DetectedSubscriber")
        self.blackboard.register_key(*BT.ALL.detected)
        
    def setup(self, **kwargs):
        self.subscription = Topic.buoy_detect.createSubscriber(self.node, self.detected_callback)
        return True
        
    def detected_callback(self, msg):
        self.blackboard.detected = msg.data
        
    def update(self):
        return py_trees.common.Status.RUNNING


class MissionSubscriber(py_trees.behaviour.Behaviour):
    """Subscribe to mission topic and update blackboard"""
    
    def __init__(self, node):
        super(MissionSubscriber, self).__init__(name="Mission Subscriber")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionSubscriber")
        self.blackboard.register_key("current_mission", access=py_trees.common.Access.WRITE)
        
    def setup(self, **kwa   rgs):
        self.subscription = self.node.create_subscription(
            UInt8,
            Topic.mission.value,
            self.mission_callback,
            10
        )
        return True
        
    def mission_callback(self, msg):
        self.blackboard.current_mission = msg.data
        
    def update(self):
        return py_trees.common.Status.RUNNING


class DSCSubscriber(py_trees.behaviour.Behaviour):
    """Subscribe to DSC topic and update blackboard"""
    
    def __init__(self, node):
        super(DSCSubscriber, self).__init__(name="DSC Subscriber")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="DSCSubscriber")
        self.blackboard.register_key("dsc", access=py_trees.common.Access.WRITE)
        
    def setup(self, **kwargs):
        self.subscription = self.node.create_subscription(
            Float64,
            Topic.dsc.value,
            self.dsc_callback,
            10
        )
        return True
        
    def dsc_callback(self, msg):
        self.blackboard.dsc = msg.data
        
    def update(self):
        return py_trees.common.Status.RUNNING


class PXModeSubscriber(py_trees.behaviour.Behaviour):
    """Subscribe to PX mode topic and update blackboard"""
    
    def __init__(self, node):
        super(PXModeSubscriber, self).__init__(name="PX Mode Subscriber")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="PXModeSubscriber")
        self.blackboard.register_key("pxmode", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("px_heading", access=py_trees.common.Access.READ)
        self.blackboard.register_key("find_mode", access=py_trees.common.Access.WRITE)
        
    def setup(self, **kwargs):
        self.subscription = self.node.create_subscription(
            UInt8,
            Topic.pxmode.value,
            self.pxmode_callback,
            10
        )
        return True
        
    def pxmode_callback(self, msg):
        old_pxmode = self.blackboard.pxmode
        self.blackboard.pxmode = msg.data
        
        # Reset mission if PX mode changes
        if old_pxmode != PxMode.HOLD and msg.data == PxMode.HOLD:
            self.blackboard.find_mode.set_initial_heading(self.blackboard.px_heading)
            self.blackboard.current_mission = AutoControl.MISSION_FIND_STEP_ONE
        
    def update(self):
        return py_trees.common.Status.RUNNING


class ImageSubscriber(py_trees.behaviour.Behaviour):
    """Subscribe to image topic and update blackboard"""
    
    def __init__(self, node):
        super(ImageSubscriber, self).__init__(name="Image Subscriber")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="ImageSubscriber")
        self.blackboard.register_key("image", access=py_trees.common.Access.WRITE)
        
    def setup(self, **kwargs):
        self.subscription = self.node.create_subscription(
            UInt8,  # Replace with actual image message type
            Topic.camera_processed.value,
            self.image_callback,
            10
        )
        return True
        
    def image_callback(self, msg):
        self.blackboard.image = msg.data
        
    def update(self):
        return py_trees.common.Status.RUNNING


class CameraBottomSubscriber(py_trees.behaviour.Behaviour):
    """Subscribe to bottom camera topic and update blackboard"""
    
    def __init__(self, node):
        super(CameraBottomSubscriber, self).__init__(name="Camera Bottom Subscriber")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="CameraBottomSubscriber")
        self.blackboard.register_key("camera_bottom", access=py_trees.common.Access.WRITE)
        
    def setup(self, **kwargs):
        self.subscription = self.node.create_subscription(
            UInt8,  # Replace with actual image message type
            Topic.camera_bottom.value,
            self.camera_bottom_callback,
            10
        )
        return True
        
    def camera_bottom_callback(self, msg):
        self.blackboard.camera_bottom = msg.data
        
    def update(self):
        return py_trees.common.Status.RUNNING


class CheckPXMode(py_trees.behaviour.Behaviour):
    """Check if PX mode is HOLD"""
    
    def __init__(self):
        super(CheckPXMode, self).__init__(name="Check PX Mode")
        self.blackboard = py_trees.blackboard.Client(name="CheckPXMode")
        self.blackboard.register_key("pxmode", access=py_trees.common.Access.READ)
        
    def update(self):
        if self.blackboard.pxmode == PxMode.HOLD:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


# Mission Step Behaviors
class MissionFindStepOne(py_trees.behaviour.Behaviour):
    """Mission to find buoy step one"""
    
    def __init__(self, node):
        super(MissionFindStepOne, self).__init__(name="Mission Find Step One")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionFindStepOne")
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("frame_counter", access=py_trees.common.Access.READ)
        self.blackboard.register_key("dsc", access=py_trees.common.Access.READ)
        self.blackboard.register_key("px_heading", access=py_trees.common.Access.READ)
        self.blackboard.register_key("find_mode", access=py_trees.common.Access.WRITE)
        
        # Publishers
        self.mission_pub = node.create_publisher(
            UInt8,
            Topic.mission.value,
            10
        )
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst.value,
            10
        )
        
    def initialise(self):
        self.get_logger().info(f"<=> [{NodeConfig.mission}] Entering improc for mission mencari buoy tahap 1")
        
    def update(self):
        dsc_state = self.blackboard.dsc
        self.state_dst_pub.publish(Float64(data=dsc_state))
        
        if self.blackboard.detected:
            self.blackboard.frame_counter.is_started()
            if self.blackboard.frame_counter.is_enough():
                self.get_logger().info(f"<=> [{NodeConfig.mission}] {self.blackboard.detected} Detecting...")
                self.blackboard.frame_counter.reset()
                return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.frame_counter.reset()
            
        self.blackboard.find_mode.set_initial_heading(self.blackboard.px_heading)
        return py_trees.common.Status.RUNNING


class MissionStepOne(py_trees.behaviour.Behaviour):
    """Mission step one"""
    
    def __init__(self, node):
        super(MissionStepOne, self).__init__(name="Mission Step One")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionStepOne")
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("frame_counter", access=py_trees.common.Access.READ)
        self.blackboard.register_key("dsc", access=py_trees.common.Access.READ)
        
        # Publishers
        self.mission_pub = node.create_publisher(
            UInt8,
            Topic.mission.value,
            10
        )
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst.value,
            10
        )
        
    def initialise(self):
        self.get_logger().info(f"<=> [{NodeConfig.mission}] Entering improc for mission step 1")
        
    def update(self):
        dsc_state = self.blackboard.dsc
        
        if not self.blackboard.detected:
            self.blackboard.frame_counter.is_started()
            if self.blackboard.frame_counter.is_enough():
                self.blackboard.frame_counter.reset()
                msg = UInt8()
                msg.data = AutoControl.MISSION_FIND_STEP_TWO
                self.mission_pub.publish(msg)
                self.get_logger().info(f"<=> [{NodeConfig.mission}] {self.blackboard.detected}")
                return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.frame_counter.reset()
            
        self.state_dst_pub.publish(Float64(data=dsc_state))
        return py_trees.common.Status.RUNNING


class MissionFindStepTwo(py_trees.behaviour.Behaviour):
    """Mission to find buoy step two"""
    
    def __init__(self, node):
        super(MissionFindStepTwo, self).__init__(name="Mission Find Step Two")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionFindStepTwo")
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("frame_counter", access=py_trees.common.Access.READ)
        self.blackboard.register_key("dsc", access=py_trees.common.Access.READ)
        self.blackboard.register_key("px_heading", access=py_trees.common.Access.READ)
        self.blackboard.register_key("find_mode", access=py_trees.common.Access.WRITE)
        
        # Publishers
        self.mission_pub = node.create_publisher(
            UInt8,
            Topic.mission.value,
            10
        )
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst.value,
            10
        )
        
    def initialise(self):
        self.get_logger().info(f"<=> [{NodeConfig.mission}] Entering improc for mission mencari buoy tahap 2")
        self.blackboard.find_mode.set_range(1)
        
    def update(self):
        self.blackboard.find_mode.current_heading = self.blackboard.find_mode.get_heading(self.blackboard.px_heading)
        dsc_state = self.blackboard.dsc
        
        if self.blackboard.detected:  # Tower Found
            self.blackboard.frame_counter.is_started()
            if self.blackboard.frame_counter.is_enough():
                self.blackboard.frame_counter.reset()
                msg = UInt8()
                msg.data = AutoControl.MISSION_STEP_TWO
                self.mission_pub.publish(msg)
                self.get_logger().info(f"<=> [{NodeConfig.mission}] {self.blackboard.detected}")
                return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.frame_counter.reset()
            if not self.blackboard.detected:
                dsc_state = self.blackboard.find_mode.get_state(self.blackboard.px_heading)
                self.get_logger().info(f"[{NodeConfig.mission}] {dsc_state}")
                
        self.get_logger().info(f"[{NodeConfig.mission}] Current Heading : {self.blackboard.px_heading} {self.blackboard.find_mode.range_low} {self.blackboard.find_mode.range_high}")
        self.get_logger().info(f"[{NodeConfig.mission}] Initial Heading : {self.blackboard.find_mode.initial_heading}")
        self.get_logger().info(f"[{NodeConfig.mission}] Base Heading : {self.blackboard.find_mode.current_heading}")
        
        self.state_dst_pub.publish(Float64(data=dsc_state))
        return py_trees.common.Status.RUNNING


class MissionStepTwo(py_trees.behaviour.Behaviour):
    """Mission step two"""
    
    def __init__(self, node):
        super(MissionStepTwo, self).__init__(name="Mission Step Two")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionStepTwo")
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("frame_counter", access=py_trees.common.Access.READ)
        self.blackboard.register_key("dsc", access=py_trees.common.Access.READ)
        
        # Publishers
        self.mission_pub = node.create_publisher(
            UInt8,
            Topic.mission.value,
            10
        )
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst.value,
            10
        )
        
    def initialise(self):
        self.get_logger().info(f"<=> [{NodeConfig.mission}] Entering improc for mission buoy tahap 2")
        
    def update(self):
        dsc_state = self.blackboard.dsc
        
        if not self.blackboard.detected:
            self.blackboard.frame_counter.is_started()
            if self.blackboard.frame_counter.is_enough():
                self.blackboard.frame_counter.reset()
                msg = UInt8()
                msg.data = AutoControl.MISSION_FIND_STEP_THREE
                self.mission_pub.publish(msg)
                self.get_logger().info(f"<=> [{NodeConfig.mission}] {self.blackboard.detected}")
                return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.frame_counter.reset()
            
        self.state_dst_pub.publish(Float64(data=dsc_state))
        return py_trees.common.Status.RUNNING


class MissionFindStepThree(py_trees.behaviour.Behaviour):
    """Mission to find buoy step three"""
    
    def __init__(self, node):
        super(MissionFindStepThree, self).__init__(name="Mission Find Step Three")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionFindStepThree")
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("frame_counter", access=py_trees.common.Access.READ)
        self.blackboard.register_key("dsc", access=py_trees.common.Access.READ)
        self.blackboard.register_key("px_heading", access=py_trees.common.Access.READ)
        self.blackboard.register_key("find_mode", access=py_trees.common.Access.WRITE)
        
        # Publishers
        self.mission_pub = node.create_publisher(
            UInt8,
            Topic.mission.value,
            10
        )
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst.value,
            10
        )
        
    def initialise(self):
        self.get_logger().info(f"<=> [{NodeConfig.mission}] Entering improc for mission mencari buoy tahap 3")
        self.blackboard.find_mode.set_range(2)
        
    def update(self):
        self.blackboard.find_mode.current_heading = self.blackboard.find_mode.get_heading(self.blackboard.px_heading)
        dsc_state = self.blackboard.dsc
        
        if self.blackboard.detected:
            self.blackboard.frame_counter.is_started()
            if self.blackboard.frame_counter.is_enough():
                self.blackboard.frame_counter.reset()
                msg = UInt8()
                msg.data = AutoControl.MISSION_STEP_THREE
                self.mission_pub.publish(msg)
                return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.frame_counter.reset()
            if not self.blackboard.detected:
                dsc_state = self.blackboard.find_mode.get_state(self.blackboard.px_heading)
                
        self.get_logger().info(f"[{NodeConfig.mission}] {self.blackboard.find_mode.initial_heading}")
        self.state_dst_pub.publish(Float64(data=dsc_state))
        return py_trees.common.Status.RUNNING


class MissionStepThree(py_trees.behaviour.Behaviour):
    """Mission step three"""
    
    def __init__(self, node):
        super(MissionStepThree, self).__init__(name="Mission Step Three")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionStepThree")
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("frame_counter", access=py_trees.common.Access.READ)
        self.blackboard.register_key("dsc", access=py_trees.common.Access.READ)
        self.blackboard.register_key("isChange", access=py_trees.common.Access.WRITE)
        
        # Publishers
        self.mission_pub = node.create_publisher(
            UInt8,
            Topic.mission.value,
            10
        )
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst.value,
            10
        )
        
    def initialise(self):
        self.get_logger().error(f"<=> [{NodeConfig.mission}] Entering improc for mission buoy tahap 3")
        
    def update(self):
        if rclpy.get_param(Param.TRACK) == "A" and not self.blackboard.isChange:
            rclpy.set_param(Param.TRACK, "B")
            self.blackboard.isChange = True
        elif rclpy.get_param(Param.TRACK) == "B" and not self.blackboard.isChange:
            rclpy.set_param(Param.TRACK, "A")
            self.blackboard.isChange = True
            
        dsc_state = self.blackboard.dsc
        
        if not self.blackboard.detected:
            self.blackboard.frame_counter.is_started()
            if self.blackboard.frame_counter.is_enough():
                self.blackboard.frame_counter.reset()
                return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.frame_counter.reset()
            
        self.state_dst_pub.publish(Float64(data=dsc_state))
        return py_trees.common.Status.RUNNING


class MissionManuver(py_trees.behaviour.Behaviour):
    """Mission manuver"""
    
    def __init__(self, node):
        super(MissionManuver, self).__init__(name="Mission Manuver")
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="MissionManuver")
        self.blackboard.register_key("detected", access=py_trees.common.Access.READ)
        self.blackboard.register_key("frame_counter_manuver", access=py_trees.common.Access.READ)
        self.blackboard.register_key("manuver_detected", access=py_trees.common.Access.WRITE)
        
        # Publishers
        self.mission_pub = node.create_publisher(
            UInt8,
            Topic.mission.value,
            10
        )
        self.state_dst_pub = node.create_publisher(
            Float64,
            Topic.state_dst.value,
            10
        )
        
    def initialise(self):
        self.get_logger().error(f"<=> [{NodeConfig.mission}] Entering improc for mission manuver")
        
    def update(self):
        if self.blackboard.detected:
            self.blackboard.manuver_detected = True
            # Flip di mission 3
            if rclpy.get_param(Param.TRACK) == "A":
                # Ke kiri
                self.state_dst_pub.publish(Float64(data=-200))
            else:
                # Ke kanan
                self.state_dst_pub.publish(Float64(data=200))
        elif self.blackboard.manuver_detected == True and self.blackboard.detected == False:
            self.blackboard.frame_counter_manuver.is_started()
            if rclpy.get_param(Param.TRACK) == "A":
                # Ke kiri
                self.state_dst_pub.publish(Float64(data=-200))
            else:
                # Ke kanan
                self.state_dst_pub.publish(Float64(data=200))

            if self.blackboard.frame_counter_manuver.is_enough():
                self.blackboard.frame_counter.reset()
                msg = UInt8()
                msg.data = AutoControl.MISSION_POSITION_GREEN_BOX
                self.mission_pub.publish(msg)
                return py_trees.common.Status.SUCCESS

        # Kasus ngga detek sama sekali dari awal
