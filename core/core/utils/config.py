#!/usr/bin/env python3

import rclpy
import py_trees
from enum import Enum
from pymavlink import mavutil
from core.utils.factory import TopicFactory, ParamFactory
from std_msgs.msg import Float64, Bool, UInt8, String, UInt16, UInt8MultiArray, UInt32
from sensor_msgs.msg import Image, CompressedImage
from core_msgs.msg import (
    Controller,
    Config,
    Camera,
    KillSwitch,
    AutoControl,
    ObjectCount,
    Option,
    Pwm,
    Pixhawk,
    StateObject,
)

class Param:
    KP = ParamFactory("/yaw_controller/yaw_controller/Kp", float)
    KI = ParamFactory("/yaw_controller/yaw_controller/Ki", float)
    KD = ParamFactory("/yaw_controller/yaw_controller/Kd", float)
    THRESHOLD = ParamFactory("/camera_front/threshold", float)

    BOW_SPEED = ParamFactory("/motor_controller/bow_speed", float)
    CONF_THRESHOLD = ParamFactory("/mission/confidence_threshold", float)
    FLAG = ParamFactory("/mission/flag", bool)


class NodeConfig:
    camera_front = "camera_front"
    mission_controller = "mission_controller"
    motor_controller = "motor_controller"
    pid_controller = "pid_controller"
    depth_controller = "depth_controller"
    mission_control = "mission_controller"
    microcontroller = "microcontroller"
    cam_recorder = "cam_recorder"
    motor_utils = "motor_utils"
    param_controller = "param_controller"
    mission_manual = "mission_manual"
    mission = "mission"
    gcs = "GCS"
    movement_controller = "movement_controller"


class Topic:

    # Migrate From Param
    x_speed = ParamFactory("/motor_controller/x_speed", Float64)
    motor_speed = ParamFactory("/motor_controller/motor_speed", Float64)
    docking_lat = ParamFactory("/docking/target_latitude", Float64)
    docking_lon = ParamFactory("/docking/target_longitude", Float64)

    # Camera
    camera_processed = TopicFactory("/asv/vision/camera/processed", String)
    image_green_box = TopicFactory("/asv/vision/image/show_green", String)
    image_blue_box = TopicFactory("/asv/vision/image/show_blue", String)

    # Inference
    dsc = TopicFactory("/core/vision/image/dsc", Float64)
    detected = TopicFactory("/core/vision/image/detected", Bool)

    heading_deg = TopicFactory("/core/heading_deg", Float64)

    manual_yaw = TopicFactory("/core/manual_yaw", Float64)
    manual_speed = TopicFactory("/core/manual_speed", Float64)
    
    # Misc.
    kill_switch = TopicFactory("/core/kill_switch", KillSwitch)
    jetson_batt = TopicFactory("/core/battery/jetson", UInt16)
    motor_batt = TopicFactory("/core/battery/motor", UInt16)
    mux_state = TopicFactory("/core/mux_state", UInt8)

    # Motion Control
    # YAW = X AXIS (Left/Right)
    # SPEED = Y AXIS (Forward/Backward)
    yaw_effort = TopicFactory("/core/motor/yaw_effort", Float64)
    speed_effort = TopicFactory("/core/motor/speed_effort", Float64)

    rc5 = TopicFactory("/core/motor/rc5", Float64)
    rc6 = TopicFactory("/core/motor/rc6", Float64)

    # Mission
    mission = TopicFactory("/core/mission/current", UInt8)

    # PWM
    pwm = TopicFactory("/core/pwm", Pwm)

    # Micocontroller
    auto_status_remote = TopicFactory("/core/micon/auto_status_remote", UInt8)
    pico_raw = TopicFactory("/core/micon/pico_raw", String)
    pixhawk = TopicFactory("/core/micon/pixhawk", Pixhawk)
    pxmode = TopicFactory("/core/micon/pixhawk/mode", String)

    #Finding Mode
    initial_heading = TopicFactory("/core/initial/heading", UInt8) 

    # Tuning
    tuning_mission = TopicFactory("/core/tuning/mission", UInt8) 
    tuning_effort_st = TopicFactory("/core/tuning/effort/st", Float64)
    tuning_effort_tn = TopicFactory("/core/tuning/effort/tn", Float64)

    # Arena
    arena = TopicFactory("core/arena", String)

class BT:
    class ALL:
        px_heading = ("px_heading", py_trees.common.Access.WRITE)
        detected = ("detected", py_trees.common.Access.WRITE)
        current_mission = ("current_mission", py_trees.common.Access.WRITE)
        dsc = ("dsc", py_trees.common.Access.WRITE)
        pxmode = ("pxmode", py_trees.common.Access.WRITE)
        mission_status = ("mission_status", py_trees.common.Access.WRITE)
        frame_counter = ("frame_counter", py_trees.common.Access.WRITE)
        manuver_detected = ("manuver_detected", py_trees.common.Access.WRITE)
        isChange = ("isChange", py_trees.common.Access.WRITE)
        find_mode = ("find_mode", py_trees.common.Access.WRITE)
        frame_counter_manuver = ("frame_counter_manuver", py_trees.common.Access.WRITE)
        image = ("image", py_trees.common.Access.WRITE)
        camera_bottom = ("camera_bottom", py_trees.common.Access.WRITE)
        
class Direction:
    A = [0, 270, 180, 270]
    B = [0, 90, 180, 90]

class Tower:
    RED = "Red"
    GREEN = "Green"

    def __init__(self, node):
        self.node = node
        self.reverse_factory = ParamFactory(Param.REVERSEMODE, bool)

    @property
    def RED(self):
        return self.RED if not self.reverse_factory.getParam(self.node) else self.GREEN

    @property
    def GREEN(self):
        return self.GREEN if not self.reverse_factory.getParam(self.node) else self.RED4


class Camera:
    # Using YUY2 Format, Raw video stream
    # FRONT = "v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! nvvidconv ! video/x-raw(memory:NVMM) ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
    FRONT = "v4l2src device=/dev/video0 io-mode=2 ! image/jpeg, width=(int)1920, height=(int)1080, framerate=30/1 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
    LEFT = "v4l2src device=/dev/video1 io-mode=2 ! image/jpeg, width=(int)1920, height=(int)1080, framerate=30/1 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
    RIGHT = "v4l2src device=/dev/video2 io-mode=2 ! image/jpeg, width=(int)1920, height=(int)1080, framerate=30/1 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"

    TOP = "0"
    BOTTOM = "1"


class SPEED:
    Maximum = 1
    MediumFast = 0.7
    Medium = 0.5
    Slow = 0.3
    Idle = 0

class Channel:
    MOTOR_X = 0
    MOTOR_Y = 2

class PxMode:
    HOLD = "HOLD"
    MANUAL = "MANUAL"
    GUIDED = "GUIDED"
    AUTO = "AUTO"

class SETPOINT:
    SETPOINT_YAW = 320
    SETPOINT_DSC = 0

class MotorReverse:
    map = ()