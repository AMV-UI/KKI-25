#!/usr/bin/env python3

import rclpy
import py_trees
from enum import Enum
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
    TRACK = ParamFactory("/mission/track", str)
    BOW_SPEED = ParamFactory("/motor_controller/bow_speed", float)
    X_SPEED = ParamFactory("/motor_controller/x_speed", float)
    MOTOR_SPEED = ParamFactory("/motor_controller/motor_speed", float)
    CONF_THRESHOLD = ParamFactory("/mission/confidence_threshold", float)

    DOCKING_LAT = ParamFactory("/docking/target_latitude", float)
    DOCKING_LON = ParamFactory("/docking/target_longitude", float)
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


class Topic:

    # Motor_controller
    controller = TopicFactory('/controller', Controller)

    # Autocontrol/Mode
    auto_control = TopicFactory("asv/auto_control/", AutoControl)

    # Camera
    camera_raw = TopicFactory("/asv/vision/camera/raw", Image)
    # camera_compressed = TopicFactory(
    #     '/kki24/vision/camera/compressed', String)
    camera_processed = TopicFactory("/asv/vision/camera/processed", String)
    camera_config = TopicFactory("/asv/vision/camera/config", Camera)
    image_green_box = TopicFactory("/asv/vision/image/show_green", String)
    image_blue_box = TopicFactory("/asv/vision/image/show_blue", String)

    dsc = TopicFactory("/core/vision/image/dsc", Float64)
    dsc_flag = TopicFactory("/core/vision/image/dsc_flag", Float64)
    state_object = TopicFactory("/core/vision/image/state", StateObject)
    detected = TopicFactory("/core/vision/image/detected", Bool)

    # GCS Config
    gcs_config = TopicFactory("/core/gcs/config", Config)

    # Misc.
    fail_safe = TopicFactory("/core/fail_safe", Bool)
    kill_switch = TopicFactory("/core/kill_switch", KillSwitch)
    heading_deg = TopicFactory("/core/heading_deg", Float64)
    jetson_batt = TopicFactory("/core/battery/jetson", UInt16)
    motor_batt = TopicFactory("/core/battery/motor", UInt16)
    internal_temp_deg = TopicFactory("/core/temp/internal", Float64)
    mux_state = TopicFactory("/core/mux_state", UInt8)
    diagnostics = TopicFactory("/core/diagnostics", String)


    # PID Controller Topics
    # imagine a 2d space like turtlesim
    # YAW = X AXIS
    # SPEED = Y AXIS
    yaw_effort = TopicFactory("/core/pid_controller/yaw_effort", Float64)
    speed_effort = TopicFactory("/core/pid_controller/speed_effort", Float64)


    # Object Detected
    object_detected = TopicFactory("/core/vision/object_detected", Bool)
    object_counted = TopicFactory("/core/vision/object_counted", ObjectCount)

    # Mission
    mission = TopicFactory("/core/mission/current", UInt8)
    mission_counter = TopicFactory("/core/mission/counter", UInt8)

    # Reverse autonomous mode
    reverse_auto_mode = TopicFactory("/core/reverse_auto_mode", Bool)

    # Strategy Option
    strat_option = TopicFactory("/core/strat", Option)

    # PWM
    pwm = TopicFactory("/core/pwm", Pwm)

    # Micon <> GCS
    # killswitch = TopicFactory('/asv/')
    auto_status_remote = TopicFactory("/core/micon/auto_status_remote", UInt8)
    auto_status_gcs = TopicFactory("/core/micon/auto_status_gcs", UInt8)
    pico_raw = TopicFactory("/core/micon/pico_raw", String)
    # compass = TopicFactory('/asv/micon/compass', String)
    # coordinate = TopicFactory('/asv/micon/coordinate', String)
    # micon_yaw = TopicFactory('/asv/micon/yaw', Float64)
    pixhawk = TopicFactory("/core/micon/pixhawk", Pixhawk)
    pxmode = TopicFactory("/core/micon/pixhawk/mode", String)

    # Echosounder
    echosounder_dist = TopicFactory("/core/echosounder/distance", Float64)
    echosounder_conf = TopicFactory("/core/echosounder/confidence", Float64)

    #rov tambahan
    state_depth = TopicFactory("/core/rov/depth", Float64)
    pxmode_uint8 = TopicFactory("/core/micon/pixhawk/mode", UInt8)

    #Finding Mode
    initial_heading = TopicFactory("/core/initial/heading", UInt8) 

    # Tuning
    tuning_mission = TopicFactory("/core/tuning/mission", UInt8) 
    tuning_effort = TopicFactory("/core/tuning/effort", Float64)

class BT:

    ### PY_TREES STATUS 
    failure = py_trees.common.Status.FAILURE
    running = py_trees.common.Status.RUNNING
    invalid = py_trees.common.Status.INVALID 
    success = py_trees.common.Status.SUCCESS

    ### PY_TREES PERMISSIONS
    read = py_trees.common.Access.READ
    write = py_trees.common.Access.WRITE

    ### RCLPY QOS  (Quality Of Service) for PubSub
    volatile = rclpy.qos.DurabilityPolicy.VOLATILE
    reliable = rclpy.qos.ReliabilityPolicy.RELIABLE

    ### 
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
        


class AutoState:
    HARDWARE = 0
    AUTO = 1
    MANUAL = 2


class RemoteState:
    TBS_AUTO = 0
    TBS_MANUAL = 1

class Direction:
    A = [0, 270, 180, 270]
    B = [0, 90, 180, 90]


class Buoys:
    def __init__(self, node):
        self.node = node
        self.reverse_factory = ParamFactory(Param.REVERSEMODE, bool)

    @property
    def RED(self):
        return int(not self.reverse_factory.getParam(self.node))

    @property
    def GREEN(self):
        return int(self.reverse_factory.getParam(self.node))

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


class Box:
    BLUE = "Blue"
    GREEN = "Green"


class ArduinoCfg:
    PORT = "/dev/ttyUSB0"
    BAUDRATE = 9600
    TIMEOUT = 1


class Camera:
    # Using YUY2 Format, Raw video stream
    # FRONT = "v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! nvvidconv ! video/x-raw(memory:NVMM) ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"

    # Using MJPEG Format (razer Kiyo), Compressed video stream
    FRONT = "v4l2src device=/dev/video0 io-mode=2 ! image/jpeg, width=(int)1920, height=(int)1080, framerate=30/1 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
    LEFT = "v4l2src device=/dev/video1 io-mode=2 ! image/jpeg, width=(int)1920, height=(int)1080, framerate=30/1 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
    RIGHT = "v4l2src device=/dev/video2 io-mode=2 ! image/jpeg, width=(int)1920, height=(int)1080, framerate=30/1 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"

    TOP = "0"
    BOTTOM = "1"
    # Using FFMPEG, not supported by Jetson Module (by default)
    # FRONT = 0


class Channel:
    MOTOR_X = 0
    MOTOR_Y = 2
    # KILL_SWITCH = 7


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


class DetectedStatus:
    NOT_DETECTED = 0
    ONE_DETECTED = 1
    BOTH_DETECTED = 2


class ModelPath:
    buoy = "model/buoy/buoy.engine"


class SPEED:
    Maximum = 1
    MediumFast = 0.7
    Medium = 0.5
    Slow = 0.3
    Idle = 0
