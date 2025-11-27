#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8, Float64, Bool
from core.utils.config import Topic, PxMode, Param, NodeConfig
from core_msgs.msg import Pixhawk, Pwm
from core.mission.docking import DockingController
import traceback
from time import time
import math
from enum import Enum

class RecordingState(Enum):
    IDLE = 0
    RECORDING = 1
    RETURNING_TO_START = 2
    PLAYING_BACK = 3

class MovementController(Node):
    """
    Advanced Movement Controller with Autonomous Docking Capability
    
    This controller integrates the DockingController for autonomous navigation
    and provides both manual control and autonomous docking modes.
    
    Modes:
    - MANUAL: Direct control via yaw_effort and speed_effort topics
    - AUTONOMOUS_DOCKING: Uses DockingController to navigate to target position

    #Channel 5 = Docking
    #Channel 6 = Record Coord

    #Magic Numbers => :
        #LOW: Chan : 983
        #MID: Chan  : 1495
        #HIGH: Chan : 2006
    
    """
    
    def __init__(self):
        super().__init__(NodeConfig.movement_controller)
        
        self.docking_controller = DockingController()
        
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0  # In degrees (Pixhawk: 0=North, 90=East, 180=South, 270=West, clockwise)
        self.current_speed = 0.0
        
        self.manual_yaw_effort = 0.0
        self.manual_speed_effort = 0.0
        
        self.PWM_LOW = 1000
        self.PWM_MID = 1500
        self.PWM_HIGH = 1700
        self.PWM_THRESHOLD = 200

        self.MAX_PWM = 300
        self.MIN_PWM = -300
        
        self.docking_enabled = False
        self.docking_target_set = False
        self.last_update_time = time()
        
        
        self.prev_chan5_state = 'LOW'
        self.prev_chan6_state = 'LOW'
        
        self.recording_state = RecordingState.IDLE
        self.has_left_dock = False  # Track if vehicle has moved away from docking point
        self.initial_vehicle_lat = 0.0
        self.initial_vehicle_lon = 0.0
        self.initial_vehicle_heading = 0.0

        self.waypoint_threshold = 0.1
        self.playback_index = 0
        self.playback_lat_lon = []
        
        self.control_rate = 50.0
        self.dt = 1.0 / self.control_rate

        self.rc5 = float()
        self.rc6 = float()
        self.yaw_msg = Float64()
        self.speed_msg = Float64()
        
        self._setup_communication()
        
        self.get_logger().info(f"[{NodeConfig.movement_controller}] Successfully initialized")
        self.get_logger().info("Waiting for Pixhawk data...")
    
    def _setup_communication(self):
        """Initialize all ROS2 subscribers and publishers"""
        
        self.pixhawk_sub = Topic.pixhawk.createSubscriber(self, self._pixhawk_callback)
        self.pwm_sub = Topic.pwm.createSubscriber(self, self._pwm_callback)
        self.rc5_sub = Topic.rc5.createSubscriber(self, self._rc5_callback)
        self.rc6_sub = Topic.rc6.createSubscriber(self, self._rc6_callback)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self)
        self.manual_yaw_sub = Topic.manual_yaw.createSubscriber(self, self._manual_yaw_callback)
        self.manual_speed_sub = Topic.manual_speed.createSubscriber(self, self._manual_speed_callback)
        self.recorded_path_pub = Topic.recorded_path.createPublisher(self)
        
        self.get_logger().info("Communication setup complete")
    
    def _rc5_callback(self, msg: Float64):
        self.rc5 = msg.data
        self._process_channel_controls()

    def _rc6_callback(self, msg: Float64):
        self.rc6 = msg.data

    
    def _pixhawk_callback(self, msg: Pixhawk):
        """Update current position and heading from Pixhawk"""
        self.current_lat = msg.lat
        self.current_lon = msg.lon
        self.current_speed = msg.msg_spd
        self.current_heading = msg.msg_heading
    
    def _pwm_callback(self, msg: Pwm):
        """Update RC channel values from PWM message"""
        self.pwm_chan = msg.channels

    def _manual_yaw_callback(self, msg: Float64):
        """Store manual yaw effort for recording"""
        self.manual_yaw_effort = msg.data
    
    def _manual_speed_callback(self, msg: Float64):
        """Store manual speed effort for recording"""
        self.manual_speed_effort = msg.data
    
    def _docking_target_callback(self, msg: Pixhawk):
        """Set docking target position"""
        target_lat = msg.lat
        target_lon = msg.lon
        
        self.docking_controller.set_target(target_lat, target_lon)
        self.docking_target_set = True
        
        self.get_logger().info(
            f"Docking target set: Lat={target_lat:.6f}, Lon={target_lon:.6f}"
        )
    
    def _docking_enable_callback(self, msg: Bool):
        """Enable or disable autonomous docking"""
        if msg.data and not self.docking_target_set:
            self.get_logger().warn("Cannot enable docking: No target set!")
            return
        
        self.docking_enabled = msg.data
        
        if self.docking_enabled:
            self.docking_controller.reset_controller()
            self.get_logger().info("Autonomous docking ENABLED")
        else:
            self.get_logger().info("Autonomous docking DISABLED")
    
    def _get_channel_state(self, pwm_value):
        """
        Determine channel state from PWM value.
        
        Returns:
        str: 'LOW', 'MID', or 'HIGH'
        """
        if pwm_value < self.PWM_LOW:
            return 'LOW'
        elif pwm_value > self.PWM_HIGH:
            return 'HIGH'
        else:
            return 'MID'
    
    def calculate_control_efforts(self):
        #TODO: finish this
        
    
    def control_loop(self):
        try:
            yaw_effort, speed_effort = self.calculate_control_efforts()
            
            yaw_effort = max(self.MIN_PWM, min(self.MAX_PWM, yaw_effort))
            speed_effort = max(self.MIN_PWM, min(self.MAX_PWM, speed_effort))
            
            self.yaw_msg.data = float(yaw_effort)
            self.speed_msg.data = float(speed_effort)

            self.yaw_effort_pub.publish(self.yaw_msg)
            self.speed_effort_pub.publish(self.speed_msg)
        
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {traceback.format_exc()}")
    
    def run(self):
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"Movement controller running at {self.control_rate} Hz")

def main(args=None):
    try:
        rclpy.init(args=args)
        
        movement_controller = MovementController()
        movement_controller.run()
        
        rclpy.spin(movement_controller)
        
    except Exception as e:
        print(f"Error in main: {traceback.format_exc()}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
