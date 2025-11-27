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
        
        self.distance_threshold = 2.0
        
        self.prev_chan5_state = 'LOW'
        self.prev_chan6_state = 'LOW'
        
        self.recording_state = RecordingState.IDLE
        self.has_left_dock = False  # Track if vehicle has moved away from docking point
        self.initial_vehicle_lat = 0.0
        self.initial_vehicle_lon = 0.0
        self.initial_vehicle_heading = 0.0
        self.playback_index = 0
        self.playback_lat_lon = []
        
        self.control_rate = 50.0
        self.dt = 1.0 / self.control_rate

        self.rc5 = float()
        self.rc6 = float()
        self.dt = time()
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
        
        # Store heading directly in Pixhawk convention (degrees, 0-360)
        # Pixhawk: 0 = North, 90 = East, 180 = South, 270 = West (clockwise)
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
    
    def _process_channel_controls(self):
        """
        Process RC channel inputs for docking and recording control.
        
        Channel 5 (Docking):
        - LOW: Neutral (manual control)
        - MID: Set docking init point (current position)
        - HIGH: Go to docking point (autonomous)
        
        Channel 6 (Recording):
        - LOW: Neutral (stop recording/playback)
        - MID: Start recording movements
        - HIGH: Stop recording and playback from start
        """
        chan5_state = self._get_channel_state(self.rc5)
        
        if chan5_state != self.prev_chan5_state:
            if chan5_state == 'MID':
                self.docking_controller.set_target(self.current_lat, self.current_lon)
                self.docking_target_set = True
                self.docking_enabled = False

                self.recording_state = RecordingState.RECORDING
                self.docking_controller.start_lat_lon_recording(
                    self.current_lat,
                    self.current_lon,
                    self.current_heading
                )
                
                self.get_logger().info(
                    f"[CH5-MID] Docking point set & recording started: "
                    f"Lat={self.current_lat:.6f}, Lon={self.current_lon:.6f}"
                )

            elif chan5_state == 'HIGH':
                if self.docking_target_set and self.recording_state == RecordingState.RECORDING:
                    num_points = self.docking_controller.stop_lat_lon_recording()
                    self.recording_state = RecordingState.IDLE
                    
                    self.docking_enabled = True
                    self.docking_controller.reset_controller()
                    
                    self.get_logger().info(
                        f"[CH5-HIGH] Recording stopped ({num_points} waypoints). "
                        f"Returning to dock point..."
                        f"Starting Playback..."
                    )
                elif self.docking_target_set:
                    self.docking_enabled = True
                    self.docking_controller.reset_controller()
                    self.get_logger().info("[CH5-HIGH] Autonomous docking ENABLED")
                else:
                    self.get_logger().warn("[CH5-HIGH] Cannot enable docking: No target set!")
            
            elif chan5_state == 'LOW':
                if self.docking_enabled:
                    self.docking_enabled = False
                    self.get_logger().info("[CH5-LOW] Docking DISABLED - Manual control")
                if self.recording_state != RecordingState.IDLE:
                    self.recording_state = RecordingState.IDLE
                    self.get_logger().info("[CH5-LOW] Recording/Playback cancelled")
            
            self.prev_chan5_state = chan5_state
        
    
    
    def calculate_control_efforts(self):
        """
        Calculate control efforts based on current mode.
        Priority: RETURNING_TO_START > Playback > Docking > Recording > Manual
        
        Returns:
        tuple: (yaw_effort, speed_effort)
        """
        current_time = time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Priority 1: RETURNING_TO_START - navigate back to initial recording point
        if self.recording_state == RecordingState.RETURNING_TO_START:
            yaw_effort, speed_effort, is_docked = self.docking_controller.calculate_control_efforts(
                self.current_lat,
                self.current_lon,
                self.current_heading,
                dt
            )
            
            if is_docked:
                distance = self.docking_controller.get_distance_to_target(self.current_lat, self.current_lon)
                self.get_logger().info(f"Reached initial point! Distance: {distance:.2f}m")
                self.get_logger().info("Starting playback...")
                self.recording_state = RecordingState.PLAYING_BACK
                self.playback_index = 0
                return 0.0, 0.0
            
            return yaw_effort, speed_effort
        
        # Priority 2: Playback mode
        if self.recording_state == RecordingState.PLAYING_BACK and len(self.playback_lat_lon) > 0:
            if self.playback_index < len(self.playback_lat_lon):
                target_lat, target_lon, _ = self.playback_lat_lon[self.playback_index]
                
                if self.docking_controller.target_lat != target_lat or self.docking_controller.target_lon != target_lon:
                    self.docking_controller.set_target(target_lat, target_lon)
                
                yaw_effort, speed_effort, is_docked = self.docking_controller.calculate_control_efforts(
                    self.current_lat,
                    self.current_lon,
                    self.current_heading,
                    dt
                )
                
                distance_to_target = self.docking_controller.get_distance_to_target(self.current_lat, self.current_lon)
                
                if distance_to_target < self.distance_threshold:
                    self.playback_index += 1
                
                return yaw_effort, speed_effort
            else:
                self.recording_state = RecordingState.IDLE
                self.playback_index = 0
                self.get_logger().info("Playback complete!")
                return 0.0, 0.0
        
        # Priority 3: Autonomous docking mode
        if self.docking_enabled and self.docking_target_set:
            yaw_effort, speed_effort, is_docked = self.docking_controller.calculate_control_efforts(
                self.current_lat,
                self.current_lon,
                self.current_heading,
                dt
            )
            
            if is_docked:
                self.docking_enabled = False
                self.get_logger().info("Docking complete!")
                
                # Check if we have a recorded path to playback
                if self.docking_controller.has_lat_lon():
                    self.playback_lat_lon = self.docking_controller.get_recorded_lat_lon()
                    self.recording_state = RecordingState.PLAYING_BACK
                    self.playback_index = 0
                    self.get_logger().info(
                        f"Starting playback with {len(self.playback_lat_lon)} waypoints..."
                    )
                else:
                    self.get_logger().info("No recorded path available for playback")
            
            return yaw_effort, speed_effort
        
        # Priority 3: Recording mode (pass-through manual control but record)
        if self.recording_state == RecordingState.RECORDING:
            return 0.0, 0.0
        
        return 0.0, 0.0
    
    
    def control_loop(self):
        try:
            current_time = time()
            dt = current_time - self.last_update_time
            # self._process_channel_controls()
            
            yaw_effort, speed_effort = self.calculate_control_efforts()
            
            if self.recording_state == RecordingState.RECORDING:
                self.update_recording_with_auto_stop(dt)
                yaw_effort = self.manual_yaw_effort
                speed_effort = self.manual_speed_effort

            yaw_effort = max(self.MIN_PWM, min(self.MAX_PWM, yaw_effort))
            speed_effort = max(self.MIN_PWM, min(self.MAX_PWM, speed_effort))
            
            self.yaw_msg.data = yaw_effort
            self.speed_msg.data = speed_effort

            self.yaw_effort_pub.publish(self.yaw_msg)
            self.speed_effort_pub.publish(self.speed_msg)
        
        
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {traceback.format_exc()}")
    
    def run(self):
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"Movement controller running at {self.control_rate} Hz")
    
    def set_target_from_current_position(self):
        """
        Convenience method to set current position as docking target.
        Useful for testing or marking waypoints.
        """
        self.docking_controller.set_target(self.current_lat, self.current_lon)
        self.docking_target_set = True
        self.get_logger().info(
            f"Docking target set to current position: "
            f"Lat={self.current_lat:.6f}, Lon={self.current_lon:.6f}"
        )
    
    def update_recording_with_auto_stop(self, dt):
        """
        Update recording with current position during manual control.
        Records lat/lon waypoints as the vehicle moves.
        
        Parameters:
        dt (float): Time delta since last update
        """
        if self.recording_state == RecordingState.RECORDING:
            recorded = self.docking_controller.record_lat_lon(
                self.current_lat,
                self.current_lon,
                dt
            )
            if recorded:
                self.get_logger().info(
                    f"Recorded waypoint: Lat={self.current_lat:.6f}, Lon={self.current_lon:.6f}"
                )


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
