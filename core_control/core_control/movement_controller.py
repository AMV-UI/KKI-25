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
        self.yaw_msg = Float64()
        self.speed_msg = Float64()
        
        self._setup_communication()
        
        self.get_logger().info(f"[{NodeConfig.movement_controller}] Successfully initialized")
        self.get_logger().info("Waiting for Pixhawk data...")
    
    def _setup_communication(self):
        """Initialize all ROS2 subscribers and publishers"""
        
        # Subscribers
        self.pixhawk_sub = Topic.pixhawk.createSubscriber(self, self._pixhawk_callback)
        self.pwm_sub = Topic.pwm.createSubscriber(self, self._pwm_callback)
        self.rc5_sub = Topic.rc5.createSubscriber(self, self._rc5_callback)
        self.rc6_sub = Topic.rc6.createSubscriber(self, self._rc6_callback)

    
        # self.docking_target_sub = self.create_subscription(
        #     Pixhawk, '/docking_target', self._docking_target_callback, 10
        # )
        # self.docking_enable_sub = self.create_subscription(
        #     Bool, '/docking_enable', self._docking_enable_callback, 10
        # )
        self.manual_yaw_sub = Topic.manual_yaw.createSubscriber(self, self._manual_yaw_callback)
        self.manual_speed_sub = Topic.manual_speed.createSubscriber(self, self._manual_speed_callback)
        
        # Publishers
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self)
        self.docking_status_pub = self.create_publisher(String, '/docking_status', 10)
        
        
        self.get_logger().info("Communication setup complete")
    
    def _rc5_callback(self, msg: Float64):
        self.rc5 = msg.data
        self._process_channel_controls()

    def _rc6_callback(self, msg: Float64):
        self.rc6 = msg.data
        self._process_channel_controls()

    def _pxmode_callback(self, msg: String):
        self.pxmode = msg.data

    def _manual_yaw_callback(self, msg: Float64):
        """Store manual yaw effort for recording"""
        self.manual_yaw_effort = msg.data
    
    def _manual_speed_callback(self, msg: Float64):
        """Store manual speed effort for recording"""
        self.manual_speed_effort = msg.data
    
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
        chan6_state = self._get_channel_state(self.rc6)
        
        if chan5_state != self.prev_chan5_state:
            if chan5_state == 'MID':
                # Set docking init point at current position
                self.docking_controller.set_target(self.current_lat, self.current_lon)
                self.docking_target_set = True
                self.docking_enabled = False
                self.get_logger().info(
                    f"[CH5-MID] Docking init point set: Lat={self.current_lat:.6f}, Lon={self.current_lon:.6f}"
                )
            
            elif chan5_state == 'HIGH':
                # Go to docking point
                if self.docking_target_set:
                    self.docking_enabled = True
                    self.docking_controller.reset_controller()
                    self.get_logger().info("[CH5-HIGH] Autonomous docking ENABLED - Going to docking point")
                else:
                    self.get_logger().warn("[CH5-HIGH] Cannot enable docking: No target set!")
            
            elif chan5_state == 'LOW':
                # Neutral - disable docking
                if self.docking_enabled:
                    self.docking_enabled = False
                    self.get_logger().info("[CH5-LOW] Docking DISABLED - Manual control")
            
            self.prev_chan5_state = chan5_state
        
        if chan6_state != self.prev_chan6_state:
            if chan6_state == 'MID':
                # Set docking point (home) and start recording
                if self.recording_state == RecordingState.IDLE:
                    if self.docking_target_set:
                        self.docking_enabled = True
                        self.docking_controller.reset_controller()
                        self.get_logger().info("[CH6-MID] Navigating back to home point...")
                    else:
                        self.docking_controller.set_target(self.current_lat, self.current_lon)
                        self.docking_target_set = True
                        self.docking_enabled = False
                        
                        self.docking_controller.start_lat_lon_recording(
                            self.current_lat,
                            self.current_lon,
                            self.current_heading
                        )
                        self.recording_state = RecordingState.RECORDING
                        self.initial_vehicle_lat = self.current_lat
                        self.initial_vehicle_lon = self.current_lon
                        self.initial_vehicle_heading = self.current_heading
                        self.has_left_dock = False
                        
                        self.get_logger().info(
                            f"[CH6-MID] Home point set and recording started at: Lat={self.current_lat:.6f}, Lon={self.current_lon:.6f}"
                        )
                        self.get_logger().info("Recording will automatically stop when you return to the home point.")
            
            elif chan6_state == 'HIGH':
                if self.recording_state == RecordingState.RECORDING:
                    self.get_logger().info("[CH6-HIGH] Recording in progress... Return to home point to auto-stop and playback.")
            
            elif chan6_state == 'LOW':
                if self.recording_state == RecordingState.PLAYING_BACK:
                    self.recording_state = RecordingState.IDLE
                    self.playback_index = 0
                    self.get_logger().info("[CH6-LOW] Playback STOPPED. Set CH6 to MID to navigate back to home.")
                elif self.recording_state == RecordingState.RECORDING:
                    num_frames = self.docking_controller.stop_lat_lon_recording()
                    self.recording_state = RecordingState.IDLE
                    self.get_logger().info(f"[CH6-LOW] Recording manually stopped ({num_frames} frames)")
            
            self.prev_chan6_state = chan6_state
    
    def update_recording_with_auto_stop(self, dt):
        """Update recording with automatic stop detection when returning to home"""
        if self.recording_state != RecordingState.RECORDING:
            return
        
        # Record current lat/lon
        self.docking_controller.record_lat_lon(self.current_lat, self.current_lon, dt)
        
        # Check if vehicle has reached the home point
        if self.docking_target_set:
            distance_to_home = self.docking_controller.get_distance_to_target(self.current_lat, self.current_lon)
            
            # Track if vehicle has left the docking area (moved at least 3m away)
            if not self.has_left_dock and distance_to_home > self.distance_threshold:
                self.has_left_dock = True
                self.get_logger().info("Left docking area - recording path...")
            
            # Only check for return to home after vehicle has left the area
            if self.has_left_dock and distance_to_home < self.docking_controller.docking_distance_threshold:
                num_frames = self.docking_controller.stop_lat_lon_recording()
                duration = self.docking_controller.get_lat_lon_duration()
                self.get_logger().info(f"Reached home point! Recording stopped. Recorded {num_frames} frames ({duration:.1f}s)")
                
                if num_frames > 0:
                    # Navigate back to home point (already set when recording started)
                    self.playback_lat_lon = self.docking_controller.get_recorded_lat_lon()
                    
                    # Start return navigation phase
                    self.playback_index = 0
                    self.recording_state = RecordingState.RETURNING_TO_START
                    self.get_logger().info(f"Navigating back to home point: Lat={self.initial_vehicle_lat:.6f}, Lon={self.initial_vehicle_lon:.6f}")
                else:
                    self.get_logger().warn("No movements recorded!")
                    self.recording_state = RecordingState.IDLE
    
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
                # Reached initial point, transition to playback
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
                self.get_logger().info("Docking complete! Switching to manual mode.")
            
            return yaw_effort, speed_effort
        
        # Priority 3: Recording mode (pass-through manual control but record)
        if self.is_recording:
            return 0.0, 0.0
        
        return 0.0, 0.0
    
    def publish_docking_status(self):
        """Publish current docking and recording status information"""
        status_msg = String()
        status_parts = []
        
        if self.recording_state == RecordingState.RECORDING:
            duration = self.docking_controller.get_lat_lon_duration()
            status_parts.append(f"RECORDING ({duration:.1f}s)")
        elif self.recording_state == RecordingState.PLAYING_BACK:
            progress = (self.playback_index / len(self.playback_lat_lon) * 100) if self.playback_lat_lon else 0
            status_parts.append(f"PLAYBACK ({progress:.0f}%)")
        elif self.recording_state == RecordingState.RETURNING_TO_START:
            status_parts.append("RETURNING TO START")
        
        if self.docking_target_set:
            distance = self.docking_controller.get_distance_to_target(
                self.current_lat, self.current_lon
            )
            heading_error = self.docking_controller.get_heading_error_deg()
            
            docking_status = (
                f"Docking {'ACTIVE' if self.docking_enabled else 'SET'} | "
                f"Dist: {distance:.2f}m | "
                f"Hdg Err: {heading_error:.1f}° | "
                f"Docked: {self.docking_controller.is_docked}"
            )
            status_parts.append(docking_status)
        
        status_parts.append(f"CH5:{self.prev_chan5_state} CH6:{self.prev_chan6_state}")
        
        status_msg.data = " | ".join(status_parts)
        self.docking_status_pub.publish(status_msg)
    
    def control_loop(self):
        try:
            current_time = time()
            dt = current_time - self.last_update_time
            
            yaw_effort, speed_effort = self.calculate_control_efforts()
            
            # Update recording with auto-stop detection
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
            
            # self.publish_docking_status()

            # if int(time() * 2) % 10 == 0:  # Every 5 seconds
            #     state_name = self.recording_state.name
            #     mode = state_name if self.recording_state != RecordingState.IDLE else ("DOCKING" if self.docking_enabled else "MANUAL")
            #     status = f"Mode: {mode} | Pos: ({self.current_lat:.6f}, {self.current_lon:.6f}) | Heading: {self.current_heading:.1f}° | "
            #     status += f"Yaw: {yaw_effort:.1f}, Speed: {speed_effort:.1f} | "
            #     status += f"CH5: {self.rc5} ({self.prev_chan5_state}), CH6: {self.rc6} ({self.prev_chan6_state})"
            #     self.get_logger().info(status)
        
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
