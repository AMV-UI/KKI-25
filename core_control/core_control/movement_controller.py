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


import sys
import termios
import tty
import select
import threading


class Waypoint(Enum):
    IDLE = 0
    START = 1
    DOCKING = 2

class Gcs(Enum):
    IDLE = 0
    TAKE_PHOTO_CAMERA_1 = 1
    TAKE_PHOTO_CAMERA_2 = 2


class KeyboardInput:
    """Non-blocking keyboard input handler for ROS2"""
    
    def __init__(self, node):
        self.node = node
        self.manual_speed_effort = 0.0
        self.manual_yaw_effort = 0.0
        self.use_simulator = True
        
        # Store terminal settings
        self.settings = None
        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)
        
        self.running = False
        self.keyboard_thread = None
        
    def start_keyboard_listener(self):
        """Start the keyboard listener thread"""
        if not sys.stdin.isatty():
            self.node.get_logger().warn("Not running in a terminal - keyboard input disabled")
            return
        
        self.running = True
        self.keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.keyboard_thread.start()
        
        self.node.get_logger().info("Keyboard control started:")
        self.node.get_logger().info("  W - Forward")
        self.node.get_logger().info("  S - Backward")
        self.node.get_logger().info("  A - Turn Left")
        self.node.get_logger().info("  D - Turn Right")
        self.node.get_logger().info("  Q - Quit")
        self.node.get_logger().info("  SPACE - Stop")
    
    def stop_keyboard_listener(self):
        """Stop the keyboard listener and restore terminal settings"""
        self.running = False
        if self.keyboard_thread:
            self.keyboard_thread.join(timeout=1.0)
        
        if self.settings and sys.stdin.isatty():
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def _keyboard_loop(self):
        """Main keyboard listening loop (runs in separate thread)"""
        try:
            # Set terminal to raw mode for character-by-character input
            tty.setraw(sys.stdin.fileno())
            
            while self.running:
                # Check if input is available (non-blocking with 0.1s timeout)
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    self._process_key(key)
                    
        except Exception as e:
            self.node.get_logger().error(f"Keyboard thread error: {e}")
        finally:
            # Restore terminal settings
            if self.settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def _process_key(self, key):
        """Process a single key press"""
        if not self.use_simulator:
            return
        
        # Reset efforts first (for single-key press behavior)
        # Remove these lines if you want continuous movement until stop
        # self.manual_speed_effort = 0.0
        # self.manual_yaw_effort = 0.0
        
        if key == 'w':
            self.manual_speed_effort = 300.0  # Forward
            self.manual_yaw_effort = 0.0
            self.node.get_logger().info("Forward")
            
        elif key == 's':
            self.manual_speed_effort = -300.0  # Backward
            self.manual_yaw_effort = 0.0
            self.node.get_logger().info("Backward")
            
        elif key == 'a':
            self.manual_yaw_effort = -300.0  # Turn left
            self.manual_speed_effort = 0.0
            self.node.get_logger().info("Turn Left")
            
        elif key == 'd':
            self.manual_yaw_effort = 300.0  # Turn right
            self.manual_speed_effort = 0.0
            self.node.get_logger().info("Turn Right")
            
        elif key == ' ':
            self.manual_speed_effort = 0.0  # Stop
            self.manual_yaw_effort = 0.0
            self.node.get_logger().info("Stop")
            
        elif key == 'q':
            self.node.get_logger().info("Quit requested")
            self.running = False
            
    def get_manual_efforts(self):
        """Get current manual control efforts"""
        return self.manual_yaw_effort, self.manual_speed_effort


class MovementController(Node):
    """
    TrueCoding
    """
    
    def __init__(self):
        super().__init__(NodeConfig.movement_controller)
        
        self.docking_controller = DockingController()

        self.use_simulator = True
        self.keyboard_input = KeyboardInput(self)
        if self.use_simulator:
            self.keyboard_input.start_keyboard_listener()
        
        self.init_lat_lon = False
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

        self.rc5_state = 'LOW'
        self.rc6_state = 'LOW'
        
        self.is_docked = False
        self.playback_mode = False
        
        self.waypoint_state = Waypoint.IDLE
        self.gcs_state = Gcs.IDLE
        
        self.control_rate = 50.0
        self.dt = 1.0 / self.control_rate

        self.delta_time = 0.1
        self.last_time = time()

        self.rc5 = float()
        self.rc6 = float()
        self.yaw_msg = Float64()
        self.speed_msg = Float64()
        
        self._setup_communication()
        
        self.get_logger().info(f"[{NodeConfig.movement_controller}] Successfully initialized")
        self.get_logger().info("Waiting for Pixhawk data...")

    def _set_docking_target(self, target_lat, target_lon):
        """Set the target position for docking"""
        self.docking_controller.set_target(target_lat, target_lon)
        self.get_logger().info(f"Docking target set to: {target_lat}, {target_lon}")

    def _start_recording(self):
        """
        return manual yaw and speed effort and is_docked for recording
        """
        if not self.init_lat_lon:
            self._set_docking_target(self.current_lat, self.current_lon)
            self.init_lat_lon = True

        if not self.docking_controller.is_recording:
            self.docking_controller.start_recording()
            self.docking_controller.start_lat_lon_recording(
                self.current_lat,
                self.current_lon,
                self.current_heading
            )
            self.get_logger().info("Started recording manual movements")

        if self.docking_controller.is_recording:
            self.docking_controller.record_movement(
                self.manual_yaw_effort,
                self.manual_speed_effort,
                self.delta_time
            )
        
        return self.manual_yaw_effort, self.manual_speed_effort, False
    
    def _start_docking_and_playback(self, delta_time):
        """
        returns yaw_effort, speed_effort, finished
        """
        if not self.playback_mode:
            self.docking_controller.start_docking()
            yaw_effort, speed_effort, is_docked = self.docking_controller.calculate_control_efforts(
                self.current_lat,
                self.current_lon,
                self.current_heading,
                delta_time
            )

            self.is_docked = is_docked
            self.get_logger().info(f"Docking state: {self.is_docked}")

            if self.is_docked:
                self.docking_controller.stop_recording()
                self.playback_mode = True
                self.get_logger().info("Switching to playback mode!!! 😎")

            return yaw_effort, speed_effort, is_docked

        yaw_effort, speed_effort, finished = self.docking_controller.execute_playback_latlon(
            self.current_lat,
            self.current_lon,
            self.current_heading,
            delta_time
        )

        if finished:
            self.playback_mode = False
            self.get_logger().info("Playback complete.")
        
        return yaw_effort, speed_effort, finished

    
    def _waypoint_idle_executor(self):
        """
        return zero yaw and speed effort and is_docked for idle
        """
        return 0.0, 0.0, False

    def _take_photo_camera_1(self):
        """
        return image from camera 1
        """
        pass

    def _take_photo_camera_2(self):
        """
        return image from camera 2
        """
        pass

    def _gcs_idle_executor(self):
        """
        return image from camera 1 and 2
        """
        pass 

    def _waypoint_executor(self, delta_time):
        """
        localizer for waypoint functions, returns yaw and speed effort
        """
        if self.waypoint_state == Waypoint.START:
            return self._start_recording()
        elif self.waypoint_state == Waypoint.DOCKING:
            return self._start_docking_and_playback(delta_time)
        elif self.waypoint_state == Waypoint.IDLE:
            return self._waypoint_idle_executor()

    def _gcs_executor(self):
        """
        localizer for gcs functions
        """
        if self.recording_state == Gcs.TAKE_PHOTO_CAMERA_1:
            self._take_photo_camera_1()
        elif self.recording_state == Gcs.TAKE_PHOTO_CAMERA_2:
            self._take_photo_camera_2()
        elif self.recording_state == Gcs.IDLE:
            self._gcs_idle_executor()

    def _rc5_decision_maker(self, delta_time):
        """
        will return yaw and speed effort from the self._waypoint_executor()
        """
        if self.rc5_state == 'MID':
            self.waypoint_state = Waypoint.START
        elif self.rc5_state == 'HIGH':
            self.waypoint_state = Waypoint.DOCKING
        elif self.rc5_state == 'LOW':
            self.waypoint_state = Waypoint.IDLE
        
        yaw_effort, speed_effort, self.is_docked = self._waypoint_executor(delta_time)
        return yaw_effort, speed_effort

    def _rc6_decision_maker(self):
        if self.rc6_state == 'MID':
            self.recording_state = Gcs.TAKE_PHOTO_CAMERA_1
        elif self.rc6_state == 'HIGH':
            self.recording_state = Gcs.TAKE_PHOTO_CAMERA_2
        elif self.rc6_state == 'LOW':
            self.recording_state = Gcs.IDLE
        
        self._gcs_executor()

    
    def _setup_communication(self):
        """Initialize all ROS2 subscribers and publishers"""        
        self.pixhawk_sub = Topic.pixhawk.createSubscriber(self, self._pixhawk_callback)
        self.pwm_sub = Topic.pwm.createSubscriber(self, self._pwm_callback)


        self.rc5_sub = Topic.rc5.createSubscriber(self, self._rc5_callback) #for docking + playback
        self.rc6_sub = Topic.rc6.createSubscriber(self, self._rc6_callback) #for taking photo

        self.image1_pub = Topic.image_blue_box.createPublisher(self)
        self.image2_pub = Topic.image_green_box.createPublisher(self)

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self)

        self.manual_yaw_sub = Topic.manual_yaw.createSubscriber(self, self._manual_yaw_callback)
        self.manual_speed_sub = Topic.manual_speed.createSubscriber(self, self._manual_speed_callback)
        self.recorded_path_pub = Topic.recorded_path.createPublisher(self)
        
        self.get_logger().info("Communication setup complete")
    
    def _rc5_callback(self, msg: Float64):
        self.rc5 = msg.data
        self.rc5_state = self._get_channel_state(self.rc5)

    def _rc6_callback(self, msg: Float64):
        self.rc6 = msg.data
        self.rc6_state = self._get_channel_state(self.rc6)

    
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
    
    def read_keyboard_input(self):
        self.simulator_keyboard = input("Enter keyboard input: ")
    
    def _use_simulator_keyboard(self):
        if self.use_simulator:
            self.manual_yaw_effort, self.manual_speed_effort = self.keyboard_input.get_manual_efforts()
            
    
    def calculate_control_efforts(self):
        """
        Calculate control efforts based on RC channel states
        """
        current_time = time()
        self.delta_time = current_time - self.last_time
        self.last_time = current_time
        
        yaw_effort, speed_effort = self._rc5_decision_maker(self.delta_time)
        self._rc6_decision_maker()

        return yaw_effort, speed_effort


    def control_loop(self):
        try:
            if self.use_simulator:
                self._use_simulator_keyboard()

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
