#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from core.utils.config import Topic, NodeConfig
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

import matplotlib.pyplot as plt
from core_control.pid_controller import PIDController

class Waypoint(Enum):
    IDLE = 0
    START = 1
    DOCKING = 2


class Gcs(Enum):
    IDLE = 0
    TAKE_PHOTO_CAMERA_1 = 1
    TAKE_PHOTO_CAMERA_2 = 2


class RecordingState(Enum):
    IDLE = 0
    RECORDING = 1
    RETURNING_TO_START = 2
    PLAYING_BACK = 3


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

        if key == 'w':
            self.manual_speed_effort = 200.0  # Forwards
            self.manual_yaw_effort = 0.0
            self.node.get_logger().info("Forward")

        elif key == 's':
            self.manual_speed_effort = -200.0  # Backward
            self.manual_yaw_effort = 0.0
            self.node.get_logger().info("Backward")

        elif key == 'a':
            self.manual_yaw_effort = -200.0  # Turn left
            self.manual_speed_effort = 0.0
            self.node.get_logger().info("Turn Left")

        elif key == 'd':
            self.manual_yaw_effort = 200.0  # Turn right
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
    def __init__(self):
        super().__init__(NodeConfig.movement_controller)

        self.docking_controller = DockingController()

        self.pid_controller = PIDController(self.docking_controller.Kp_yaw, self.docking_controller.Ki_yaw, self.docking_controller.Kd_yaw)
        self.pid_controller._init_comms()

        self.use_simulator = True
        self.keyboard_input = KeyboardInput(self)
        if self.use_simulator:
            self.keyboard_input.start_keyboard_listener()

        self.init_lat_lon = False
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0  # degrees (Pixhawk convention)
        self.current_speed = 0.0

        self.manual_yaw_effort = 0.0
        self.manual_speed_effort = 0.0

        self.PWM_LOW = 1000
        self.PWM_MID = 1500
        self.PWM_HIGH = 1700

        # Clamping efforts
        self.MAX_PWM = 300.0
        self.MIN_PWM = -300.0

        # RC channels and states
        self.rc5 = 0.0
        self.rc6 = 0.0

        self.kp = self.docking_controller.Kp_yaw
        self.ki = self.docking_controller.Ki_yaw
        self.kd = self.docking_controller.Kd_yaw

        self.rc5_state = 'LOW'
        self.prev_rc5_state = 'LOW'

        self.rc6_state = 'LOW'
        self.prev_rc6_state = 'LOW'

        self.waypoint_state = Waypoint.IDLE
        self.gcs_state = Gcs.IDLE

        self.recording_state = RecordingState.IDLE
        self.playback_index = 0
        self.playback_waypoint_threshold = 2.0  # meters to advance waypoint during playback
        self.has_left_dock = False
        self.initial_record_position = None  # (lat, lon, heading_deg)

        # control timing
        self.control_rate = 50.0
        self.dt = 1.0 / self.control_rate
        self.last_time = time()
        self.delta_time = self.dt

        # publishers/messages
        self.yaw_msg = Float64()
        self.speed_msg = Float64()

        # setup communication
        self._setup_communication()

        # timer will be created by run()
        self.get_logger().info(f"[{NodeConfig.movement_controller}] Successfully initialized")
        self.get_logger().info("Waiting for Pixhawk data...")

    def _setup_communication(self):
        """Initialize ROS2 pubs/subs"""
        self.pixhawk_sub = Topic.pixhawk.createSubscriber(self, self._pixhawk_callback)
        self.pwm_sub = Topic.pwm.createSubscriber(self, self._pwm_callback)

        self.rc5_sub = Topic.rc5.createSubscriber(self, self._rc5_callback)  # for docking + recording toggle
        self.rc6_sub = Topic.rc6.createSubscriber(self, self._rc6_callback)  # for other GCS actions

        self.manual_yaw_sub = Topic.manual_yaw.createSubscriber(self, self._manual_yaw_callback)
        self.manual_speed_sub = Topic.manual_speed.createSubscriber(self, self._manual_speed_callback)

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self)
        self.error_pub = Topic.error.createPublisher(self)
    

        self.get_logger().info("Communication setup complete")

    # ------------------------
    # Callbacks
    # ------------------------
    def _pixhawk_callback(self, msg: Pixhawk):
        """Update current position and heading from Pixhawk"""
        self.current_lat = msg.lat
        self.current_lon = msg.lon
        self.current_speed = getattr(msg, "msg_spd", self.current_speed)
        self.current_heading = getattr(msg, "msg_heading", self.current_heading)

    def _kp_callback(self, msg: Float64):
        """Update docking controller PID kp gain"""
        self.kp = msg.data
        self.docking_controller.Kp_yaw = self.kp
        self.pid_controller.set_gains(self.kp, self.ki, self.kd)

    def _ki_callback(self, msg: Float64):
        """Update docking controller PID ki gain"""
        self.ki = msg.data
        self.docking_controller.Ki_yaw = self.ki
        self.pid_controller.set_gains(self.kp, self.ki, self.kd)
    
    def _kd_callback(self, msg: Float64):
        """Update docking controller PID kd gain"""
        self.kd = msg.data
        self.docking_controller.Kd_yaw = self.kd
        self.pid_controller.set_gains(self.kp, self.ki, self.kd)

    def _pwm_callback(self, msg: Pwm):
        """Store pwm channels if needed"""
        self.pwm_chan = msg.channels

    def _manual_yaw_callback(self, msg: Float64):
        self.manual_yaw_effort = msg.data

    def _manual_speed_callback(self, msg: Float64):
        self.manual_speed_effort = msg.data

    def _get_channel_state(self, pwm_value):
        """Determine RC channel state"""
        if pwm_value < self.PWM_LOW:
            return 'LOW'
        elif pwm_value > self.PWM_HIGH:
            return 'HIGH'
        else:
            return 'MID'

    def _rc5_callback(self, msg: Float64):
        self.rc5 = msg.data
        self.rc5_state = self._get_channel_state(self.rc5)
        
        if self.rc5_state == 'MID' and self.prev_rc5_state != 'MID':
            self._on_rc5_mid_pressed()

        self.prev_rc5_state = self.rc5_state

    def _rc6_callback(self, msg: Float64):
        self.rc6 = msg.data
        self.rc6_state = self._get_channel_state(self.rc6)

        if self.rc6_state == 'MID' and self.prev_rc6_state != 'MID':
            self._on_rc6_mid_pressed()
        
        if self.rc6_state == 'HIGH' and self.prev_rc6_state != 'HIGH':
            self._on_rc6_high_pressed()
        
        self.prev_rc6_state = self.rc6_state
        

    # ------------------------
    # Recording / playback control
    # ------------------------
    def _on_rc5_mid_pressed(self):
        if self.recording_state == RecordingState.IDLE:
            self.docking_controller.set_target(self.current_lat, self.current_lon)
            self.docking_controller.start_lat_lon_recording(self.current_lat, self.current_lon, self.current_heading)
            self.initial_record_position = (self.current_lat, self.current_lon, self.current_heading)
            self.recording_state = RecordingState.RECORDING
            self.has_left_dock = False
            self.playback_index = 0
            self.get_logger().info(f"Recording started at lat={self.current_lat:.8f}, lon={self.current_lon:.8f}")

        elif self.recording_state == RecordingState.PLAYING_BACK:
            self.recording_state = RecordingState.IDLE
            self.playback_index = 0
            self.get_logger().info("Playback stopped by RC5 MID press. Returning to IDLE.")

        else:
            self.get_logger().info(f"RC5 MID pressed while in state {self.recording_state.name}. Ignoring.")

    def _on_rc6_mid_pressed(self):
        self.get_logger().info("Photo 1 command received (RC6 MID)")

    def _on_rc6_high_pressed(self):
        self.get_logger().info("Photo 2 command received (RC6 HIGH)")




    def update_recording(self, dt):
        """Record current lat/lon frames when in RECORDING state."""
        if self.recording_state != RecordingState.RECORDING:
            return

        self.docking_controller.record_lat_lon(self.current_lat, self.current_lon, dt, self.rc6)

        # if no docking target, skip
        if self.docking_controller.target_lat is None or self.docking_controller.target_lon is None:
            return

        distance_to_dock = self.docking_controller.get_distance_to_target(self.current_lat, self.current_lon)

        if not self.has_left_dock and distance_to_dock > self.playback_waypoint_threshold:
            self.has_left_dock = True
            self.get_logger().info("Left docking area — recording path...")

        if self.has_left_dock and distance_to_dock < self.docking_controller.docking_distance_threshold:
            num_frames = self.docking_controller.stop_lat_lon_recording()
            duration = self.docking_controller.get_lat_lon_duration()
            self.get_logger().info(f"Returned to docking point — recording stopped. Frames: {num_frames}, duration: {duration:.2f}s")

            if num_frames > 0:
                self.recording_state = RecordingState.RETURNING_TO_START
                self.playback_index = 0
                self.get_logger().info("Entering RETURNING_TO_START phase.")
            else:
                self.recording_state = RecordingState.IDLE
                self.has_left_dock = False
                self.get_logger().warn("No lat/lon frames recorded; returning to IDLE.")

    def update_return_navigation(self, dt):
        """Use docking_controller to navigate back to the initial recorded position."""
        if self.recording_state != RecordingState.RETURNING_TO_START:
            return

        if not self.initial_record_position:
            self.get_logger().warn("No initial record position saved; aborting return navigation.")
            self.recording_state = RecordingState.IDLE
            return

        init_lat, init_lon, _ = self.initial_record_position
        self.docking_controller.set_target(init_lat, init_lon)

        yaw_effort, speed_effort, is_docked = self.docking_controller.calculate_control_efforts(
            self.current_lat, self.current_lon, self.current_heading, dt
        )

        # if reached initial point -> transition to PLAYING_BACK
        if is_docked:
            self.get_logger().info("Reached initial record point — starting playback.")
            self.recording_state = RecordingState.PLAYING_BACK
            self.playback_index = 0
            return

        # otherwise, set manual efforts so the main control loop will publish them
        # these override keyboard manual control while returning
        self.manual_yaw_effort = yaw_effort
        self.manual_speed_effort = speed_effort

    def update_playback(self, dt):
        """Play back recorded lat/lon waypoints sequentially."""
        if self.recording_state != RecordingState.PLAYING_BACK:
            return

        recorded = self.docking_controller.get_recorded_lat_lon()
        if not recorded or len(recorded) == 0:
            self.get_logger().info("No recorded waypoints for playback. Stopping.")
            self.recording_state = RecordingState.IDLE
            return

        # if finished playback
        if self.playback_index >= len(recorded):
            self.get_logger().info("Playback complete.")
            self.playback_index = 0
            self.recording_state = RecordingState.IDLE
            # stop motors
            self.manual_yaw_effort = 0.0
            self.manual_speed_effort = 0.0
            return

        # use current waypoint as target
        target_lat, target_lon, _, _ = recorded[self.playback_index]
        # only set docking target if changed (to avoid resetting PID unnecessarily)
        if (self.docking_controller.target_lat != target_lat or
                self.docking_controller.target_lon != target_lon):
            self.docking_controller.set_target(target_lat, target_lon)

        # compute efforts toward current waypoint
        yaw_effort, speed_effort, is_docked = self.docking_controller.calculate_control_efforts(
            self.current_lat, self.current_lon, self.current_heading, dt
        )

        # publish as manual efforts (control_loop publishes)
        self.manual_yaw_effort = yaw_effort
        self.manual_speed_effort = speed_effort

        # check distance to waypoint
        distance_to_target = self.docking_controller.get_distance_to_target(self.current_lat, self.current_lon)

        if distance_to_target < self.playback_waypoint_threshold:
            self.playback_index += 1
            # do not immediately overwrite target — next loop iteration will pick new target

    def update_docking(self, dt):
        """Autonomous docking when rc5 is HIGH (Waypoint.DOCKING)."""
        # If rc5 indicates DOCKING mode, use docking controller target (should be set elsewhere)
        if self.rc5_state != 'HIGH':
            return

        # If no docking target set, nothing to do
        if self.docking_controller.target_lat is None or self.docking_controller.target_lon is None:
            return

        yaw_effort, speed_effort, is_docked = self.docking_controller.calculate_control_efforts(
            self.current_lat, self.current_lon, self.current_heading, dt
        )

        if is_docked:
            self.get_logger().info("Autonomous docking complete.")
            # stop
            self.manual_yaw_effort = 0.0
            self.manual_speed_effort = 0.0
            self.is_docked = True
            # remain in docking mode until rc5 changed
        else:
            # apply docking efforts (override manual keyboard while docking)
            self.manual_yaw_effort = yaw_effort
            self.manual_speed_effort = speed_effort

    # ------------------------
    # Main control loop
    # ------------------------
    def _use_simulator_keyboard(self):
        """Read keyboard-simulated efforts (if enabled). These act as default manual efforts."""
        if self.use_simulator:
            kb_yaw, kb_speed = self.keyboard_input.get_manual_efforts()
            # Only apply keyboard if not overridden by autonomous phases.
            # We'll keep them in self.keyboard_* and only use if no auto mode is active.
            self.keyboard_yaw_effort = kb_yaw
            self.keyboard_speed_effort = kb_speed

    def calculate_control_efforts(self):
        """This function is kept for compatibility; main logic runs in control_loop."""
        # returns current manual efforts (not used by autonomous flows)
        return self.manual_yaw_effort, self.manual_speed_effort

    def control_loop(self):
        try:
            # time
            now = time()
            self.delta_time = now - self.last_time if self.last_time else self.dt
            # clamp unrealistic dt
            if self.delta_time <= 0.0 or self.delta_time > 1.0:
                self.delta_time = self.dt
            self.last_time = now

            # update keyboard values
            if self.use_simulator:
                self._use_simulator_keyboard()

            # Run stateful updates in this order:
            # 1) If recording -> append frames and check auto-stop
            # 2) If returning to start -> navigate back
            # 3) If playing back -> perform playback waypoint control
            # 4) If docking by RC5 HIGH -> do autonomous docking

            self.update_recording(self.delta_time)
            self.update_return_navigation(self.delta_time)
            self.update_playback(self.delta_time)
            self.update_docking(self.delta_time)

            # Decide which efforts to publish. Priority (highest -> lowest):
            # RETURNING_TO_START / PLAYING_BACK / DOCKING override manual keyboard
            if self.recording_state == RecordingState.RETURNING_TO_START or \
               self.recording_state == RecordingState.PLAYING_BACK:
                yaw_effort = self.manual_yaw_effort
                speed_effort = self.manual_speed_effort
            elif self.rc5_state == 'HIGH':
                # docking mode enforced by update_docking setting manual_* already
                yaw_effort = self.manual_yaw_effort
                speed_effort = self.manual_speed_effort
            else:
                # use keyboard or external manual topics
                # combine keyboard simulated inputs with external manual inputs
                yaw_effort = getattr(self, "keyboard_yaw_effort") + self.manual_yaw_effort
                speed_effort = getattr(self, "keyboard_speed_effort") + self.manual_speed_effort

            # clamp efforts
            yaw_effort = max(self.MIN_PWM, min(self.MAX_PWM, yaw_effort))
            speed_effort = max(self.MIN_PWM, min(self.MAX_PWM, speed_effort))

            # publish
            self.yaw_msg.data = float(yaw_effort)
            self.speed_msg.data = float(speed_effort)

            self.yaw_effort_pub.publish(self.yaw_msg)
            self.speed_effort_pub.publish(self.speed_msg)
            self.error_pub.publish(Float64(data=self.docking_controller.get_current_error()))
            
            self

        except Exception as e:
            self.get_logger().error(f"Error in control loop: {traceback.format_exc()}")

    def run(self):
        # create timer with same dt as control loop
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
