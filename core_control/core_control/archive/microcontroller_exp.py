#!/usr/bin/env python3

import rclpy
import os
import fnmatch
import serial
import traceback
import math
import sys
import threading
import time
from typing import Optional, List, Tuple
import numpy as np
from pymavlink import mavutil
from pykalman import KalmanFilter
from std_msgs.msg import Float64, UInt8, UInt16, String
from core_msgs.msg import Pwm, AutoControl, KillSwitch, Pixhawk
from core.utils.config import (
    AutoState,
    RemoteState,
    Topic,
    MotorReverse,
    SETPOINT,
    Param,
    PxMode,
)
from adafruit_simplemath import map_range
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Constants
ADS_MAX_VAL = 26096
GAIN_RATIO = 1069 / 1000
OPEN_DRAIN_RATIO = 122 / 22
BAT_MAX_VAL = 16.8
BAT_MIN_VAL = 14.4
PWM_OFFSET = 15
PWM_MIN = 1100
PWM_MAX = 1900
PWM_NEUTRAL = 1500

# Configuration
SERIAL_TIMEOUT = 0.1
MAVLINK_BAUD = 57600
ESP32_BAUD = 115200
MAIN_LOOP_RATE = 10.0  # Hz
HEARTBEAT_RATE = 1.0   # Hz


class MiconType:
    PX = 1
    PICO = 2
    ESP32 = 3
    NOT_IDENTIFIED = 0
    NONE = -1


class SerialConnectionManager:
    """Manages serial connections with automatic reconnection"""
    
    def __init__(self, logger):
        self.logger = logger
        self.esp32_conn: Optional[serial.Serial] = None
        self.pixhawk_conn: Optional[mavutil.mavlink_connection] = None
        self._lock = threading.Lock()
    
    def connect_esp32(self) -> bool:
        """Attempt to connect to ESP32"""
        try:
            devs = os.listdir('/dev')
            acm_devices = [f for f in devs if fnmatch.fnmatch(f, 'ttyACM*')]
            
            if not acm_devices:
                return False
                
            device_path = f"/dev/{acm_devices[0]}"
            with self._lock:
                if self.esp32_conn:
                    self.esp32_conn.close()
                
                self.esp32_conn = serial.Serial(device_path, ESP32_BAUD, timeout=SERIAL_TIMEOUT)
                self.logger.info(f"ESP32 connected on {device_path}")
                return True
                
        except Exception as e:
            self.logger.error(f"Failed to connect ESP32: {e}")
            return False
    
    def connect_pixhawk(self) -> bool:
        """Attempt to connect to Pixhawk"""
        try:
            devs = os.listdir('/dev')
            usb_devices = [f for f in devs if fnmatch.fnmatch(f, 'ttyUSB*')]
            
            if not usb_devices:
                return False
                
            device_path = f"/dev/{usb_devices[0]}"
            with self._lock:
                if self.pixhawk_conn:
                    self.pixhawk_conn.close()
                
                self.pixhawk_conn = mavutil.mavlink_connection(device_path, baud=MAVLINK_BAUD)
                # Handshake
                self.pixhawk_conn.mav.heartbeat_send(0, 0, 0, 0, 0)
                self.pixhawk_conn.wait_heartbeat(timeout=5)
                self.logger.info(f"Pixhawk connected on {device_path}")
                return True
                
        except Exception as e:
            self.logger.error(f"Failed to connect Pixhawk: {e}")
            return False
    
    def is_esp32_connected(self) -> bool:
        with self._lock:
            return self.esp32_conn is not None and self.esp32_conn.is_open
    
    def is_pixhawk_connected(self) -> bool:
        with self._lock:
            return self.pixhawk_conn is not None
    
    def read_esp32_line(self) -> Optional[str]:
        with self._lock:
            if self.esp32_conn and self.esp32_conn.in_waiting:
                try:
                    return self.esp32_conn.readline().decode('utf-8', errors='ignore')
                except Exception:
                    return None
        return None
    
    def get_pixhawk_connection(self):
        with self._lock:
            return self.pixhawk_conn


class SensorDataProcessor:
    """Handles sensor data parsing and filtering"""
    
    def __init__(self, logger):
        self.logger = logger
        
        # Kalman filters
        sensor_variance = 0.1
        self.heading_kf = KalmanFilter(
            transition_matrices=[1],
            observation_matrices=[1],
            initial_state_mean=0,
            observation_covariance=sensor_variance,
            transition_covariance=1e-5,
        )
        
        self.depth_kf = KalmanFilter(
            transition_matrices=[1],
            observation_matrices=[1],
            initial_state_mean=0,
            observation_covariance=sensor_variance,
            transition_covariance=1e-5,
        )
    
    @staticmethod
    def parse_sensor_data(raw_str: str) -> Optional[List[float]]:
        """Parse raw sensor string into float list"""
        try:
            return [float(e) for e in raw_str.replace("\r\n", "").strip().split(",")]
        except (ValueError, AttributeError) as e:
            return None
    
    @staticmethod
    def validate_pwm(pwm: float) -> int:
        """Validate and clamp PWM value"""
        pwm_int = int(pwm)
        return max(PWM_MIN, min(PWM_MAX, pwm_int)) if PWM_MIN <= pwm_int <= PWM_MAX else PWM_NEUTRAL
    
    @staticmethod
    def calculate_battery_percentage(raw_value: float) -> float:
        """Calculate battery percentage from raw ADC value"""
        voltage = (((raw_value / ADS_MAX_VAL) * OPEN_DRAIN_RATIO * 3.3) * GAIN_RATIO)
        percentage = ((voltage - BAT_MIN_VAL) * 100) / (BAT_MAX_VAL - BAT_MIN_VAL)
        return max(0.0, min(100.0, percentage))
    
    def filter_heading(self, heading_deg: float) -> float:
        """Apply Kalman filtering to heading"""
        try:
            filtered_state_means, _ = self.heading_kf.filter([heading_deg])
            return float(filtered_state_means[-1])
        except Exception as e:
            self.logger.warning(f"Heading filter error: {e}")
            return heading_deg


class Microcontroller(Node):
    def __init__(self):
        super().__init__('microcontroller_node')
        
        # Use callback groups for proper threading
        self.callback_group = ReentrantCallbackGroup()
        
        # Core components
        self.connection_manager = SerialConnectionManager(self.get_logger())
        self.sensor_processor = SensorDataProcessor(self.get_logger())
        
        # State variables
        self._initialize_state_variables()
        
        # ROS 2 publishers and subscribers
        self._setup_ros_interfaces()
        
        # Control flags
        self.is_running = True
        self.connection_retry_count = 0
        self.max_retry_attempts = 5
        
        # Timers
        self.main_timer = self.create_timer(
            1.0 / MAIN_LOOP_RATE, 
            self.main_loop_callback,
            callback_group=self.callback_group
        )
        
        self.connection_timer = self.create_timer(
            5.0,  # Check connections every 5 seconds
            self.check_connections_callback,
            callback_group=self.callback_group
        )
        
        # Initialize connections
        self._initialize_connections()
        
        self.get_logger().info("Microcontroller node initialized")
    
    def _initialize_state_variables(self):
        """Initialize all state variables"""
        # Hardware states
        self.mc1 = MiconType.NONE
        self.mc2 = MiconType.NONE
        
        # PWM data
        self.pwm_chan = Pwm()
        
        # Kill switch state
        self.ks_kill_state = KillSwitch()
        self.ks_kill_state.data = KillSwitch.DEFAULT
        
        # Sensor data
        self.jetson_batt = 0.0
        self.motor_batt = 0.0
        self.depth = 0.0
        self.internal_temp_deg = 0.0
        self.tbs_pwm_in = PWM_NEUTRAL
        self.heading_deg = 0.0
        self.mux_state = 0
        self.echosounder_dist = 0.0
        self.echosounder_conf = 1.0
        
        # Pixhawk data
        self.pixhawk = Pixhawk()
        self.pxmode = PxMode.MANUAL
        self.is_armed = False
        
        # Control states
        self.auto_status_gcs = UInt8()
        self.auto_status_gcs.data = AutoState.HARDWARE
        self.auto_status_remote = UInt8()
        self.auto_status_remote.data = RemoteState.TBS_MANUAL
    
    def _setup_ros_interfaces(self):
        """Setup ROS 2 publishers and subscribers"""
        # Publishers
        self.kill_switch_pub = Topic.kill_switch.createPublisher(self)
        self.heading_deg_pub = Topic.heading_deg.createPublisher(self)
        self.auto_status_remote_pub = Topic.auto_status_remote.createPublisher(self)
        self.jetson_batt_pub = Topic.jetson_batt.createPublisher(self)
        self.motor_batt_pub = Topic.motor_batt.createPublisher(self)
        self.mux_state_pub = Topic.mux_state.createPublisher(self)
        self.pixhawk_pub = Topic.pixhawk.createPublisher(self)
        self.pxmode_pub = Topic.pxmode.createPublisher(self)
        
        #new diagnostics publisher
        self.diagnostics_pub = Topic.diagnostics.createPublisher(self)
        
        # Subscribers
        self.pwm_sub = Topic.pwm.createSubscriber(
            self,
            self._pwm_callback,
        )
        
        self.get_logger().info("ROS interfaces initialized")
    
    def _initialize_connections(self):
        """Initialize hardware connections"""
        if self.connection_manager.connect_esp32():
            self.mc1 = MiconType.ESP32
        
        if self.connection_manager.connect_pixhawk():
            self.mc2 = MiconType.PX
            self._arm_pixhawk()
    
    def check_connections_callback(self):
        """Periodically check and restore connections"""
        try:
            if not self.connection_manager.is_esp32_connected():
                self.get_logger().warning("ESP32 disconnected, attempting reconnection")
                if self.connection_manager.connect_esp32():
                    self.mc1 = MiconType.ESP32
                    self.connection_retry_count = 0
                else:
                    self.mc1 = MiconType.NONE
                    self.connection_retry_count += 1
            
            if not self.connection_manager.is_pixhawk_connected():
                self.get_logger().warning("Pixhawk disconnected, attempting reconnection")
                if self.connection_manager.connect_pixhawk():
                    self.mc2 = MiconType.PX
                    self._arm_pixhawk()
                    self.connection_retry_count = 0
                else:
                    self.mc2 = MiconType.NONE
                    self.connection_retry_count += 1
            
            # Publish diagnostics
            self._publish_diagnostics()
            
        except Exception as e:
            self.get_logger().error(f"Connection check error: {e}")
    
    def _publish_diagnostics(self):
        """Publish system diagnostics"""
        diag_msg = String()
        status = {
            'esp32_connected': self.connection_manager.is_esp32_connected(),
            'pixhawk_connected': self.connection_manager.is_pixhawk_connected(),
            'is_armed': self.is_armed,
            'current_mode': self.pxmode,
            'retry_count': self.connection_retry_count
        }
        diag_msg.data = str(status)
        self.diagnostics_pub.publish(diag_msg)
    
    def _pwm_callback(self, pwm_msg: Pwm):
        """Handle incoming PWM commands"""
        try:
            # Validate PWM values
            validated_channels = []
            for channel in pwm_msg.channels:
                validated_channels.append(self.sensor_processor.validate_pwm(channel))
            
            self.pwm_chan.channels = validated_channels
            self.get_logger().debug(f"PWM received: {validated_channels}")
            
        except Exception as e:
            self.get_logger().error(f"PWM callback error: {e}")
    
    def main_loop_callback(self):
        """Main processing loop - handles partial connections gracefully"""
        try:
            # Read ESP32 sensor data (if connected)
            if self.connection_manager.is_esp32_connected():
                self._read_esp32_sensors()
            else:
                self.get_logger().debug("ESP32 not connected - skipping sensor read")
            
            # Read Pixhawk data (if connected)
            if self.connection_manager.is_pixhawk_connected():
                self._read_pixhawk_data()
                self._process_pwm_commands()
            else:
                self.get_logger().debug("Pixhawk not connected - skipping telemetry and PWM")
            
            self._publish_sensor_data()
            
        except Exception as e:
            self.get_logger().error(f"Main loop error: {e}")
            traceback.print_exc()
    
    def _read_esp32_sensors(self):
        """Read and process ESP32 sensor data"""
        raw_line = self.connection_manager.read_esp32_line()
        if not raw_line or not raw_line.startswith('s'):
            return
        
        parsed_data = self.sensor_processor.parse_sensor_data(raw_line[1:])
        if not parsed_data or len(parsed_data) < 10:
            self.get_logger().warning("Invalid ESP32 sensor data")
            return
        
        try:
            (self.jetson_batt, self.motor_batt, depth, ks_state, dht22_raw,
             self.tbs_pwm_in, self.mux_state, heading_deg,
             self.echosounder_dist, self.echosounder_conf) = parsed_data[:10]
            
            self.depth = float(depth)
            self.heading_deg = self.sensor_processor.filter_heading(float(heading_deg))

            if ks_state == 1:
                self.ks_kill_state.data = KillSwitch.KILL
            else:
                self.ks_kill_state.data = KillSwitch.UNKILL
                
        except (ValueError, IndexError) as e:
            self.get_logger().warning(f"Sensor data processing error: {e}")
    
    def _read_pixhawk_data(self):
        """Read Pixhawk telemetry data"""
        px_conn = self.connection_manager.get_pixhawk_connection()
        if not px_conn:
            return
        
        try:
            # Get attitude data
            attitude_msg = px_conn.recv_match(type="ATTITUDE", blocking=False)
            if attitude_msg:
                yaw_deg = math.degrees(attitude_msg.yaw)
                if yaw_deg < 0:
                    yaw_deg += 360
                self.pixhawk.msg_heading = yaw_deg
            
            # Get position data
            pos_msg = px_conn.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
            if pos_msg:
                self.pixhawk.lat = pos_msg.lat / 1e7
                self.pixhawk.lon = pos_msg.lon / 1e7
                self.pixhawk.alt = pos_msg.alt / 1000
            
            # Get VFR HUD data
            vfr_msg = px_conn.recv_match(type="VFR_HUD", blocking=False)
            if vfr_msg:
                self.pixhawk.msg_spd = vfr_msg.groundspeed
                
        except Exception as e:
            self.get_logger().warning(f"Pixhawk data read error: {e}")
    
    def _process_pwm_commands(self):
        """Process and send PWM commands to Pixhawk"""
        px_conn = self.connection_manager.get_pixhawk_connection()
        if not px_conn:
            return
        
        try:
            rc_channels = px_conn.recv_match(type="RC_CHANNELS", blocking=False)
            if not rc_channels:
                return
            self._set_flight_mode(rc_channels.chan8_raw)
            
            # Send PWM commands based on mode
            if rc_channels.chan8_raw > 1700:  # Auto mode
                self._send_auto_pwm_commands(px_conn)
            elif 1301 <= rc_channels.chan8_raw <= 1700:  # Manual mode
                self._send_manual_pwm_commands(px_conn)
            else:
                self.get_logger().debug("PWM commands disabled")
                
        except Exception as e:
            self.get_logger().error(f"PWM processing error: {e}")
    
    def _send_auto_pwm_commands(self, px_conn):
        """Send autonomous PWM commands"""
        for i, pwm_val in enumerate(self.pwm_chan.channels[:7]):  # Limit to 7 channels
            self._set_rc_channel_pwm(px_conn, i + 1, int(pwm_val))
    
    def _send_manual_pwm_commands(self, px_conn):
        """Send manual mode PWM commands (disable autonomous control)"""
        for i in range(7):
            self._set_rc_channel_pwm(px_conn, i + 1, 65535)  # Release control
    
    def _set_rc_channel_pwm(self, px_conn, channel_id: int, pwm: int = PWM_NEUTRAL):
        """Set RC channel PWM value"""
        if channel_id < 1 or channel_id > 8:
            return
        
        try:
            rc_channel_values = [65535] * 8
            rc_channel_values[channel_id - 1] = pwm
            px_conn.mav.rc_channels_override_send(
                px_conn.target_system,
                px_conn.target_component,
                *rc_channel_values
            )
        except Exception as e:
            self.get_logger().error(f"RC channel PWM error: {e}")
    
    def _set_flight_mode(self, pwm_val: int):
        """Set Pixhawk flight mode based on PWM value"""
        px_conn = self.connection_manager.get_pixhawk_connection()
        if not px_conn:
            return
        
        try:
            if pwm_val <= 1300:
                new_mode = PxMode.HOLD
            elif 1301 <= pwm_val <= 1700:
                new_mode = PxMode.MANUAL
            else:
                new_mode = PxMode.MANUAL
            
            if new_mode != self.pxmode:
                self.pxmode = new_mode
                
                if self.pxmode in px_conn.mode_mapping():
                    mode_id = px_conn.mode_mapping()[self.pxmode]
                    px_conn.mav.set_mode_send(
                        px_conn.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mode_id
                    )
                    self.get_logger().info(f"Mode changed to: {self.pxmode}")
                else:
                    self.get_logger().warning(f"Unknown mode: {self.pxmode}")
                    
        except Exception as e:
            self.get_logger().error(f"Mode setting error: {e}")
    
    def _arm_pixhawk(self):
        """Arm Pixhawk motors"""
        px_conn = self.connection_manager.get_pixhawk_connection()
        if not px_conn:
            return
        
        try:
            px_conn.mav.command_long_send(
                px_conn.target_system,
                px_conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            self.get_logger().info("Arming motors...")
            
            # Wait for arming confirmation with timeout
            start_time = time.time()
            while time.time() - start_time < 5.0:  # 5 second timeout
                if px_conn.motors_armed_wait(timeout=1):
                    self.is_armed = True
                    self.get_logger().info("Motors armed successfully")
                    return
            
            self.get_logger().warning("Motor arming timeout")
            
        except Exception as e:
            self.get_logger().error(f"Motor arming error: {e}")
    
    def _publish_sensor_data(self):
        """Publish all sensor data"""
        try:
            # Create and publish messages
            heading_msg = Float64()
            heading_msg.data = self.heading_deg
            
            jetson_batt_msg = UInt16()
            jetson_batt_msg.data = int(self.sensor_processor.calculate_battery_percentage(self.jetson_batt))
            
            motor_batt_msg = UInt16()
            motor_batt_msg.data = int(self.sensor_processor.calculate_battery_percentage(self.motor_batt))
            
            mux_state_msg = UInt8()
            mux_state_msg.data = int(self.mux_state)
            
            pxmode_msg = String()
            pxmode_msg.data = str(self.pxmode)
            
            # Publish all messages
            self.kill_switch_pub.publish(self.ks_kill_state)
            self.heading_deg_pub.publish(heading_msg)
            self.auto_status_remote_pub.publish(self.auto_status_remote)
            self.jetson_batt_pub.publish(jetson_batt_msg)
            self.motor_batt_pub.publish(motor_batt_msg)
            self.mux_state_pub.publish(mux_state_msg)
            self.pixhawk_pub.publish(self.pixhawk)
            self.pxmode_pub.publish(pxmode_msg)
            
        except Exception as e:
            self.get_logger().error(f"Data publishing error: {e}")
    
    def destroy_node(self):
        """Clean shutdown"""
        self.is_running = False
        if hasattr(self.connection_manager, 'esp32_conn') and self.connection_manager.esp32_conn:
            self.connection_manager.esp32_conn.close()
        if hasattr(self.connection_manager, 'pixhawk_conn') and self.connection_manager.pixhawk_conn:
            self.connection_manager.pixhawk_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        micon = Microcontroller()
        executor = MultiThreadedExecutor()
        executor.add_node(micon)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            micon.destroy_node()
            
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()