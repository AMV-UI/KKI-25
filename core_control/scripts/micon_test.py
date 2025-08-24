#!/usr/bin/env python3

import rclpy
import os
import fnmatch
import serial
import traceback
import math
import numpy as np
from pymavlink import mavutil
from pykalman import KalmanFilter
from std_msgs.msg import Float64, UInt8, UInt16
from core_msgs_asv.msg import Pwm, AutoControl, KillSwitch, Pixhawk
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
import time
from rclpy.node import Node
import sys
import random
import threading


# Constants
ADS_MAX_VAL = 26096
GAIN_RATIO = 1069 / 1000
OPEN_DRAIN_RATIO = 122 / 22
BAT_MAX_VAL = 16.8
BAT_MIN_VAL = 14.4

# Reverse map
_reverse_map = MotorReverse.map

# PWM
_pwm_offset = 15


class MiconType:
    PX = 1
    PICO = 2
    ESP32 = 3
    NOT_IDENTIFIED = 0
    NONE = -1


class TestMode:
    """Test modes for different scenarios"""
    DUMMY_DATA = "dummy"
    BATTERY_TEST = "battery"
    PWM_TEST = "pwm"
    IMU_TEST = "imu"
    GPS_TEST = "gps"
    COMPREHENSIVE = "comprehensive"
    STRESS_TEST = "stress"


class MockRCChannels:
    """Mock RC channels for testing"""
    def __init__(self, chan8_raw=1500):
        self.chan1_raw = 1500
        self.chan2_raw = 1500
        self.chan3_raw = 1500
        self.chan4_raw = 1500
        self.chan5_raw = 1500
        self.chan6_raw = 1500
        self.chan7_raw = 1500
        self.chan8_raw = chan8_raw


class MockSerialConnection:
    """Mock serial connection for testing without hardware"""
    def __init__(self):
        self.in_waiting = True
        self.data_buffer = []
        
    def readline(self):
        # Simulate ESP32 sensor data
        test_data = [
            20000,  # jetson_batt
            18000,  # motor_batt
            5.0,    # depth
            1,      # ks_state
            300,    # dht22_raw
            1500,   # tbs_pwm_in
            1,      # mux_state
            random.uniform(0, 360),  # heading_deg
            2.5,    # echosounder_dist
            3.0     # echosounder_conf
        ]
        data_str = "s" + ",".join(map(str, test_data)) + "\r\n"
        return data_str.encode()
        
    def decode(self):
        return self


class MockMavlinkConnection:
    """Mock MAVLink connection for testing"""
    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.mav = self
        self._armed = False
        
    def heartbeat_send(self, *args):
        pass
        
    def wait_heartbeat(self):
        pass
        
    def motors_armed_wait(self):
        self._armed = True
        
    def command_long_send(self, *args):
        pass
        
    def rc_channels_override_send(self, *args):
        pass
        
    def param_request_read_send(self, *args):
        pass
        
    def set_mode_send(self, *args):
        pass
        
    def mode_mapping(self):
        return {
            "MANUAL": 0,
            "HOLD": 1,
            "AUTO": 2
        }
        
    def recv_match(self, type=None, blocking=False):
        """Mock message receiving"""
        if type == "RC_CHANNELS":
            return MockRCChannels(random.choice([1200, 1500, 1800]))
        elif type == "GLOBAL_POSITION_INT":
            mock_msg = type('GPSMsg', (object,), {
                'lat': int(-6.2000 * 1e7),  # Jakarta coordinates
                'lon': int(106.8000 * 1e7),
                'alt': int(10 * 1000)  # 10 meters
            })()
            return mock_msg
        elif type == "VFR_HUD":
            mock_msg = type('VFRMsg', (object,), {
                'heading': random.uniform(0, 360),
                'groundspeed': random.uniform(0, 5)
            })()
            return mock_msg
        elif type == "ATTITUDE":
            mock_msg = type('AttitudeMsg', (object,), {
                'yaw': math.radians(random.uniform(0, 360))
            })()
            return mock_msg
        return None


class Microcontroller(Node):
    def __init__(self, test_mode=None):
        super().__init__('Microcontroller')
        self.test_mode = test_mode
        self.test_active = test_mode is not None
        
        # Initialize all your existing attributes
        self.pwm_chan = Pwm()
        self.mc1 = MiconType.NONE
        self.mc2 = MiconType.NONE
        self.imu = None

        # States from pico
        self.ks_kill_state = KillSwitch()
        self.ks_kill_state.data = KillSwitch.DEFAULT

        self.pxmode = "MANUAL"

        self.jetson_batt = 0
        self.motor_batt = 0
        self.depth = 0
        self.dht22_raw = 0
        self.internal_temp_deg = 0.0
        self.tbs_pwm_in = 0
        self.heading_deg = 0.0
        self.mux_state = 0

        self.echosounder_dist = 0
        self.echosounder_conf = 1

        self.auto_control = AutoControl()

        # GCS <> Micon
        self.auto_status_gcs = UInt8()
        self.auto_status_gcs.data = AutoState.HARDWARE
        self.auto_status_remote = UInt8()
        self.auto_status_remote.data = RemoteState.TBS_MANUAL
        self.pixhawk = Pixhawk()

        # Kalman filters
        sensor_variance = 0.1
        self.kf = KalmanFilter(
            transition_matrices=[1],
            observation_matrices=[1],
            initial_state_mean=0,
            observation_covariance=sensor_variance,
            transition_covariance=1e-5,
        )

        self.kf_imu = KalmanFilter(
            transition_matrices=[1],
            observation_matrices=[1],
            initial_state_mean=0,
            observation_covariance=sensor_variance,
            transition_covariance=1e-5,
        )

        # Test statistics
        self.test_stats = {
            'messages_published': 0,
            'pwm_commands_sent': 0,
            'errors_encountered': 0,
            'start_time': time.time()
        }

    def _init_mc(self):
        """Initialize microcontroller connections (or mock them for testing)"""
        if self.test_active:
            self._init_mock_connections()
            return
            
        # Original connection logic
        devs = os.listdir('/dev')
        acm = [f for f in devs if fnmatch.fnmatch(f, 'ttyACM*')]
        usb = [f for f in devs if fnmatch.fnmatch(f, 'ttyUSB*')]

        if acm:
            try:
                self.ser_1 = serial.Serial(f"/dev/{acm[0]}", 115200, timeout=0.1)
                self.mc1 = MiconType.ESP32
            except Exception:
                self.get_logger().error("Failed to open ESP32 serial port")
        else:
            self.mc1 = MiconType.NONE
            self.get_logger().info("No ttyACM* connection")

        if usb:
            try:
                self.ser_2 = mavutil.mavlink_connection(f"/dev/{usb[0]}", baud=57600)
                self.ser_2.mav.heartbeat_send(0, 0, 0, 0, 0)
                self.ser_2.wait_heartbeat()
                self._px_arm()
                self.mc2 = MiconType.PX
            except Exception:
                self.get_logger().error("Failed to open Pixhawk MAVLink port")
        else:
            self.mc2 = MiconType.NONE
            self.get_logger().info("No ttyUSB* connection")

    def _init_mock_connections(self):
        """Initialize mock connections for testing"""
        self.ser_1 = MockSerialConnection()
        self.ser_2 = MockMavlinkConnection()
        self.mc1 = MiconType.ESP32
        self.mc2 = MiconType.PX
        self.get_logger().info(f"Mock connections initialized for test mode: {self.test_mode}")

    @staticmethod
    def _parse_raw(raw_str):
        try:
            return [float(e) for e in raw_str.replace("\r\n", "").split(",")]
        except Exception as e:
            # Remove inappropriate comment and use proper logging
            return None

    @staticmethod
    def _validate(pwm):
        if pwm < 1100 or pwm > 1900:
            return 1500
        return pwm

    def _read_sensor_esp32(self):
        """Read sensor data from ESP32 (or generate test data)"""
        if self.test_active:
            return self._generate_test_sensor_data()
            
        parsed_data = None

        if hasattr(self.ser_1, 'in_waiting') and self.ser_1.in_waiting:
            raw_ser_1 = self.ser_1.readline().decode()

            if raw_ser_1 and raw_ser_1[0] == "s":
                self.mc1 = MiconType.ESP32
                self.mc2 = MiconType.PX
                parsed_data = self._parse_raw(raw_ser_1[1:])

        if not parsed_data:
            return None

        (
            self.jetson_batt,
            self.motor_batt,
            self.depth,
            ks_state,
            self.dht22_raw,
            self.tbs_pwm_in,
            self.mux_state,
            self.heading_deg,
            self.echosounder_dist,
            self.echosounder_conf,
        ) = parsed_data

        return True

    def _generate_test_sensor_data(self):
        """Generate test sensor data based on test mode"""
        if self.test_mode == TestMode.BATTERY_TEST:
            # Test battery levels from full to empty
            cycle_time = (time.time() - self.test_stats['start_time']) % 20
            battery_level = 26096 - (cycle_time / 20) * 10000
            self.jetson_batt = max(battery_level, 16000)
            self.motor_batt = max(battery_level - 1000, 15000)
            
        elif self.test_mode == TestMode.IMU_TEST:
            # Test IMU with circular motion
            cycle_time = time.time() - self.test_stats['start_time']
            self.heading_deg = (cycle_time * 30) % 360  # 30 degrees per second
            
        elif self.test_mode == TestMode.STRESS_TEST:
            # Rapid random changes
            self.jetson_batt = random.uniform(16000, 26000)
            self.motor_batt = random.uniform(15000, 25000)
            self.heading_deg = random.uniform(0, 360)
            self.depth = random.uniform(0, 10)
            
        else:  # Default dummy data
            self.jetson_batt = 20000
            self.motor_batt = 18000
            self.depth = 5
            self.heading_deg = 90.0
            
        # Common test data
        self.dht22_raw = 300
        self.internal_temp_deg = 25.0
        self.tbs_pwm_in = 1500
        self.mux_state = 1
        self.echosounder_dist = 2.5
        self.echosounder_conf = 3.0
        
        return True

    def _pwm_callback(self, pwm_msg):
        self.pwm_chan = pwm_msg

    @staticmethod
    def _reverse_pwm(pwm_list):
        ret_list = list(pwm_list)
        for idx, val in enumerate(ret_list):
            if idx in _reverse_map and val > 1500:
                ret_list[idx] = 1500 - abs(val - 1500)
            elif idx in _reverse_map and val < 1500:
                ret_list[idx] = 1500 + abs(val - 1500)
        return ret_list

    @staticmethod
    def _battery_value(bat_val):
        return (
            (
                (((bat_val / ADS_MAX_VAL) * (OPEN_DRAIN_RATIO) * 3.3) * (GAIN_RATIO)) - (BAT_MIN_VAL)
            ) * 100
        ) / (BAT_MAX_VAL - BAT_MIN_VAL)

    def _get_filtered_heading(self, heading_deg):
        filtered_state_means, _ = self.kf_imu.filter(heading_deg)
        return heading_deg, filtered_state_means

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        """Set RC channel pwm value"""
        if channel_id < 1:
            self.get_logger().error_throttle(5000, "PWM Channel does not exist")
            return

        if channel_id < 9:
            rc_channel_values = [65535 for _ in range(8)]
            rc_channel_values[channel_id - 1] = pwm
            if hasattr(self.ser_2, 'mav'):
                self.ser_2.mav.rc_channels_override_send(
                    self.ser_2.target_system,
                    self.ser_2.target_component,
                    *rc_channel_values,
                )
                self.test_stats['pwm_commands_sent'] += 1

    def _px_rc_val(self):
        """Get RC channel values (or mock them for testing)"""
        if self.test_active:
            if self.test_mode == TestMode.PWM_TEST:
                # Cycle through different PWM values
                cycle_time = int(time.time() - self.test_stats['start_time']) % 3
                pwm_values = [1200, 1500, 1800]
                return MockRCChannels(pwm_values[cycle_time])
            else:
                return MockRCChannels(1800)  # Default to auto mode
                
        rc_channels = self.ser_2.recv_match(type="RC_CHANNELS", blocking=True)
        if not rc_channels:
            self.get_logger().info("No message in RC_CHANNELS")
            return None
        return rc_channels

    def _send_pwm(self, rc_channels):
        """Send PWM commands"""
        pwm_count = 1

        if rc_channels.chan8_raw > 1700:
            try:
                for pwm_val in self.pwm_chan.channels:
                    if pwm_count >= 8:
                        break
                    self.set_rc_channel_pwm(pwm_count, pwm=int(pwm_val))
                    if self.test_active:
                        self.get_logger().info(f"Test PWM sent: Channel {pwm_count}, Value {pwm_val}")
                    pwm_count += 1
            except Exception as e:
                self.get_logger().error(f"PWM Channel cannot pass. Error: {e}")
                self.test_stats['errors_encountered'] += 1

        elif 1301 <= rc_channels.chan8_raw <= 1700:
            try:
                for pwm_val in self.pwm_chan.channels:
                    if pwm_count >= 8:
                        break
                    self.set_rc_channel_pwm(pwm_count, 65535)
                    pwm_count += 1
            except Exception as e:
                self.get_logger().error(f"Manual Mode: PWM Channel cannot pass. Error: {e}")
                self.test_stats['errors_encountered'] += 1

    def auto_status_gcs_cb(self, msg):
        self.auto_status_gcs.data = msg.data

    def _px_arm(self):
        """Arm Pixhawk motors (or simulate for testing)"""
        if hasattr(self.ser_2, 'mav'):
            self.ser_2.mav.command_long_send(
                self.ser_2.target_system,
                self.ser_2.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0,
            )
        self.get_logger().info("Arming motors ...")
        
        if hasattr(self.ser_2, 'motors_armed_wait'):
            self.ser_2.motors_armed_wait()
        self.get_logger().info("Motor Armed!")

    def _px_set_mode(self, pwm_val):
        """Set Pixhawk mode based on PWM value"""
        if pwm_val <= 1300:
            self.pxmode = PxMode.HOLD
        elif 1301 <= pwm_val <= 1700:
            self.pxmode = PxMode.MANUAL
        else:
            self.pxmode = PxMode.MANUAL

        if self.test_active:
            self.get_logger().info(f"Test Mode Set: {self.pxmode} (PWM: {pwm_val})")

        if hasattr(self.ser_2, 'mode_mapping') and self.pxmode in self.ser_2.mode_mapping():
            mode_id = self.ser_2.mode_mapping()[self.pxmode]
            if hasattr(self.ser_2, 'mav'):
                self.ser_2.mav.set_mode_send(
                    self.ser_2.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id,
                )
        return True

    def _request_px(self, timeout=1.0):
        """Request Pixhawk position data (or generate test data)"""
        if self.test_active:
            return self._generate_test_px_data()
            
        # Original implementation with timeout and error handling
        start_time = time.time()
        
        try:
            msg_coor = None
            while (time.time() - start_time) < timeout:
                msg_coor = self.ser_2.recv_match(
                    type="GLOBAL_POSITION_INT", blocking=False
                )
                if msg_coor:
                    break
                time.sleep(0.01)
                
            if not msg_coor:
                return None
                
            alignment = None
            remaining_time = timeout - (time.time() - start_time)
            if remaining_time > 0:
                start_vfr = time.time()
                while (time.time() - start_vfr) < remaining_time:
                    alignment = self.ser_2.recv_match(type="VFR_HUD", blocking=False)
                    if alignment:
                        break
                    time.sleep(0.01)
                    
            if not alignment:
                return None
                
            lat = msg_coor.lat / 1e7
            lon = msg_coor.lon / 1e7
            
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                return None
                
            self.pixhawk.lat = lat
            self.pixhawk.lon = lon
            self.pixhawk.alt = msg_coor.alt / 1000.0
            self.pixhawk.msg_heading = alignment.heading
            self.pixhawk.msg_spd = alignment.groundspeed
            
            return self.pixhawk
            
        except Exception as e:
            self.get_logger().error(f"Error requesting Pixhawk data: {e}")
            return None

    def _generate_test_px_data(self):
        """Generate test Pixhawk data"""
        if self.test_mode == TestMode.GPS_TEST:
            # Simulate GPS movement
            cycle_time = time.time() - self.test_stats['start_time']
            self.pixhawk.lat = -6.2000 + 0.0001 * math.sin(cycle_time * 0.1)
            self.pixhawk.lon = 106.8000 + 0.0001 * math.cos(cycle_time * 0.1)
        else:
            # Static GPS coordinates (Jakarta)
            self.pixhawk.lat = -6.2000
            self.pixhawk.lon = 106.8000
            
        self.pixhawk.alt = 10.0
        self.pixhawk.msg_heading = random.uniform(0, 360)
        self.pixhawk.msg_spd = random.uniform(0, 5)
        
        return self.pixhawk

    def _request_px_imu(self, timeout=0.5):
        """Request Pixhawk IMU data (or generate test data)"""
        if self.test_active:
            # Use the heading from sensor data
            self.imu = self.heading_deg
            return self.imu
            
        # Original implementation
        start_time = time.time()
        
        try:
            msg = None
            while (time.time() - start_time) < timeout:
                msg = self.ser_2.recv_match(type="ATTITUDE", blocking=False)
                if msg:
                    break
                time.sleep(0.01)
                
            if not msg:
                return None
                
            yaw_deg = math.degrees(msg.yaw)
            
            if yaw_deg < 0:
                yaw_deg += 360
            elif yaw_deg >= 360:
                yaw_deg -= 360
                
            if not (0 <= yaw_deg <= 360):
                return None
                
            self.imu = yaw_deg
            return self.imu
            
        except Exception as e:
            self.get_logger().error(f"Error requesting IMU data: {e}")
            return None

    def _init_compass_calibration(self):
        """Initialize compass calibration"""
        try:
            if hasattr(self.ser_2, 'mav'):
                self.ser_2.mav.param_request_read_send(
                    self.ser_2.target_system,
                    self.ser_2.target_component,
                    b"COMPASS_OFS_X",
                    -1,
                )
            self.get_logger().info("Compass calibration parameters requested")
        except Exception as e:
            self.get_logger().error(f"Error requesting compass calibration: {e}")

    def _update_pixhawk_data(self):
        """Non-blocking update of Pixhawk data"""
        if not hasattr(self, '_last_px_update') or (time.time() - self._last_px_update) > 0.2:
            px_data = self._request_px(timeout=0.1)
            if px_data:
                self._last_px_update = time.time()
                
        if not hasattr(self, '_last_imu_update') or (time.time() - self._last_imu_update) > 0.05:
            imu_data = self._request_px_imu(timeout=0.05)
            if imu_data is not None:
                self._last_imu_update = time.time()
                
        return self.pixhawk, self.imu

    def print_test_stats(self):
        """Print test statistics"""
        runtime = time.time() - self.test_stats['start_time']
        self.get_logger().info("=" * 50)
        self.get_logger().info("TEST STATISTICS")
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Test Mode: {self.test_mode}")
        self.get_logger().info(f"Runtime: {runtime:.1f} seconds")
        self.get_logger().info(f"Messages Published: {self.test_stats['messages_published']}")
        self.get_logger().info(f"PWM Commands Sent: {self.test_stats['pwm_commands_sent']}")
        self.get_logger().info(f"Errors Encountered: {self.test_stats['errors_encountered']}")
        self.get_logger().info(f"Publish Rate: {self.test_stats['messages_published']/runtime:.1f} Hz")
        self.get_logger().info("=" * 50)

    def main(self):
        """Main initialization method"""
        # Initialize microcontrollers (or mocks for testing)
        self._init_mc()

        # Initialize message objects
        self.msg_heading_msg = Float64()
        self.jetson_batt_msg = UInt16()
        self.motor_batt_msg = UInt16()
        self.mux_state_msg = UInt8()
        self.imu_msg = Float64()

        # Publishers
        self.kill_switch_pub = Topic.kill_switch.createPublisher(self)
        self.heading_deg_pub = Topic.heading_deg.createPublisher(self)
        self.auto_status_remote_pub = Topic.auto_status_remote.createPublisher(self)
        self.jetson_batt_pub = Topic.jetson_batt.createPublisher(self)
        self.motor_batt_pub = Topic.motor_batt.createPublisher(self)
        self.mux_state_pub = Topic.mux_state.createPublisher(self)
        self.pixhawk_pub = Topic.pixhawk.createPublisher(self)
        self.pxmode_pub = Topic.pxmode.createPublisher(self)

        # Subscribers
        self.pwm_sub = Topic.pwm.createSubscriber(self, self._pwm_callback)

        # Initialize test PWM data if in test mode
        if self.test_active:
            test_pwm = Pwm()
            test_pwm.channels = [1600, 1500, 1700, 1600, 1500, 1700, 1500, 1500]
            self.pwm_chan = test_pwm

        # Initialize compass calibration
        if not self.test_active:
            self._init_compass_calibration()
        
        # Create timer for main loop
        self.timer = self.create_timer(0.02, self._main_loop_callback)  # 50Hz
        
        if self.test_active:
            # Create timer to print test stats every 10 seconds
            self.stats_timer = self.create_timer(10.0, self.print_test_stats)
        
        self.get_logger().info(f"Microcontroller node started {'(TEST MODE)' if self.test_active else ''}")

    def _main_loop_callback(self):
        """Main loop callback executed at 50Hz"""
        try:
            if not self.test_active and (self.mc1 == MiconType.NONE or 
                                       self.mc2 == MiconType.NONE or 
                                       not hasattr(self, 'ser_2')):
                self.get_logger().error("micon not found")
                self._init_mc()
                return
                
            # Update Pixhawk data
            pixhawk_data, imu_data = self._update_pixhawk_data()
            
            # Update sensor data
            sensor_data = self._read_sensor_esp32()
            
            # Prepare and publish messages
            if pixhawk_data:
                self.msg_heading_msg.data = float(self.pixhawk.msg_heading)
                self.pixhawk_pub.publish(pixhawk_data)
                self.test_stats['messages_published'] += 1
                
            if imu_data is not None:
                self.imu_msg.data, _ = self._get_filtered_heading(imu_data)
                self.heading_deg_pub.publish(self.imu_msg)
                self.test_stats['messages_published'] += 1
                
            if sensor_data:
                self.jetson_batt_msg.data = int(self._battery_value(self.jetson_batt))
                self.motor_batt_msg.data = int(self._battery_value(self.motor_batt))
                self.mux_state_msg.data = int(self.mux_state)

                self.kill_switch_pub.publish(self.ks_kill_state)
                self.jetson_batt_pub.publish(self.jetson_batt_msg)
                self.motor_batt_pub.publish(self.motor_batt_msg)
                self.mux_state_pub.publish(self.mux_state_msg)
                self.test_stats['messages_published'] += 4

            # Handle PWM
            rc_chans = self._px_rc_val()
            if rc_chans:
                self._px_set_mode(rc_chans.chan8_raw)
                self._send_pwm(rc_chans)
                
        except Exception as e:
            self.get_logger().error(f"Error in main loop: {e}")
            self.test_stats['errors_encountered'] += 1


# Test functions for easy usage
def run_test(test_mode=TestMode.DUMMY_DATA, duration=30):
    """
    Run a specific test for a given duration
    
    Args:
        test_mode (str): Test mode from TestMode class
        duration (int): Test duration in seconds
    """
    rclpy.init()
    
    try:
        # Create microcontroller node in test mode
        mc_node = Microcontroller(test_mode=test_mode)
        mc_node.main()
        
        print(f"Starting {test_mode} test for {duration} seconds...")
        
        # Create a timer to stop the test
        def stop_test():
            mc_node.print_test_stats()
            rclpy.shutdown()
        
        timer = threading.Timer(duration, stop_test)
        timer.start()
        
        # Spin the node
        rclpy.spin(mc_node)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed with error: {e}")
    finally:
        try:
            mc_node.destroy_node()
        except:
            pass
        if rclpy.ok():
            rclpy.shutdown()


def run_comprehensive_test():
    """Run all test modes sequentially"""
    test_modes = [
        (TestMode.DUMMY_DATA, 10),
        (TestMode.BATTERY_TEST, 15),
        (TestMode.IMU_TEST, 15),
        (TestMode.GPS_TEST, 15),
        (TestMode.PWM_TEST, 10),
        (TestMode.STRESS_TEST, 10)
    ]
    
    print("Starting comprehensive test suite...")
    
    for test_mode, duration in test_modes:
        print(f"\n{'='*60}")
        print(f"Running {test_mode.upper()} test...")
        print(f"{'='*60}")
        
        try:
            run_test(test_mode, duration)
            time.sleep(2)  # Brief pause between tests
        except Exception as e:
            print(f"Test {test_mode} failed: {e}")
            continue
    
    print("\nComprehensive test suite completed!")


def run_quick_test():
    """Run a quick 10-second test with dummy data"""
    print("Running quick test...")
    run_test(TestMode.DUMMY_DATA, 10)


def run_battery_test():
    """Run battery level test"""
    print("Running battery test...")
    run_test(TestMode.BATTERY_TEST, 20)


def run_imu_test():
    """Run IMU rotation test"""
    print("Running IMU test...")
    run_test(TestMode.IMU_TEST, 20)


def run_gps_test():
    """Run GPS movement test"""
    print("Running GPS test...")
    run_test(TestMode.GPS_TEST, 20)


def run_pwm_test():
    """Run PWM switching test"""
    print("Running PWM test...")
    run_test(TestMode.PWM_TEST, 20)


def run_stress_test():
    """Run stress test with rapid changes"""
    print("Running stress test...")
    run_test(TestMode.STRESS_TEST, 30)


def main(args=None):
    """Main entry point - runs in normal mode unless test arguments provided"""
    if args is None:
        args = sys.argv[1:]
    
    # Check for test arguments
    if len(args) > 0:
        test_arg = args[0].lower()
        
        if test_arg == "quick":
            run_quick_test()
        elif test_arg == "battery":
            run_battery_test()
        elif test_arg == "imu":
            run_imu_test()
        elif test_arg == "gps":
            run_gps_test()
        elif test_arg == "pwm":
            run_pwm_test()
        elif test_arg == "stress":
            run_stress_test()
        elif test_arg == "comprehensive":
            run_comprehensive_test()
        elif test_arg == "help":
            print("Available test modes:")
            print("  quick        - Quick 10-second test")
            print("  battery      - Battery level cycling test")
            print("  imu          - IMU rotation test")
            print("  gps          - GPS movement test")
            print("  pwm          - PWM mode switching test")
            print("  stress       - Stress test with rapid changes")
            print("  comprehensive- Run all tests sequentially")
            print("  help         - Show this help message")
            print("\nUsage: python3 microcontroller.py [test_mode]")
            print("       python3 microcontroller.py           (normal mode)")
        else:
            print(f"Unknown test mode: {test_arg}")
            print("Use 'help' to see available test modes")
        
        return
    
    # Normal mode - no test arguments
    rclpy.init(args=args)
    
    try:
        microcontroller_node = Microcontroller()
        microcontroller_node.main()
        rclpy.spin(microcontroller_node)
        
    except KeyboardInterrupt:
        print("\nNode interrupted by user")
    except Exception as e:
        print(f"Node failed with error: {e}")
    finally:
        try:
            microcontroller_node.destroy_node()
        except:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()