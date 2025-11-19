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
from std_msgs.msg import Float64, UInt8, UInt16, String
from core_msgs.msg import Pwm, AutoControl, KillSwitch, Pixhawk

from core.utils.config import (
    Topic,
    MotorReverse,
    PxMode,
    NodeConfig
)

from adafruit_simplemath import map_range
import time
from rclpy.node import Node
import sys


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


class Microcontroller(Node):
    def __init__(self):
        super().__init__(NodeConfig.microcontroller)
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
        self.internal_temp_deg = 0
        self.tbs_pwm_in = 0
        self.heading_deg = 0
        self.mux_state = 0

        self.echosounder_dist = 0
        self.echosounder_conf = 0

        # GCS <> Micon
        self.pixhawk = Pixhawk()

        sensor_variance = 0.1  # Sensor variance

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

        self.pwm_sub = Topic.pwm.createSubscriber(self, self._pwm_callback)
        self.get_logger().info("<> PWM Subscriber created")

    def warn_once(self, msg):
        if not hasattr(self, '_warn_once_messages'):
            self._warn_once_messages = set()
        if msg not in self._warn_once_messages:
            self.get_logger().warn(msg)
            self._warn_once_messages.add(msg)
    
    def info_once(self, msg):
        if not hasattr(self, '_info_once_messages'):
            self._info_once_messages = set()
        if msg not in self._info_once_messages:
            self.get_logger().info(msg)
            self._info_once_messages.add(msg)
            
    def warn_throttle(self, period_ms, msg):
        self.get_logger().warn(msg, throttle_duration_sec=period_ms/1000.0)
        
    def error_throttle(self, period_ms, msg):
        self.get_logger().error(msg, throttle_duration_sec=period_ms/1000.0)
        
    def info_throttle(self, period_ms, msg):
        self.get_logger().info(msg, throttle_duration_sec=period_ms/1000.0)

    def _init_mc(self):
        dirs = self._get_micon_dir()
        #self.get_logger().info(f"Detected serial ports: {dirs}")
        #
        try:
            if dirs[0] != "/dev/ttyUSB0":
                esp32_port, px_port = dirs[0], dirs[1]
            else:
                esp32_port, px_port = dirs[1], dirs[0]

            self.get_logger().info(f"[MICON] ESP: {esp32_port}, PX: {px_port}")

            #ESP32
            self.ser_1 = serial.Serial(esp32_port, 115200)
            self.mc1 = MiconType.ESP32
            self.get_logger().info(f"[MICON] ESP Successfully initiated")

            #Pixhawk
            self.ser_2 = mavutil.mavlink_connection(px_port, baud=57600)
            self.ser_2.wait_heartbeat()
            self.get_logger().info("Waiting for Pixhawk heartbeat...")
            self.ser_2.mav.heartbeat_send(0, 0, 0, 0, 0)
            self.get_logger().info("Pixhawk Successfully initiated")
            self._px_arm()
            self.rc_chans = self._px_rc_val()

            self.mc2 = MiconType.PX

        except Exception as e:
            self.error_throttle(5000, f"MC Index is not found: {e}")
            self.mc1 = MiconType.NONE
            self.mc2 = MiconType.NONE

    def _init_mc_without_esp(self):
        dirs = self._get_micon_dir()
        
        self.get_logger().info(f"Detected serial ports: {dirs}")
  
        px_port = dirs[0]

        # #ESP32
        # self.ser_1 = serial.Serial(esp32_port, 115200)
        # self.mc1 = MiconType.ESP32

        #Pixhawk
        self.ser_2 = mavutil.mavlink_connection(px_port, baud=57600)
        self.get_logger().info("Waiting for Pixhawk heartbeat...")
        self.ser_2.mav.heartbeat_send(0, 0, 0, 0, 0)
        self.ser_2.wait_heartbeat()
        self.get_logger().info("Pixhawk Successfully initiated")
        self._px_arm()
        self.mc2 = MiconType.PX

    def _get_micon_dir(self):
        dirs = []
        list_of_files = os.listdir("/dev")
        pattern = "ttyACM*"
        for entry in list_of_files:
            if fnmatch.fnmatch(entry, pattern):
                dirs.append(f"/dev/{entry}")

        pattern = "ttyUSB*"
        for entry in list_of_files:
            if fnmatch.fnmatch(entry, pattern):
                dirs.append(f"/dev/{entry}")

        return dirs

    @staticmethod
    def _parse_raw(raw_str):
        try:
            return [float(e) for e in raw_str.replace("\r\n", "").split(",")]
        except Exception as e:
            print(e)
            print("imgay") #legacy angkatan 22
            return None

    @staticmethod
    def _validate(pwm):
        if pwm < 1100 or pwm > 1900:
            return 1500
        return pwm

    def _read_sensor_esp32(self):
        parsed_data = None

        # TODO : do this
        if self.ser_1.in_waiting:
            raw_ser_1 = self.ser_1.readline().decode()

            if raw_ser_1[0] == "s":
                self.mc1 = MiconType.ESP32
                self.mc2 = MiconType.PX
                parsed_data = self._parse_raw(raw_ser_1[1:])

        if not parsed_data:
            return None

        (
            self.jetson_batt,
            self.motor_batt,
            depth,
            ks_state,
            self.dht22_raw,
            self.tbs_pwm_in,
            self.mux_state,
            heading_deg,
            self.echosounder_dist,
            self.echosounder_conf,
        ) = parsed_data

        return True

    def _pwm_callback(self, pwm_msg):
        pass
        # self.pwm_chan = pwm_msg

    @staticmethod
    def _reverse_pwm(pwm_list):
        ret_list = list(pwm_list)
        for idx, val in enumerate(ret_list):
            if idx in _reverse_map and val > 1500:
                ret_list[idx] = 1500 - abs(val - 1500)
            elif idx in _reverse_map and val < 1500:
                ret_list[idx] = 1500 + abs(val - 1500)

        return ret_list

    def _battery_value_safe(self, bat_val):
        val = self._battery_value(bat_val)
        val = max(0, min(int(val), 65535))  # clamp to UInt16
        return val

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

    def _px_rc_val(self):
        rc_channels = self.ser_2.recv_match(type="RC_CHANNELS", blocking=True)
        if not rc_channels:
            self.error_throttle(5000, "No message in RC_CHANNELS")
            return
        return rc_channels

    def _get_pwm(self):
        rc_channels = self.ser_2.recv_match(type="RC_CHANNELS", blocking=True)
        self.info_throttle(2000, f"Channel Values : {rc_channels.chan1_raw}, {rc_channels.chan3_raw}, {rc_channels.chan8_raw}")

    def _px_arm(self):
        self.ser_2.mav.command_long_send(
            self.ser_2.target_system,
            self.ser_2.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.error_throttle(5000, "Arming motors ...")
        self.ser_2.motors_armed_wait()
        self.get_logger().warn("Motor Armed!")

    def _px_set_mode(self, pwm_val):
        if pwm_val <= 1300:
            self.pxmode = PxMode.HOLD
        elif 1301 <= pwm_val <= 1700:
            self.pxmode = PxMode.MANUAL
        else:
            self.pxmode = PxMode.AUTO

        self.get_logger().info(f"Current Mode : {self.pxmode}")
        
        pxmode_msg = String()
        pxmode_msg.data = self.pxmode
        self.pxmode_pub.publish(pxmode_msg)       
        if self.pxmode not in self.ser_2.mode_mapping():
            self.get_logger().warn(f"Unknown Mode : {self.pxmode}")
            return

        mode_id = self.ser_2.mode_mapping()[self.pxmode]
        self.ser_2.mav.set_mode_send(
            self.ser_2.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        self.get_logger().info(f"Mode set to : {self.pxmode}")
        return True

    def request_pixhawk(self):
        try:
            msg_coor = self.ser_2.recv_match(
                type="GLOBAL_POSITION_INT", blocking=True
            )
            lat = msg_coor.lat / 1e7
            lon = msg_coor.lon / 1e7
            alt = (
                msg_coor.alt / 1000
            ) 

            alignment = self.ser_2.recv_match(type="VFR_HUD", blocking=True)

            msg_spd = alignment.groundspeed  # Ground speed in m/s
            msg_heading = alignment.heading

            self.pixhawk.lat = lat
            self.pixhawk.lon = lon
            self.pixhawk.alt = alt
            self.pixhawk.msg_heading = msg_heading
            self.pixhawk.msg_spd = msg_spd

            return self.pixhawk

        except Exception as error:
            self.get_logger().error(f"Error in request_pixhawk: {error}")
            return self.pixhawk

    def main(self):

        # CONVERT TO MSG
        self.msg_heading_msg = Float64()
        self.jetson_batt_msg = UInt16()
        self.motor_batt_msg = UInt16()
        self.mux_state_msg = UInt8()
        self.imu_msg = Float64()

        # Publisher
        self.kill_switch_pub = Topic.kill_switch.createPublisher(self)
        self.heading_deg_pub = Topic.heading_deg.createPublisher(self)
        self.jetson_batt_pub = Topic.jetson_batt.createPublisher(self)
        self.motor_batt_pub = Topic.motor_batt.createPublisher(self)
        self.mux_state_pub = Topic.mux_state.createPublisher(self)
        self.pixhawk_pub = Topic.pixhawk.createPublisher(self)
        self.pxmode_pub = Topic.pxmode.createPublisher(self)
        self.get_logger().info("<> Pixhawk Publisher created")

        while rclpy.ok():  
            rclpy.spin_once(self, timeout_sec=0.01)
            if (
                self.mc1 == MiconType.NONE
                or self.mc2 == MiconType.NONE
                or self.ser_2 is None
            ):
                self.error_throttle(5000, "One of the micon is not found")
                self._init_mc()
                continue
            self.warn_once("micon found")
            self.warn_throttle(5000, f"{self.mc1}, {self.mc2}")

            #ESP32
            data = self._read_sensor_esp32()
            
            if data:
                self.kill_switch_pub.publish(self.ks_kill_state)
                self.heading_deg_pub.publish(self.msg_heading_msg)
                self.jetson_batt_msg.data = int(self._battery_value_safe(self.jetson_batt))
                self.motor_batt_msg.data = int(self._battery_value_safe(self.motor_batt))
                self.mux_state_msg.data = int(self.mux_state)
                self.jetson_batt_pub.publish(self.jetson_batt_msg)
                self.motor_batt_pub.publish(self.motor_batt_msg)
                self.mux_state_pub.publish(self.mux_state_msg)
            else:
                self.error_throttle(5000, "No data from ESP32")
            
            # Request and publish pixhawk data
            pixhawk_data = self.request_pixhawk()
            self.pixhawk_pub.publish(pixhawk_data)
            
            self.msg_heading_msg.data = float(self.pixhawk.msg_heading)
            
            # self.rc_chans = self._px_rc_val()
            # self._px_set_mode(self.rc_chans.chan8_raw)
            
            # if(self.pxmode != PxMode.AUTO):
            self._get_pwm()

            # self.warn_throttle(5000, "Sending PWM...")
            
            # Sleep equivalent to rospy.Rate(60).sleep()
            # time.sleep(1.0/60.0)
            # self.info_once("Successfully initialized node")


def main(args=None):
    rclpy.init(args=args)
    microcontroller_node = Microcontroller()

    microcontroller_node.main()

    rclpy.spin(microcontroller_node)
    microcontroller_node.destroy_node()
    rclpy.shutdown()