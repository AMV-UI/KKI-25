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
# from core_msgs.msg import *
from adafruit_simplemath import map_range
from pymavlink import mavutil
import time
from rclpy.node import Node

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
        super().__init__('Microcontroller')  # Initialize the parent Node class
        self.pwm_chan = Pwm()
        self.mc1 = MiconType.NONE
        self.mc2 = MiconType.NONE

        # Node
        # self.node = Node()

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

        # Action servers
        # self.change_mission_AS = actionlib.SimpleActionClient('mission_change', SendMissionAction)

        # GCS <> Micon
        self.auto_status_gcs = UInt8()  # HARDWARE, AUTO, MANUAL
        self.auto_status_gcs.data = AutoState.HARDWARE
        self.auto_status_remote = UInt8()  # TBS_AUTO, TBS_MANUAL
        self.auto_status_remote.data = RemoteState.TBS_MANUAL
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

    def _init_mc(self):
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
                # MAVLink connection
                self.ser_2 = mavutil.mavlink_connection(f"/dev/{usb[0]}", baud=57600)
                # handshake
                self.ser_2.mav.heartbeat_send(0, 0, 0, 0, 0)
                self.ser_2.wait_heartbeat()
                self._px_arm()
                self.mc2 = MiconType.PX
            except Exception:
                self.get_logger().error("Failed to open Pixhawk MAVLink port")
        else:
            self.mc2 = MiconType.NONE
            self.get_logger().info("No ttyUSB* connection")

    @staticmethod
    def _parse_raw(raw_str):
        try:
            return [float(e) for e in raw_str.replace("\r\n", "").split(",")]
        except Exception as e:
            print(e)
            print("imgay")
            return None

    @staticmethod
    def _validate(pwm):
        if pwm < 1100 or pwm > 1900:
            return 1500
        return pwm

    def _read_sensor_esp32(self):
        parsed_data = None

        # print(self.ser_1.in_waiting, self.ser_2.in_waiting)

        # TODO : do this
        if self.ser_1.in_waiting:
            raw_ser_1 = self.ser_1.readline().decode()
            # print(f'ser1: {raw_ser_1}')

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
        """Set RC channel pwm value
        Args:
            channel_id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900nnel.MOTOR_LEFT
        """
        if channel_id < 1:
            # print("Channel does not exist.")
            # rclpy.logerr_throttle(
            #     5, "<=> [{Node.microcontroller}] PWM Channel does not exist."
            # )
            self.get_logger().error_throttle(5000, "PWM Channel does not exist")
            return

        # Mavlink 2 supports up to 18 channels:
        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        if channel_id < 9:
            rc_channel_values = [65535 for _ in range(8)]
            rc_channel_values[channel_id - 1] = pwm
            self.ser_2.mav.rc_channels_override_send(
                self.ser_2.target_system,  # target_system
                self.ser_2.target_component,  # target_component
                *rc_channel_values,
            )  # RC channel list, in microseconds.

    def _px_rc_val(self):
        rc_channels = self.ser_2.recv_match(type="RC_CHANNELS", blocking=True)
        if not rc_channels:
            # rclpy.logerr_throttle(
            #     5, "[{Node.microcontroller}] No message in RC_CHANNELS"
            # )

            self.get_logger().info("No message in RC_CHANNELS")
            return
        return rc_channels

    def _send_pwm(self, rc_channels):
        pwm_count = 1
        # pwm_test = ["1600", "1500", "1700", "1600", "1500", "1700", "1700"]+

        if rc_channels.chan8_raw > 1700:
            try:
                for pwm_val in self.pwm_chan.channels:
                    if pwm_count >= 8:
                        pwm_count = 1
                        break
                    #   self.set_rc_channel_pwm(pwm_count, pwm=int(pwm_val))
                    #   print(pwm_count, int(pwm_val))
                    self.set_rc_channel_pwm(pwm_count, pwm=int(pwm_val))
                    # rclpy.logerr_throttle(5, "<=> [{Node.microcontroller}] PWM sent")

                    # self.get_logger().error_throttle(5000,  "<=> [{Node.microcontroller}] PWM sent")
                    self.get_logger().info("PWM sent")
                    pwm_count = pwm_count + 1
                    # time.sleep(0.2)
            except Exception as e:
                self.get_logger().error(f"PWM Channel cannot pass. Error : {e}")

        elif 1301 <= rc_channels.chan8_raw <= 1700:
            try:
                for pwm_val in self.pwm_chan.channels:
                    if pwm_count >= 8:
                        pwm_count = 1
                        break
                    self.set_rc_channel_pwm(pwm_count, 65535)
                    self.get_logger().info("Change mode to Manual control. Throttle PWM.")
                    pwm_count = pwm_count + 1

            except Exception as e:
                self.get_logger().error(f"Manual Mode: PWM Channel cannot pass. Error : {e}")
        else:
            self.get_logger().info("Error Mode : PWM not sent")

    def _get_pwm(self):
        rc_channels = self.ser_2.recv_match(type="RC_CHANNELS", blocking=True)
        self.get_logger().info(f"Channel Values : {rc_channels}")

    def auto_status_gcs_cb(self, msg):
        self.auto_status_gcs.data = msg.data
        # self.auto_status_gcs.data = 0

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
        self.get_logger().info("Arming motors ...")
        self.ser_2.motors_armed_wait()
        self.get_logger().info("Motor Armed!")

    def _px_set_mode(self, pwm_val):
        if pwm_val <= 1300:
            self.pxmode = PxMode.HOLD
        elif 1301 <= pwm_val <= 1700:
            self.pxmode = PxMode.MANUAL
        else:
            self.pxmode = PxMode.MANUAL

        self.get_logger().info(f"Current Mode : {self.pxmode}")

        if self.pxmode not in self.ser_2.mode_mapping():
            self.get_logger().info(f"Unknown Mode : {self.pxmode}")
            # print("Try:", list(self.ser_2.mode_mapping().keys()))
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
        while True:
            time.sleep(0.1)
            try:
                self.ser_2.mav.param_request_read_send(
                    self.ser_2.target_system,
                    self.ser_2.target_component,
                    b"COMPASS_OFS_X",
                    -1,
                )

                msg = self.ser_2.recv_match(type="ATTITUDE", blocking=True)
                yaw_deg = math.degrees(msg.yaw)
                if yaw_deg < 0:
                    yaw_deg += 360

                msg_coor = self.ser_2.recv_match(
                    type="GLOBAL_POSITION_INT", blocking=True
                )
                lat = msg_coor.lat / 1e7
                lon = msg_coor.lon / 1e7
                alt = (
                    msg_coor.alt / 1000
                )  # Altitude in meters (millimeters in the message)

                alignment = self.ser_2.recv_match(type="VFR_HUD", blocking=True)

                msg_spd = alignment.groundspeed  # Ground speed in m/s
                msg_heading = alignment.heading
                # airspeed = msg.airspeed  # Airspeed in m/s

                self.pixhawk.lat = lat
                self.pixhawk.lon = lon
                self.pixhawk.alt = alt
                self.pixhawk.msg_heading = msg_heading
                self.pixhawk.msg_spd = msg_spd

                return self.pixhawk

            except Exception as error:
                print(error)
                sys.exit(0)

    def main(self):

        # CONVERT TO MSG
        self.msg_heading_msg = Float64()
        self.jetson_batt_msg = UInt16()
        self.motor_batt_msg = UInt16()
        self.mux_state_msg = UInt8()

        # Publisher
        kill_switch_pub = Topic.kill_switch.createPublisher(self)
        heading_deg_pub = Topic.heading_deg.createPublisher(self)
        auto_status_remote_pub = Topic.auto_status_remote.createPublisher(self)
        jetson_batt_pub = Topic.jetson_batt.createPublisher(self)
        motor_batt_pub = Topic.motor_batt.createPublisher(self)
        mux_state_pub = Topic.mux_state.createPublisher(self)
        pixhawk_pub = Topic.pixhawk.createPublisher(self)
        pxmode_pub = Topic.pxmode.createPublisher(self)

        # Subscriber
        # self.pwm_sub = Topic.pwm.createSubscriber(self._pwm_callback)
        # auto_status_gcs_sub = Topic.auto_status_gcs.createSubscriber(self.auto_status_gcs_cb)

        while rclpy.ok():
            if (
                self.mc1 == MiconType.NONE
                or self.mc2 == MiconType.NONE
                or self.ser_2 is None
            ):
                self.get_logger().error("micon not found")
                self._init_mc()
                continue

            self.get_logger().info("micon found")

            # data = self._read_sensor_esp32()
            # if not data:
            #    continue

            # Inserting Data
            self.msg_heading_msg.data = float(self.pixhawk.msg_heading)
            # aneh errornya, kl pke uint16 error tipe data harus int, kalau pake int error tipe data harus uint16
            # self.jetson_batt_msg.data = np.uint16(self.jetson_batt) 
            # self.motor_batt_msg.data = np.uint16(self.motor_batt)
            # self.mux_state_msg.data = np.uint8(self.mux_state)

            # Publish data
            kill_switch_pub.publish(self.ks_kill_state)
            heading_deg_pub.publish(self.msg_heading_msg)
            auto_status_remote_pub.publish(self.auto_status_remote)
            # jetson_batt_pub.publish(self.jetson_batt_msg)
            # motor_batt_pub.publish(self.motor_batt)
            # internal_temp_pub.publish(self.internal_temp_deg)
            # mux_state_pub.publish(self.mux_state)
            # pixhawk_pub.publish(self.request_pixhawk())
            # pxmode_pub.publish(self.pxmode)

            # if (self.echosounder_dist >= 0 and self.echosounder_conf >= 0):
            # echo_dist_pub.publish(self.echosounder_dist)
            # echo_conf_pub.publish(self.echosounder_conf)

            # print(self.jetson_batt, "gay")

            filtered_avg = 0.0
            N_ITR = 10

            # self.set_rc_channel_pwm(2, 1800)
            rc_chans = self._px_rc_val()
            self._px_set_mode(rc_chans.chan8_raw)
            self._send_pwm(rc_chans)

            self._get_pwm()
            self.get_logger().info("Sending PWM...")

            # self.rate.sleep()
            self.get_logger().info("Successfully initialized node")

            # if self.ser_0.in_waiting :
            #    raw_ser_1 = self.ser_1.readline().decode()
            #    print(raw_ser_1)
            #    if raw_ser_1[0] == "s":
            #        parsed_data = self._parse_raw(raw_ser_1[1:])
            #        self.jetson_batt, self.motor_batt, depth, ks_state, self.dht22_raw, self.tbs_pwm_in, mux_state, heading_deg, self.echosounder_dist, self.echosounder_conf = parsed_data
            #        print(self.jetson_batt)


def main(args=None):
    rclpy.init(args=args)
    micon = Microcontroller()
    micon.main()


if __name__ == "__main__":
    try:
        main()

    except Exception:
        rclpy.logerr(traceback.format_exc())
