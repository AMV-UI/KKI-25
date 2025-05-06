#!/usr/bin/env python3


# Todo
# Ubah Yang pakai interfaces ini menjadi yang dari core_msgs
import os
import fnmatch
import math
import time
import serial
import rclpy
from rclpy.node import Node
from pykalman import KalmanFilter
from pymavlink import mavutil
from std_msgs.msg import Float64, UInt8, UInt16
from core_msgs.msg import Pwm, KillSwitch, Pixhawk, AutoControl
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter


from utils.config import (
    AutoState,
    Node as NodeName,
    RemoteState,
    Topic,
    MotorReverse,
    SETPOINT,
    Param,
    PxMode,
)
from adafruit_simplemath import map_range

# Constants for ADC and battery conversion
ADS_MAX_VAL = 26096
GAIN_RATIO = 1069 / 1000
OPEN_DRAIN_RATIO = 122 / 22
BAT_MAX_VAL = 16.8
BAT_MIN_VAL = 14.4

# Kalman filter variance
SENSOR_VARIANCE = 0.1

# PWM reverse mapping\_map
_reverse_map = MotorReverse.map
_pwm_offset = 15

class MiconType:
    ESP32 = 1
    PX = 2
    NONE = -1

class Microcontroller(Node):
    def __init__(self):
        super().__init__(NodeName.microcontroller)

        self.pwm_chan = Pwm()
        self.ks_kill_state = KillSwitch(data=KillSwitch.DEFAULT)
        self.auto_status_remote = UInt8(data=RemoteState.TBS_MANUAL)
        self.auto_status_gcs = UInt8(data=AutoState.HARDWARE)
        self.jetson_batt = -1.0
        self.motor_batt = -1.0
        self.depth = -1.0
        self.dht22_raw = -1.0
        self.tbs_pwm_in = -1.0
        self.mux_state = -1.0
        self.heading_deg = -1.0
        self.echosounder_dist = -1.0
        self.echosounder_conf = -1
        self.pixhawk_msg = Pixhawk()
        self.pxmode = PxMode.MANUAL
        self.mc_esp = MiconType.NONE
        self.mc_px = MiconType.NONE

        # --- Kalman filters ---
        self.kf_depth = KalmanFilter(
            transition_matrices=[1],
            observation_matrices=[1],
            initial_state_mean=0,
            observation_covariance=SENSOR_VARIANCE,
            transition_covariance=1e-5,
        )
        self.kf_imu = KalmanFilter(
            transition_matrices=[1],
            observation_matrices=[1],
            initial_state_mean=0,
            observation_covariance=SENSOR_VARIANCE,
            transition_covariance=1e-5,
        )

        #param blackboard client
        self.param_get = self.create_client(GetParameters, "/param_controller/get_parameters")
        self.param_set = self.create_client(SetParameters, "/param_controller/set_parameters")
        while not self.param_set.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for param service...")

        #pubsub
        self.create_subscription(Pwm, Topic.pwm[0], self._pwm_callback, 10)
        self.kill_pub = self.create_publisher(KillSwitch, Topic.kill_switch[0], 10)
        self.heading_pub = self.create_publisher(Float64, Topic.heading_deg[0], 10)
        self.depth_pub = self.create_publisher(Float64, Topic.state_depth[0], 10)
        self.jetson_pub = self.create_publisher(UInt16, Topic.jetson_batt[0], 10)
        self.motor_pub = self.create_publisher(UInt16, Topic.motor_batt[0], 10)
        self.pixhawk_pub = self.create_publisher(Pixhawk, Topic.pixhawk[0], 10)
        self.pxmode_pub = self.create_publisher(UInt8, Topic.pxmode[0], 10)

        # --- Initialize devices ---
        self._init_devices()
        self.get_logger().info("Microcontroller node initialized.")

    def _init_devices(self):
        devs = os.listdir('/dev')
        acm = [f for f in devs if fnmatch.fnmatch(f, 'ttyACM*')]
        usb = [f for f in devs if fnmatch.fnmatch(f, 'ttyUSB*')]
        if acm:
            try:
                self.ser_esp = serial.Serial(f"/dev/{acm[0]}", 115200, timeout=0.1)
                self.mc_esp = MiconType.ESP32
            except Exception:
                self.get_logger().error("Failed to open ESP32 serial port")
        if usb:
            try:
                # MAVLink connection
                self.mav = mavutil.mavlink_connection(f"/dev/{usb[0]}", baud=57600)
                # handshake
                self.mav.mav.heartbeat_send(0, 0, 0, 0, 0)
                self.mav.wait_heartbeat()
                self._px_arm()
                self.mc_px = MiconType.PX
            except Exception:
                self.get_logger().error("Failed to open Pixhawk MAVLink port")

    def _pwm_callback(self, msg: Pwm):
        self.pwm_chan = msg

    def set_param(self, name: str, value: int) -> bool:
        req = SetParameters.Request()
        param_val = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
        req.parameters = [Parameter(name=name, value=param_val)]
        fut = self.param_set.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result() is not None and all(r.successful for r in fut.result().results)

    def _read_esp(self) -> bool:
        if not (self.mc_esp == MiconType.ESP32 and self.ser_esp.in_waiting):
            return False
        raw = self.ser_esp.readline().decode('utf-8', errors='ignore')
        if not raw.startswith('s'):
            return False
        vals = [float(x) for x in raw[1:].strip().split(',')]
        (self.jetson_batt, self.motor_batt, depth_raw, ks, self.dht22_raw,
         self.tbs_pwm_in, self.mux_state, heading_raw,
         self.echosounder_dist, self.echosounder_conf) = vals
        # Kalman filter heading
        _, h_filt = self.kf_imu.filter(heading_raw)
        self.heading_deg = h_filt[-1][0]
        # Depth mapping + filter
        dep_val = int(map_range(depth_raw, 0, 4095, 0, 1000)) * 2
        if dep_val < SETPOINT.SETPOINT_DEPTH:
            self.set_param('y_speed', 2)
        else:
            self.set_param('y_speed', -1)
        _, d_filt = self.kf_depth.filter(dep_val)
        self.depth = d_filt[-1][0]
        return True

    def _px_arm(self):
        # Arm Pixhawk motors
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        self.mav.motors_armed_wait()
        self.get_logger().info("Pixhawk armed.")

    def _px_set_mode(self, chan8_raw: int):
        # choose PxMode based on channel 8
        if chan8_raw <= 1300:
            mode = PxMode.HOLD #
        else:
            mode = PxMode.MANUAL
        if mode != self.pxmode:
            self.pxmode = mode
            mode_id = self.mav.mode_mapping()[self.pxmode]
            self.mav.mav.set_mode_send(
                self.mav.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            self.get_logger().info(f"Set PX mode to {self.pxmode}")

    def _px_telemetry(self):
        # read ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD
        msg_att = self.mav.recv_match(type='ATTITUDE', blocking=True)
        msg_pos = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        msg_hud = self.mav.recv_match(type='VFR_HUD', blocking=True)
        # fill Pixhawk msg
        yaw = math.degrees(msg_att.yaw) % 360
        self.pixhawk_msg.msg_heading = msg_hud.heading
        self.pixhawk_msg.msg_spd = msg_hud.groundspeed
        self.pixhawk_msg.lat = msg_pos.lat / 1e7
        self.pixhawk_msg.lon = msg_pos.lon / 1e7
        self.pixhawk_msg.alt = msg_pos.alt / 1000.0

    def _override_rc(self):
        # read RC_CHANNELS and override based on pwm_chan
        rc = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
        self._px_set_mode(rc.chan8_raw)
        # send override for channels 1-6
        vals = [65535]*8
        for i, v in enumerate(self.pwm_chan.channels[:6]):
            vals[i] = int(v)
        self.mav.mav.rc_channels_override_send(
            self.mav.target_system, self.mav.target_component, *vals
        )

    def run(self):
        rate = self.create_rate(60)
        while rclpy.ok():
            # read sensors
            self._read_esp()
            # telemetry
            if self.mc_px == MiconType.PX:
                self._px_telemetry()
                self._override_rc()
            # publish all
            self.kill_pub.publish(self.ks_kill_state)
            self.heading_pub.publish(Float64(data=self.heading_deg))
            self.depth_pub.publish(Float64(data=self.depth))
            self.jetson_pub.publish(UInt16(data=int(self.jetson_batt)))
            self.motor_pub.publish(UInt16(data=int(self.motor_batt)))
            if self.mc_px == MiconType.PX:
                self.pixhawk_pub.publish(self.pixhawk_msg)
                self.pxmode_pub.publish(UInt8(data=self.pxmode))
            rclpy.spin_once(self)
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = Microcontroller()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
