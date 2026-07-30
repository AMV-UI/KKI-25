#!/usr/bin/env python3

import rclpy
import os
import fnmatch
import time
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import Float64, String
from core_msgs.msg import Pixhawk, ControlState
from core.utils.config import Topic


class PixhawkController(Node):
    def __init__(self):
        super().__init__("pixhawk_controller")

        self.pixhawk = Pixhawk()
        self.pxmode = "MANUAL"
        self.ser_2 = None

        self.current_manual_control = [1500.0, 1500.0, 1500.0, 1500.0]
        self.target_manual_control = [1500.0, 1500.0, 1500.0, 1500.0]
        self.rc_chans = None

        self.MAX_SLEW_PER_SEC = 400  # unit PWM per detik, sesuaikan
        self.last_servo_time = time.time()
        self.servo_pwm = 1500
        self.servo_dir = 0

        self.a_button_pressed = False
        self.b_button_pressed = False
        self.x_button_pressed = False

        self.setup_param()

        # Publishers
        self.pixhawk_pub = Topic.pixhawk.createPublisher(self)
        self.heading_deg_pub = Topic.heading_deg.createPublisher(self)
        self.pxmode_pub = Topic.pxmode.createPublisher(self)
        self.control_pub = Topic.control_state.createPublisher(self)

        # Subscribers
        self.joy_sub = Topic.joy.createSubscriber(self, self._joy_callback)

        self.msg_heading_msg = Float64()

        self._init_serial()
        self.get_logger().info("Pixhawk Controller Node Started")

    def setup_param(self):
        self.declare_parameter("log_rc", True)
        self.declare_parameter("log_manual_control", True)
        self.declare_parameter("log_joystick", True)
        self.declare_parameter("log_servo", True)

        self.declare_parameter("info_throttle", 500)
        self.declare_parameter("smoothing_factor", 0.2)

        self.declare_parameter("scale_forward", 1.0)
        self.declare_parameter("scale_lateral", 1.0)
        self.declare_parameter("scale_yaw", 1.0)
        self.declare_parameter("scale_vertical", 1.0)

        self.declare_parameter("trim_forward", 0.0)
        self.declare_parameter("trim_lateral", 0.0)
        self.declare_parameter("trim_yaw", 0.0)
        self.declare_parameter("trim_vertical", 0.0)

    def _joy_callback(self, joy_msg):

        # Parse Movement
        self.servo_dir = joy_msg.axes[7]

        now = time.time()
        dt = now - self.last_servo_time
        self.last_servo_time = now
        delta = self.servo_dir * self.MAX_SLEW_PER_SEC * dt
        self.servo_pwm = max(500, min(2500, self.servo_pwm + delta))

        raw_forward = joy_msg.axes[1] * 1
        raw_lateral = joy_msg.axes[0] * -1
        raw_yaw = joy_msg.axes[2] * -1

        raw_up = (joy_msg.axes[4] * -1.0 + 1.0) / 2.0
        raw_down = (joy_msg.axes[5] - 1.0) / 2.0
        raw_vertical = raw_up + raw_down

        scale_fwd = self.get_parameter("scale_forward").value
        scale_lat = self.get_parameter("scale_lateral").value
        scale_yaw = self.get_parameter("scale_yaw").value
        scale_vert = self.get_parameter("scale_vertical").value

        trim_fwd = self.get_parameter("trim_forward").value
        trim_lat = self.get_parameter("trim_lateral").value
        trim_yaw = self.get_parameter("trim_yaw").value
        trim_vert = self.get_parameter("trim_vertical").value

        forward = max(
            1300, min(1700, int(1500 + (raw_forward * scale_fwd + trim_fwd) * 400))
        )
        lateral = max(
            1300, min(1700, int(1500 + (raw_lateral * scale_lat + trim_lat) * 400))
        )
        yaw = max(1300, min(1700, int(1500 + (raw_yaw * scale_yaw + trim_yaw) * 400)))
        vertical = max(
            1300, min(1700, int(1500 + (raw_vertical * scale_vert + trim_vert) * 400))
        )
        self.target_manual_control = (forward, lateral, vertical, yaw)

        # Parse toggles
        b_button = joy_msg.buttons[1]  # depth hold ON
        if not b_button and self.b_button_pressed:
            self._px_set_mode("ALT_HOLD")
            self.pxmode = "ALT_HOLD"

        a_button = joy_msg.buttons[0]  # manual ON
        if not a_button and self.a_button_pressed:
            self._px_set_mode("MANUAL")
            self.pxmode = "MANUAL"

        x_button = joy_msg.buttons[3]  # stabilize ON
        if not x_button and self.x_button_pressed:
            self._px_set_mode("STABILIZE")
            self.pxmode = "STABILIZE"

        self.a_button_pressed = a_button
        self.b_button_pressed = b_button
        self.x_button_pressed = x_button

        if self.get_parameter("log_joystick").value:
            self.info_throttle(
                f"Joy Input -> Fwd: {int(forward)} | Lat: {int(lateral)} | "
                f"Vert: {int(vertical)} | Yaw: {int(yaw)} | "
                f"Btn A: {a_button} | Btn B: {b_button} | Btn X: {x_button}"
            )

    def error_throttle(self, period_ms, msg):
        self.get_logger().error(msg, throttle_duration_sec=period_ms / 1000.0)

    def info_throttle(self, msg, period_ms=None):
        self.get_logger().info(
            msg,
            throttle_duration_sec=period_ms
            if period_ms is not None
            else self.get_parameter("info_throttle").value / 1000.0,
        )

    def _get_serial_ports(self):
        dirs = []
        list_of_files = os.listdir("/dev")
        pattern = "ttyACM*"
        for entry in list_of_files:
            if fnmatch.fnmatch(entry, pattern):
                dirs.append(f"/dev/{entry}")
        return dirs

    def _init_serial(self):
        ports = self._get_serial_ports()
        if not ports:
            self.error_throttle(5000, "No USB serial ports found (Pixhawk)")
            return
        self.get_logger().info(f"Available USB ports: {ports}")
        for port in ports:
            try:
                self.ser_2 = mavutil.mavlink_connection(port, baud=57600)
                self.ser_2.wait_heartbeat()
                self.get_logger().info(f"Pixhawk found on {port}")
                self.ser_2.mav.heartbeat_send(0, 0, 0, 0, 0)
                self._px_arm()
                self.ser_2.mav.request_data_stream_send(
                    self.ser_2.target_system,
                    self.ser_2.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
                    10,
                    1,
                )

                self.ser_2.mav.request_data_stream_send(
                    self.ser_2.target_system,
                    self.ser_2.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER,
                    10,
                    1,
                )
                self.ser_2.mav.request_data_stream_send(
                    self.ser_2.target_system,
                    self.ser_2.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                    10,
                    1,
                )

                self.ser_2.mav.request_data_stream_send(
                    self.ser_2.target_system,
                    self.ser_2.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                    10,
                    1,
                )

                self.ser_2.mav.request_data_stream_send(
                    self.ser_2.target_system,
                    self.ser_2.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                    1,
                    1,
                )
                return
            except Exception as e:
                self.get_logger().warn(f"Failed to connect to Pixhawk on {port}: {e}")
                self.ser_2 = None

        self.error_throttle(5000, "Pixhawk not found on any port")

    def _px_arm(self):
        if self.ser_2 is None:
            return
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

    def request_pixhawk(self):
        try:
            msg_coor = self.ser_2.messages.get("GLOBAL_POSITION_INT", None)
            alignment = self.ser_2.messages.get("VFR_HUD", None)
            system_time = self.ser_2.messages.get("SYSTEM_TIME", None)
            attitude = self.ser_2.messages.get("ATTITUDE", None)

            if attitude is not None:
                self.pixhawk.rollspeed = attitude.rollspeed
                self.pixhawk.yawspeed = attitude.yawspeed
                self.pixhawk.pitchspeed = attitude.pitchspeed
                self.pixhawk.roll = attitude.roll
                self.pixhawk.yaw = attitude.yaw
                self.pixhawk.pitch = attitude.pitch

            if msg_coor is not None:
                self.pixhawk.lat = msg_coor.lat / 1e7
                self.pixhawk.lon = msg_coor.lon / 1e7
                self.pixhawk.alt = msg_coor.relative_alt / 1000.0
            if alignment is not None:
                self.pixhawk.msg_spd = alignment.groundspeed
                self.pixhawk.msg_heading = alignment.heading

            if system_time is not None:
                self.pixhawk.sys_time = system_time.time_unix_usec

            return self.pixhawk

        except Exception as error:
            self.get_logger().error(f"Error in request_pixhawk: {error}")
            return self.pixhawk

    def _px_set_mode(self, mode):
        self.pxmode = mode

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
        self.info_throttle(f"Mode set to : {self.pxmode}")

    def set_manual_control(self):

        for i in range(4):
            self.current_manual_control[i] += self.get_parameter(
                "smoothing_factor"
            ).value * (self.target_manual_control[i] - self.current_manual_control[i])

        forward = int(self.current_manual_control[0])
        lateral = int(self.current_manual_control[1])
        vertical = int(self.current_manual_control[2])
        yaw = int(self.current_manual_control[3])

        if self.get_parameter("log_manual_control").value:
            self.info_throttle(
                f"Sending RC_OVERRIDE -> "
                f"Fwd(CH5): {forward} | Lat(CH6): {lateral} | "
                f"Vert(CH3): {vertical} | Yaw(CH4): {yaw}"
            )

        self.ser_2.mav.rc_channels_override_send(
            self.ser_2.target_system,
            self.ser_2.target_component,
            1500,  # CH1
            1500,  # CH2
            vertical,  # CH3
            yaw,  # CH4
            forward,  # CH5
            lateral,  # CH6
            0,  # CH7
            int(self.servo_pwm),  # CH8 (Now mapped to output on Pin 9!)
            0,
            0,  # Some pymavlink versions take 10 args, some take 18. Leave trailing zeros.
        )

    def _pump_mavlink_messages(self):
        while True:
            msg = self.ser_2.recv_msg()
            if msg is None:
                break

    def _get_cur_mode(self):
        if "HEARTBEAT" in self.ser_2.messages:
            latest_heartbeat = self.ser_2.messages["HEARTBEAT"]
            return latest_heartbeat.custom_mode

        return 0

    def main(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.ser_2 is None:
                self._init_serial()
                time.sleep(1)
                continue

            self._pump_mavlink_messages()
            rc_msg = self.ser_2.messages.get("RC_CHANNELS", None)

            control_msg = ControlState()
            if self.get_parameter("log_rc").value:
                if rc_msg is not None:
                    self.info_throttle(
                        f"CH1: {rc_msg.chan1_raw} | "
                        f"CH2: {rc_msg.chan2_raw} | "
                        f"CH3: {rc_msg.chan3_raw} | "
                        f"CH4: {rc_msg.chan4_raw} | "
                        f"CH5: {rc_msg.chan5_raw} | "
                        f"CH6: {rc_msg.chan6_raw} | "
                        f"CH7: {rc_msg.chan7_raw} | "
                        f"CH8: {rc_msg.chan8_raw} | "
                        f"Mode: {self._get_cur_mode()}"
                    )
                else:
                    self.error_throttle("No RC_CHANNELS data available yet")

            servo_msg = self.ser_2.messages.get("SERVO_OUTPUT_RAW", None)

            if self.get_parameter("log_servo").value:
                if servo_msg is not None:
                    self.info_throttle(
                        f"M1: {servo_msg.servo1_raw} | M2: {servo_msg.servo2_raw} | "
                        f"M3: {servo_msg.servo3_raw} | M4: {servo_msg.servo4_raw} | "
                        f"M5: {servo_msg.servo5_raw} | M6: {servo_msg.servo6_raw}"
                    )
                else:
                    self.error_throttle("No servo output data available yet")

            control_msg.forward = rc_msg.chan5_raw
            control_msg.lateral = rc_msg.chan6_raw
            control_msg.vertical = rc_msg.chan3_raw
            control_msg.yaw = rc_msg.chan4_raw
            control_msg.mot1 = servo_msg.servo1_raw
            control_msg.mot2 = servo_msg.servo2_raw
            control_msg.mot3 = servo_msg.servo3_raw
            control_msg.mot4 = servo_msg.servo4_raw
            control_msg.mot5 = servo_msg.servo5_raw
            control_msg.mot6 = servo_msg.servo6_raw
            control_msg.grip = servo_msg.servo9_raw
            self.control_pub.publish(control_msg)

            pixhawk_data = self.request_pixhawk()
            self.pixhawk_pub.publish(pixhawk_data)

            self.msg_heading_msg.data = float(self.pixhawk.msg_heading)
            self.heading_deg_pub.publish(self.msg_heading_msg)
            self.set_manual_control()


def main(args=None):
    rclpy.init(args=args)
    node = PixhawkController()
    try:
        node.main()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
