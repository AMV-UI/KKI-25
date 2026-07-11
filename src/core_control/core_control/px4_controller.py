#!/usr/bin/env python3

import rclpy
import os
import fnmatch
import time
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import Float64, String
from core_msgs.msg import Pixhawk
from core.utils.config import Topic


class PixhawkController(Node):
    def __init__(self):
        super().__init__("pixhawk_controller")

        self.pixhawk = Pixhawk()
        self.pxmode = "MANUAL"
        self.ser_2 = None
        self.current_manual_control = (0, 0, 0, 0)

        # Publishers
        self.pixhawk_pub = Topic.pixhawk.createPublisher(self)
        self.heading_deg_pub = Topic.heading_deg.createPublisher(self)
        self.pxmode_pub = Topic.pxmode.createPublisher(self)

        # Subscribers
        self.joy_sub = Topic.joy.createSubscriber(self, self._joy_callback)

        self.msg_heading_msg = Float64()

        self.forward = 0
        self.lateral = 0
        self.yaw = 0
        self.vertical = 0

        self._init_serial()
        self.get_logger().info("Pixhawk Controller Node Started")

    def _joy_callback(self, joy_msg):
        self.forward = joy_msg.axes[1] * 1000
        self.lateral = joy_msg.axes[0] * -1000
        self.yaw = joy_msg.axes[3] * -1000
        if int(joy_msg.axes[7]) == 1:
            self.vertical = 500
        elif int(joy_msg.axes[7]) == -1:
            self.vertical = -500
        else:
            self.vertical = 0

    def error_throttle(self, period_ms, msg):
        self.get_logger().error(msg, throttle_duration_sec=period_ms / 1000.0)

    def info_throttle(self, period_ms, msg):
        self.get_logger().info(msg, throttle_duration_sec=period_ms / 1000.0)

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
        if self.ser_2 is None:
            return self.pixhawk
        try:
            msg_coor = self.ser_2.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
            lat = msg_coor.lat / 1e7 if msg_coor != None else self.pixhawk.lat
            lon = msg_coor.lon / 1e7 if msg_coor != None else self.pixhawk.lon
            alt = (msg_coor.alt / 1000) if msg_coor != None else self.pixhawk.alt

            alignment = self.ser_2.recv_match(type="VFR_HUD", blocking=True)

            msg_spd = (
                alignment.groundspeed if alignment != None else self.pixhawk.msg_spd
            )  # Ground speed in m/s
            msg_heading = (
                alignment.heading if alignment != None else self.pixhawk.msg_heading
            )

            self.pixhawk.lat = lat
            self.pixhawk.lon = lon
            self.pixhawk.alt = alt
            self.pixhawk.msg_heading = msg_heading
            self.pixhawk.msg_spd = msg_spd

            return self.pixhawk

        except Exception as error:
            self.get_logger().error(f"Error in request_pixhawk: {error}")
            return self.pixhawk

    def _px_set_mode(self, mode):
        if self.ser_2 is None:
            return

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
        self.info_throttle(5000, f"Mode set to : {self.pxmode}")

    def set_manual_control(self):
        self.ser_2.mav.manual_control_send(
            self.ser_2.target_system,
            self.current_manual_control[0],
            self.current_manual_control[1],
            self.current_manual_control[2],
            self.current_manual_control[3],
            0,
        )

    def main(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.ser_2 is None:
                self._init_serial()
                time.sleep(1)
                continue

            # Request and publish pixhawk data
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
