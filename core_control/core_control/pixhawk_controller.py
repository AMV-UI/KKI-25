#!/usr/bin/env python3

import rclpy
import os
import fnmatch
import time
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import Float64, String
from core_msgs.msg import Pixhawk, Pwm
from core.utils.config import Topic, PxMode, MotorReverse

class PixhawkController(Node):
    def __init__(self):
        super().__init__('pixhawk_controller')
        
        self.pixhawk = Pixhawk()
        self.pxmode = "MANUAL"
        self.rc_chans = None
        self.pwm_chan = None
        self.ser_2 = None
        
        # Publishers
        self.pixhawk_pub = Topic.pixhawk.createPublisher(self)
        self.heading_deg_pub = Topic.heading_deg.createPublisher(self)
        self.rc5_pub = Topic.rc5.createPublisher(self)
        self.rc6_pub = Topic.rc6.createPublisher(self)
        self.pxmode_pub = Topic.pxmode.createPublisher(self)
        
        # Subscribers
        self.pwm_sub = Topic.pwm.createSubscriber(self, self._pwm_callback)
        
        self.msg_heading_msg = Float64()
        
        self._init_serial()
        self.get_logger().info("Pixhawk Controller Node Started")

    def error_throttle(self, period_ms, msg):
        self.get_logger().error(msg, throttle_duration_sec=period_ms/1000.0)
        
    def info_throttle(self, period_ms, msg):
        self.get_logger().info(msg, throttle_duration_sec=period_ms/1000.0)

    def _get_serial_ports(self):
        dirs = []
        list_of_files = os.listdir("/dev")
        pattern = "ttyUSB*"
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
                # Request data stream so that Pixhawk streams GPS and RC data
                self.ser_2.mav.request_data_stream_send(
                    self.ser_2.target_system, self.ser_2.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
                )
                self._px_arm()
                return
            except Exception as e:
                self.get_logger().warn(f"Failed to connect to Pixhawk on {port}: {e}")
                self.ser_2 = None
        
        self.error_throttle(5000, "Pixhawk not found on any port")

    def _px_arm(self):
        if self.ser_2 is None: return
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
        if self.ser_2 is None: return self.pixhawk
        try:
            # Pump the serial buffer to get all latest messages
            while True:
                msg = self.ser_2.recv_match(blocking=False)
                if not msg:
                    break

            # Read latest cached GLOBAL_POSITION_INT
            if 'GLOBAL_POSITION_INT' in self.ser_2.messages:
                msg_coor = self.ser_2.messages['GLOBAL_POSITION_INT']
                self.pixhawk.lat = msg_coor.lat / 1e7
                self.pixhawk.lon = msg_coor.lon / 1e7
                self.pixhawk.alt = msg_coor.alt / 1000

            # Read latest cached VFR_HUD
            if 'VFR_HUD' in self.ser_2.messages:
                alignment = self.ser_2.messages['VFR_HUD']
                self.pixhawk.msg_spd = alignment.groundspeed
                self.pixhawk.msg_heading = alignment.heading

            return self.pixhawk

        except Exception as error:
            self.get_logger().error(f"Error in request_pixhawk: {error}")
            return self.pixhawk

    def _px_rc_val(self):
        if self.ser_2 is None: return self.rc_chans
        if 'RC_CHANNELS' in self.ser_2.messages:
            self.rc_chans = self.ser_2.messages['RC_CHANNELS']
        return self.rc_chans

    def _get_pwm(self):
        self.rc5_pub.publish(Float64(data=float(self.rc_chans.chan5_raw)))
        self.rc6_pub.publish(Float64(data=float(self.rc_chans.chan6_raw)))
        self.get_logger().info(f"Channel Values : {self.rc_chans.chan1_raw}, {self.rc_chans.chan3_raw}, {self.rc_chans.chan8_raw}", throttle_duration_sec=1.0)

    def _px_set_mode(self, pwm_val):
        if self.ser_2 is None: return
        
        if pwm_val <= 1300:
            self.pxmode = PxMode.AUTO
        elif 1301 <= pwm_val <= 1700:
            self.pxmode = PxMode.AUTO
        else:
            self.pxmode = PxMode.AUTO

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

    def _pwm_callback(self, pwm_msg):
        self.pwm_chan = pwm_msg.channels

    def set_rc_channel_pwm(self, pwm_list):
        if self.ser_2 is None: return
        rc_channel_values = [0 for _ in range(8)]
        for idx, pwm in enumerate(pwm_list):
            if idx < 8:
                rc_channel_values[idx] = pwm
        
        self.ser_2.mav.rc_channels_override_send(
            self.ser_2.target_system, 
            self.ser_2.target_component,
            *rc_channel_values,
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
            
            self.rc_chans = self._px_rc_val()
            if self.rc_chans:
                self._px_set_mode(self.rc_chans.chan8_raw)
                self._get_pwm()
            
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

if __name__ == '__main__':
    main()
