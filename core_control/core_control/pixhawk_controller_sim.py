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

    def _init_serial(self):
        port = "udp:127.0.0.1:14550"
        self.get_logger().info(f"Connecting to Pixhawk SITL via {port}")            
        try:
            self.ser_2 = mavutil.mavlink_connection(port, baud=57600)
            self.ser_2.wait_heartbeat()
            self.ser_2.target_system = 1 # Force target system to 1 (Vehicle)
            self.get_logger().info(f"Pixhawk SITL found on {port}")
            self.ser_2.mav.heartbeat_send(0, 0, 0, 0, 0)
            self._px_arm()
            return
        except Exception as e:
            self.get_logger().warn(f"Failed to connect to Pixhawk SITL on {port}: {e}")
            self.ser_2 = None
        
        self.error_throttle(5000, "Pixhawk SITL not found on UDP")

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
        # Do not block here! We will check motors_armed() in main loop.

    def request_pixhawk(self):
        if self.ser_2 is None: return self.pixhawk
        try:
            msg_coor = self.ser_2.recv_match(
                type="GLOBAL_POSITION_INT", blocking=True, timeout=0.1
            )
            lat = msg_coor.lat / 1e7 if msg_coor != None else self.pixhawk.lat
            lon = msg_coor.lon / 1e7 if msg_coor != None else self.pixhawk.lon
            alt = (
                msg_coor.alt / 1000
            )  if msg_coor != None else self.pixhawk.alt

            alignment = self.ser_2.recv_match(type="VFR_HUD", blocking=True, timeout=0.1)

            msg_spd = alignment.groundspeed if alignment != None else self.pixhawk.msg_spd  # Ground speed in m/s
            msg_heading = alignment.heading  if alignment != None else self.pixhawk.msg_heading

            self.pixhawk.lat = lat
            self.pixhawk.lon = lon
            self.pixhawk.alt = alt
            self.pixhawk.msg_heading = msg_heading
            self.pixhawk.msg_spd = msg_spd

            return self.pixhawk

        except Exception as error:
            # Avoid spamming error
            return self.pixhawk

    def _px_rc_val(self):
        if self.ser_2 is None: return self.rc_chans
        fetched_channels = self.ser_2.recv_match(type="RC_CHANNELS", blocking=True, timeout=0.1)
        self.rc_chans = fetched_channels if fetched_channels != None else self.rc_chans
        return self.rc_chans

    def _get_pwm(self):
        self.rc5_pub.publish(Float64(data=float(self.rc_chans.chan5_raw)))
        self.rc6_pub.publish(Float64(data=float(self.rc_chans.chan6_raw)))
        self.get_logger().info(f"Channel Values : {self.rc_chans.chan1_raw}, {self.rc_chans.chan3_raw}, {self.rc_chans.chan8_raw}", throttle_duration_sec=1.0)

    def _px_set_mode(self, pwm_val):
        if self.ser_2 is None: return
        
        # In simulation, we don't have a real RC transmitter to flip the switch.
        # We force the mode to AUTO so pwm_controller_sim will forward computer thrust commands.
        self.pxmode = PxMode.AUTO

        pxmode_msg = String()
        pxmode_msg.data = self.pxmode
        self.pxmode_pub.publish(pxmode_msg)       
        
        # In ArduPilot, AUTO mode means the autopilot flies its own mission and IGNORES RC overrides!
        # Since KKI-25 uses AUTO to mean "send RC overrides from computer", 
        # we must tell ArduPilot to go into MANUAL mode so it accepts the overrides.
        ardupilot_mode = "MANUAL" if self.pxmode == PxMode.AUTO else self.pxmode
        
        if ardupilot_mode not in self.ser_2.mode_mapping():
            self.get_logger().warn(f"Unknown Mode : {ardupilot_mode}")
            return

        mode_id = self.ser_2.mode_mapping()[ardupilot_mode]

        self.ser_2.mav.set_mode_send(
            self.ser_2.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        self.info_throttle(5000, f"Mode set to : {self.pxmode} (ArduPilot: {ardupilot_mode})")

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
            
            # Pump incoming MAVLink messages so internal states (like motors_armed) are updated
            while self.ser_2.recv_match(type='HEARTBEAT', blocking=False) is not None:
                pass

            # Check if armed. If not, try to arm again and wait.
            if not self.ser_2.motors_armed():
                self._px_arm()
                time.sleep(1)
                continue

            # Request and publish pixhawk data
            pixhawk_data = self.request_pixhawk()
            self.pixhawk_pub.publish(pixhawk_data)
            
            self.msg_heading_msg.data = float(self.pixhawk.msg_heading)
            self.heading_deg_pub.publish(self.msg_heading_msg)
            
            self.rc_chans = self._px_rc_val()
            
            # In simulation, we always want the mode to be AUTO, regardless of whether 
            # ArduPilot is publishing RC_CHANNELS yet.
            self._px_set_mode(0)
            
            if self.rc_chans:
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