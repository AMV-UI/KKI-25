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


    def _get_serial_ports(self):
        dirs = []
        try:
            list_of_files = os.listdir("/dev")
            for entry in list_of_files:
                if fnmatch.fnmatch(entry, "ttyUSB*") or fnmatch.fnmatch(entry, "ttyACM*"):
                    dirs.append(f"/dev/{entry}")
        except Exception:
            pass
        return dirs

    def _init_serial(self):
        ports = self._get_serial_ports()
        if not ports:
            self.get_logger().error("No USB serial ports found (Pixhawk)", throttle_duration_sec=5.0)
            return

        self.get_logger().info(f"Available USB ports: {ports}")            
        for port in ports:
            for baud in [57600, 115200, 38400, 9600, 921600]:
                try:
                    self.get_logger().info(f"Trying to connect to Pixhawk on {port} at {baud} baud...")
                    self.ser_2 = mavutil.mavlink_connection(port, baud=baud)
                    msg = self.ser_2.wait_heartbeat(timeout=3.0)
                    
                    if msg is not None:
                        self.get_logger().info(f"Pixhawk found on {port} at {baud} baud!")
                        self.ser_2.mav.heartbeat_send(0, 0, 0, 0, 0)
                        self._px_arm()
                        return
                    else:
                        self.get_logger().warn(f"No heartbeat from {port} at {baud} baud (Timeout).")
                        self.ser_2.close()
                        self.ser_2 = None
                except Exception as e:
                    self.get_logger().warn(f"Failed to connect to Pixhawk on {port} at {baud}: {e}")
                    self.ser_2 = None
        
        self.get_logger().error("Pixhawk not found on any port", throttle_duration_sec=5.0)

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
        self.get_logger().error("Arming motors ...", throttle_duration_sec=5.0)
        self.ser_2.motors_armed_wait()
        self.get_logger().warn("Motor Armed!")

    def request_pixhawk(self):
        if self.ser_2 is None: return self.pixhawk
        try:
            msg_coor = self.ser_2.recv_match(
                type="GLOBAL_POSITION_INT", blocking=False
            )
            if msg_coor is not None:
                self.pixhawk.lat = msg_coor.lat / 1e7
                self.pixhawk.lon = msg_coor.lon / 1e7
                self.pixhawk.alt = msg_coor.alt / 1000

            alignment = self.ser_2.recv_match(type="VFR_HUD", blocking=False)
            if alignment is not None:
                self.pixhawk.msg_spd = alignment.groundspeed
                self.pixhawk.msg_heading = alignment.heading

            return self.pixhawk

        except Exception as error:
            self.get_logger().error(f"Error in request_pixhawk: {error}")
            return self.pixhawk

    def _px_rc_val(self):
        if self.ser_2 is None: return self.rc_chans
        fetched_channels = self.ser_2.recv_match(type="RC_CHANNELS", blocking=False)
        self.rc_chans = fetched_channels if fetched_channels != None else self.rc_chans
        return self.rc_chans

    def _get_pwm(self):
        self.rc5_pub.publish(Float64(data=float(self.rc_chans.chan5_raw)))
        self.rc6_pub.publish(Float64(data=float(self.rc_chans.chan6_raw)))
        self.get_logger().info(f"Channel Values : {self.rc_chans.chan1_raw}, {self.rc_chans.chan3_raw}, {self.rc_chans.chan8_raw}", throttle_duration_sec=1.0)

    def _px_set_mode(self, pwm_val):
        if self.ser_2 is None: return
        
        # NOTE: Sesuaikan dengan mode switch Anda (sepertinya di Jetson sudah diubah)
        new_pxmode = self.pxmode
        if pwm_val <= 1300:
            new_pxmode = PxMode.AUTO
        elif 1301 <= pwm_val <= 1700:
            new_pxmode = PxMode.AUTO
        else:
            new_pxmode = PxMode.AUTO

        if new_pxmode != getattr(self, '_last_pxmode', None):
            self.pxmode = new_pxmode
            self._last_pxmode = new_pxmode
            
            pxmode_msg = String()
            pxmode_msg.data = self.pxmode
            self.pxmode_pub.publish(pxmode_msg)       
            
            # Pixhawk mode to send
            pxhawk_internal_mode = self.pxmode
            
            # CRITICAL: Jika ROS mode adalah AUTO, kita HARUS mengirim mode MANUAL ke Pixhawk.
            # Karena di ArduRover, mode AUTO mengabaikan RC Override (menunggu waypoint).
            # Mode MANUAL mengizinkan RC Override untuk mengontrol thruster.
            if self.pxmode == PxMode.AUTO:
                pxhawk_internal_mode = PxMode.MANUAL

            if pxhawk_internal_mode not in self.ser_2.mode_mapping():
                self.get_logger().warn(f"Unknown Mode : {pxhawk_internal_mode}")
                return

            mode_id = self.ser_2.mode_mapping()[pxhawk_internal_mode]

            self.ser_2.mav.set_mode_send(
                self.ser_2.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            self.get_logger().info(f"Mode set to : {self.pxmode} (Pixhawk Internal: {pxhawk_internal_mode})")
        else:
            self.get_logger().info(f"Mode is : {self.pxmode}", throttle_duration_sec=5.0)

    def _pwm_callback(self, pwm_msg):
        self.pwm_chan = pwm_msg.channels

    def set_rc_channel_pwm(self, pwm_list):
        if self.ser_2 is None: return
        rc_channel_values = [0 for _ in range(8)]
        for idx, pwm in enumerate(pwm_list):
            if idx < 8:
                rc_channel_values[idx] = pwm
                
        self.get_logger().info(f"Sending RC Override: {rc_channel_values}", throttle_duration_sec=2.0)
        
        self.ser_2.mav.rc_channels_override_send(
            self.ser_2.target_system, 
            self.ser_2.target_component,
            *rc_channel_values,
        )

    def main(self):
        last_heartbeat_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.ser_2 is None:
                self._init_serial()
                time.sleep(1)
                continue

            current_time = time.time()
            if current_time - last_heartbeat_time > 1.0:
                self.ser_2.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                last_heartbeat_time = current_time

            # Request and publish pixhawk data
            pixhawk_data = self.request_pixhawk()
            self.pixhawk_pub.publish(pixhawk_data)
            
            self.msg_heading_msg.data = float(self.pixhawk.msg_heading)
            self.heading_deg_pub.publish(self.msg_heading_msg)
            
            self.rc_chans = self._px_rc_val()
            if self.rc_chans:
                self._px_set_mode(self.rc_chans.chan8_raw)
                self._get_pwm()
                
                if self.pxmode == PxMode.AUTO and self.pwm_chan is not None:
                    self.set_rc_channel_pwm(self.pwm_chan)
                else:
                    self.set_rc_channel_pwm([0] * 8)
            
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
