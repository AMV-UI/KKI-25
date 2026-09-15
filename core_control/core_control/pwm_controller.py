import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8, Bool, Float64
from core.utils.config import Topic, PxMode, Param
from core_msgs.msg import Pixhawk
from pymavlink import mavutil
import traceback
from time import time, sleep

class PWMController(Node):
    def __init__(self, node = Node):
        super().__init__('PWM')
        import os
        import fnmatch
        
        self.node = node
        
        # Auto-detect serial port
        self.master = None
        ports = []
        try:
            list_of_files = os.listdir("/dev")
            for entry in list_of_files:
                if fnmatch.fnmatch(entry, "ttyUSB*") or fnmatch.fnmatch(entry, "ttyACM*"):
                    ports.append(f"/dev/{entry}")
        except Exception:
            pass

        for port in ports:
            try:
                self.get_logger().info(f"PWMController trying to connect to Pixhawk on {port}...")
                self.master = mavutil.mavlink_connection(port, baud=57600)
                self.master.target_system = 1
                self.master.target_component = 1
                break
            except Exception as e:
                pass
                
        if self.master is None:
            self.get_logger().error("PWMController failed to find Pixhawk!")

        self.pxmode = PxMode.HOLD
        self.pwm_chan = None
        self._setup_communication()

    def _setup_communication(self):
        self.pxmode_subscriber = Topic.pxmode.createSubscriber(
            self,
            self.pxmode_callback
        )

        self.pwm_sub = Topic.pwm.createSubscriber(
            self, 
            self._pwm_callback
        )

    def _pwm_callback(self, pwm_msg):
        self.pwm_chan = pwm_msg.channels

    def pxmode_callback(self, msg: String):
        self.pxmode = msg.data

    def set_rc_channel_pwm(self, pwm_list):
        rc_channel_values = [0 for _ in range(8)]
        for idx, pwm in enumerate(pwm_list):
            if idx < 8:
                rc_channel_values[idx] = pwm
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system, 
            self.master.target_component,
            *rc_channel_values,
        )
    
    def main(self):
        self.get_logger().info("PWM Controller Node Started")
        while rclpy.ok():
            rclpy.spin_once(self)
                
            try:
                if self.pwm_chan is not None and self.pxmode == PxMode.AUTO:               
                    self.set_rc_channel_pwm(self.pwm_chan)
                else:
                    rc_channel_values = [0 for _ in range(8)]
                    self.set_rc_channel_pwm(rc_channel_values)

            except Exception as e:
                self.get_logger().error(f"Error sending PWM: {e}")
                traceback.print_exc()

def main(args=None):
    rclpy.init(args=args)
    pwmcontroller_node = PWMController()
    pwmcontroller_node.main()

    rclpy.spin(pwmcontroller_node)
    pwmcontroller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()