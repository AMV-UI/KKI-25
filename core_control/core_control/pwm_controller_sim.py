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
        self.node = node
        self.master = mavutil.mavlink_connection("udp:127.0.0.1:14551", baud=57600)
        self.master.wait_heartbeat()
        self.master.target_system = 1 # Force target system to 1 (Vehicle) instead of 255 (MAVProxy)
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
        
        last_send_time = 0.0
        send_interval = 0.1 # 10 Hz rate limit

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
                
            try:
                current_time = time()
                if current_time - last_send_time >= send_interval:
                    if self.pwm_chan is not None and self.pxmode == PxMode.AUTO:               
                        self.set_rc_channel_pwm(self.pwm_chan)
                    else:
                        rc_channel_values = [0 for _ in range(8)]
                        self.set_rc_channel_pwm(rc_channel_values)
                    last_send_time = current_time

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