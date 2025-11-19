import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8, Bool
from core.utils.config import Topic, PxMode, Param
from core_msgs.msg import Pixhawk
from pymavlink import mavutil
import traceback
from time import time, sleep

class PWMController(Node):
    def __init__(self, node = Node):
        super().__init__('PWM')
        self.node = node
        self.master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)
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

    def send_position_target_global(conn, lat, lon, alt):
        conn.mav.set_position_target_global_int_send(
            int(time.time() * 1e6),    # timestamp (ignored)
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,        # type mask: use only position
            int(lat * 1e7),            # lat 1e7
            int(lon * 1e7),            # lon 1e7
            alt,                       # altitude
            0, 0, 0,                   # velocity
            0, 0, 0,                   # acceleration
            0, 0                       # yaw, yaw rate
        )


    def _pwm_callback(self, pwm_msg):
        self.pwm_chan = pwm_msg.channels

    def pxmode_callback(self, msg: String):
        self.pxmode = msg.data

    def set_rc_channel_pwm(self, pwm_list):
        rc_channel_values = [65535 for _ in range(8)]
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
            ## TESTING PURPOSE ONLY

            # rc_channel_values = [0 for _ in range(8)]
            # rc_channel_values[7] = 2007
            # self.set_rc_channel_pwm(rc_channel_values)


            try:
                # print(f"Current PxMode: {self.pxmode}")
                if self.pwm_chan is not None and self.pxmode == PxMode.AUTO:
                    # self._get_pwm()
                    self.get_logger().info("Sending PWM...Unsafe Mode Disabled")
                    self.set_rc_channel_pwm(self.pwm_chan)
                else:
                    # self._get_pwm()
                    rc_channel_values = [0 for _ in range(8)]
                    self.set_rc_channel_pwm(rc_channel_values)

                # time.sleep(1.0/60.0)

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
