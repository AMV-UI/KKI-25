#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from core_msgs.msg import KillSwitch, Controller, Pwm

class MotorTest(Node):
    def __init__(self):
        super().__init__('motor_test_manual')

        # Publisher
        self.pub_mission = self.create_publisher(UInt8, '/core/mission/current', 10)
        self.pub_kill    = self.create_publisher(KillSwitch, '/kill_switch', 10)
        self.pub_joy     = self.create_publisher(Controller, '/controller', 10)

        # Subscriber to lihat PWM
        self.sub_pwm = self.create_subscription(Pwm, '/pwm', self._pwm_cb, 10)

        self._seq = 0
        self._pwm_count = 0

        self._timer = self.create_timer(0.5, self._manual_control)  # Tiap 0.5 detik
        self.get_logger().info('Manual MotorTest started')

    def _manual_control(self):
        # Aktifkan misi manual (ID = 0)
        self.pub_mission.publish(UInt8(data=0))

        # Pastikan killswitch OFF
        self.pub_kill.publish(KillSwitch(data=0))

        # Atur arah gerak secara manual
        joy = Controller()
        # Gerakan
        if self._seq < 3:
            if self._seq == 0:
                self.get_logger().info("Maju")
                joy.linear_y = 300.0
            elif self._seq == 1:
                self.get_logger().info("Belok Kiri")
                joy.angular_z = 300.0
            elif self._seq == 2:
                self.get_logger().info("Belok Kanan")
                joy.angular_z = -300.0
        else:
            self.get_logger().info("Diam")
            joy.linear_y = 0.0
            joy.angular_z = 0.0

        self.pub_joy.publish(joy)
        self._seq += 1

        if self._seq > 4:
            self.get_logger().info("Selesai")
            rclpy.shutdown()

    def _pwm_cb(self, msg: Pwm):
        self._pwm_count += 1
        self.get_logger().info(f'← PWM #{self._pwm_count}: {list(msg.channels)}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorTest()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
