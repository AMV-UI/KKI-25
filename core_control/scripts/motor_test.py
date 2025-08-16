#!/usr/bin/env python3
"""
motor_test_basic.py
- Publishes Twist with linear.x, angular.z
- Estimates left/right PWM using linear.x & angular.z (differential mixing)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from core.utils.motor import Motor  

class MotorTest(Node):
    def __init__(self, rate_hz=10, linear_scale=200.0, angular_scale=200.0):
        super().__init__('motor_test_publisher_basic')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        #self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) # turtle test
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.linear_scale = linear_scale
        self.angular_scale = angular_scale
        try:
            self.motor_calc = Motor()
        except Exception as e:
            self.get_logger().warn(f'Tidak dapat membuat instance Class Motor: {e}') # Error flagger
            self.motor_calc = None
            self.STANDBY = 1500

    def map_twist_to_pwm(self, linear_x, angular_z):
        linear_ce = int(linear_x * self.linear_scale)
        angular_ce = int(angular_z * self.angular_scale)
        left_ce = linear_ce - angular_ce
        right_ce = linear_ce + angular_ce
        if self.motor_calc:
            left_pwm = self.motor_calc.calculateSpeed(left_ce)
            right_pwm = self.motor_calc.calculateSpeed(right_ce)
        else:
            left_pwm = self.STANDBY + left_ce
            right_pwm = self.STANDBY + right_ce
        return left_pwm, right_pwm, left_ce, right_ce

    def play_sequence(self):
        # sequence tuples: (name, linear_x, angular_z, durasi_s)
        # Tambahkan saja sequence baru di sini
        seq = [
            ("idle", 0.0, 0.0, 2.0),
            ("Maju", 0.5, 0.0, 4.0),
            ("Kiri", 0.0, 0.6, 3.0),
            ("Maju", 0.5, 0.0, 4.0),
            ("Kanan", 0.0, -0.6, 3.0),
            ("Maju", 0.5, 0.0, 4.0),
            ("Mundur", -0.5, 0.0, 4.0),
            ("idle", 0.0, 0.0, 2.0),
        ]

        try:
            for name, lx, az, dur in seq:
                steps = max(1, int(dur * self.rate_hz))
                self.get_logger().info(f"STEP '{name}': lx={lx} az={az} duration={dur}s -> steps={steps}")
                start_time = time.time()
                i = 0
                while (time.time() - start_time) < dur and rclpy.ok():
                    msg = Twist()
                    msg.linear.x = lx
                    msg.angular.z = az
                    self.pub.publish(msg)

                    left_pwm, right_pwm, left_ce, right_ce = self.map_twist_to_pwm(lx, az)
                    self.get_logger().info(f"[{name}] t={i/self.rate_hz:.2f}s -> left_pwm={left_pwm} right_pwm={right_pwm} (ce L={left_ce} R={right_ce})")
                    
                    i += 1
                    rclpy.spin_once(self, timeout_sec=0)
                    time.sleep(self.dt)
            self.get_logger().info("Sequence selesai")
            self.pub.publish(Twist())
        finally:
            if self.motor_calc:
                try:
                    self.motor_calc.destroy_node()
                except Exception:
                    pass

def main(args=None):
    rclpy.init(args=args)
    node = MotorTest(rate_hz=10)
    node.play_sequence()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
