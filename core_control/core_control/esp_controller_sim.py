#!/usr/bin/env python3

import rclpy
import os
import fnmatch
import serial
import time
from rclpy.node import Node
from std_msgs.msg import UInt8, UInt16
from core_msgs.msg import KillSwitch
from core.utils.config import Topic, NodeConfig

ADS_MAX_VAL = 26096
GAIN_RATIO = 1069 / 1000
OPEN_DRAIN_RATIO = 122 / 22
BAT_MAX_VAL = 16.8
BAT_MIN_VAL = 14.4

class ESPController(Node):
    def __init__(self):
        super().__init__('esp_controller')
        
        self.jetson_batt = 0
        self.motor_batt = 0
        self.depth = 0
        self.dht22_raw = 0
        self.tbs_pwm_in = 0
        self.heading_deg = 0
        self.mux_state = 0
        self.echosounder_dist = 0
        self.echosounder_conf = 0
        
        self.ks_kill_state = KillSwitch()
        self.ks_kill_state.data = KillSwitch.DEFAULT
        
        self.ser_1 = None
        
        # Publishers
        self.kill_switch_pub = Topic.kill_switch.createPublisher(self)
        self.jetson_batt_pub = Topic.jetson_batt.createPublisher(self)
        self.motor_batt_pub = Topic.motor_batt.createPublisher(self)
        self.mux_state_pub = Topic.mux_state.createPublisher(self)
        
        self.jetson_batt_msg = UInt16()
        self.motor_batt_msg = UInt16()
        self.mux_state_msg = UInt8()
        
        self._init_serial()
        self.get_logger().info("ESP Controller Node Started")

    def warn_once(self, msg):
        if not hasattr(self, '_warn_once_messages'):
            self._warn_once_messages = set()
        if msg not in self._warn_once_messages:
            self.get_logger().warn(msg)
            self._warn_once_messages.add(msg)

    def error_throttle(self, period_ms, msg):
        self.get_logger().error(msg, throttle_duration_sec=period_ms/1000.0)

    def _init_serial(self):
        self.get_logger().info("ESP32 Simulation Mode - Mucking sensors")
        self.ser_1 = "Simulated"
        
    @staticmethod
    def _parse_raw(raw_str):
        try:
            return [float(e) for e in raw_str.replace("\r\n", "").split(",")]
        except Exception as e:
            return None

    def _read_sensor_esp32(self):
        if self.ser_1 is None:
            self._init_serial()
            
        # Mock values
        self.jetson_batt = ADS_MAX_VAL * (BAT_MAX_VAL / 100.0) # Mock almost full battery
        self.motor_batt = ADS_MAX_VAL * (BAT_MAX_VAL / 100.0)
        self.mux_state = 0
        
        time.sleep(0.1) # Simulate delay
        return True

    @staticmethod
    def _battery_value(bat_val):
        return (
            (
                (((bat_val / ADS_MAX_VAL) * (OPEN_DRAIN_RATIO) * 3.3) * (GAIN_RATIO)) - (BAT_MIN_VAL)
            ) * 100
        ) / (BAT_MAX_VAL - BAT_MIN_VAL)

    def _battery_value_safe(self, bat_val):
        val = self._battery_value(bat_val)
        val = max(0, min(int(val), 65535))  # clamp to UInt16
        return val

    def main(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self._read_sensor_esp32():
                self.kill_switch_pub.publish(self.ks_kill_state)
                
                self.jetson_batt_msg.data = int(self._battery_value_safe(self.jetson_batt))
                self.motor_batt_msg.data = int(self._battery_value_safe(self.motor_batt))
                self.mux_state_msg.data = int(self.mux_state)
                
                self.jetson_batt_pub.publish(self.jetson_batt_msg)
                self.motor_batt_pub.publish(self.motor_batt_msg)
                self.mux_state_pub.publish(self.mux_state_msg)
            
def main(args=None):
    rclpy.init(args=args)
    node = ESPController()
    try:
        node.main()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
