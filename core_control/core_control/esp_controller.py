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

    def _get_serial_ports(self):
        dirs = []
        try:
            list_of_files = os.listdir("/dev")
            for entry in list_of_files:
                if fnmatch.fnmatch(entry, "ttyACM*") or fnmatch.fnmatch(entry, "ttyUSB*"):
                    dirs.append(f"/dev/{entry}")
        except Exception:
            pass
        return dirs

    def _init_serial(self):
        ports = self._get_serial_ports()
        if not ports:
            self.error_throttle(5000, "No USB serial ports found")
            return

        for port in ports:
            try:
                ser = serial.Serial(port, 115200, timeout=1)   
                start_time = time.time()
                while time.time() - start_time < 2.0:
                    if ser.in_waiting:
                        line = ser.readline().decode(errors='ignore')
                        if line.startswith('s'):
                            self.ser_1 = ser
                            self.get_logger().info(f"ESP32 found on {port}")
                            return
                ser.close()
            except Exception as e:
                self.get_logger().warn(f"Failed to open {port}: {e}")
        
    @staticmethod
    def _parse_raw(raw_str):
        try:
            return [float(e) for e in raw_str.replace("\r\n", "").split(",")]
        except Exception as e:
            return None

    def _read_sensor_esp32(self):
        if self.ser_1 is None:
            self._init_serial()
            return False

        parsed_data = None
        try:
            if self.ser_1.in_waiting:
                raw_ser_1 = self.ser_1.readline().decode(errors='ignore')
                if raw_ser_1.startswith("s"):
                    parsed_data = self._parse_raw(raw_ser_1[1:])
        except Exception as e:
            self.error_throttle(1000, f"Serial read error: {e}")
            self.ser_1.close()
            self.ser_1 = None
            return False

        if not parsed_data:
            return False

        try:
            (
                self.jetson_batt,
                self.motor_batt,
                depth,
                ks_state,
                self.dht22_raw,
                self.tbs_pwm_in,
                self.mux_state,
                heading_deg,
                self.echosounder_dist,
                self.echosounder_conf,
            ) = parsed_data
            
            
        except ValueError:
            return False

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
