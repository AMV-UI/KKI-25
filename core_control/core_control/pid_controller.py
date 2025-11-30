from simple_pid import PID
from core.utils.config import Topic
from rclpy.node import Node
import rclpy
import traceback


class PIDController(Node):
    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = 0.0):
        super().__init__('pid_controller')
        self.ki = ki
        self.kp = kp
        self.kd = kd

        self.pid = PID(kp, ki, kd, setpoint=setpoint)
        self.max_val = 300.0
        self.pid.output_limits = (-self.max_val, self.max_val)

    def _init_comms(self):
        self.kp_pub = Topic.kp.createPublisher(self)
        self.ki_pub = Topic.ki.createPublisher(self)
        self.kd_pub = Topic.kd.createPublisher(self)

        self.error_sub = Topic.error.createSubscriber(self,self.error_callback)
        self.get_logger().info("PID Controller Node Initialized")

    def update(self, measurement: float) -> float:
        return self.pid(measurement)

    def adaptive_update(self, measurement: float):
        error = self.pid.setpoint - measurement

        base_kp = self.kp
        base_ki = self.ki
        base_kd = self.kd

        kp = base_kp + abs(error) * 0.5
        ki = base_ki + abs(error) * 0.3
        kd = base_kd

        self.set_gains(kp, ki, kd)
        return self.pid(measurement)

    def set_setpoint(self, setpoint: float):
        self.pid.setpoint = setpoint

    def set_gains(self, kp: float, ki: float, kd: float):
        self.pid.tunings = (kp, ki, kd)
    
    def get_gains(self):
        return self.pid.tunings
    
    def error_callback(self, msg):
        error = msg.data
        self.update(error)
        self.kp_pub.publish(self.pid.kp)
        self.ki_pub.publish(self.pid.ki)
        self.kd_pub.publish(self.pid.kd)
