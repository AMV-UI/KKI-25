#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from core_msgs.msg import AutoControl
from std_msgs.msg import Float64, UInt16
# Import files
from core.utils.config import NodeConfig, Topic, SETPOINT
from simple_pid import PID
import time

class PIDController(Node):
    def __init__(self) -> None:
        super().__init__(NodeConfig.pid_controller)
        
        # Initialize PID controller
        self.pid = PID(0.8, 0.1, 0.8, setpoint=SETPOINT.SETPOINT_YAW)
        self.pid.output_limits = (0, 300)

        self.control = Float64()
        self.state = float(SETPOINT.SETPOINT_YAW)  # Initialize to setpoint
        self.dsc = 0.0
        
        # subscribers
        self.state_subscriber = Topic.state.createSubscriber(self, self.state_callback)
        self.dsc_subscriber = Topic.dsc.createSubscriber(self, self.dsc_callback)
        
        # publisher
        self.yaw_pid_publisher = Topic.yaw_effort.createPublisher(self)
        
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        
        self.get_logger().info(f"[{NodeConfig.pid_controller}] PID Controller initialized")
    
    def state_callback(self, msg):
        """Callback for state updates"""
        self.state = msg.data
    
    def dsc_callback(self, msg):
        """Callback for dsc updates"""
        self.dsc = msg.data
    
    def control_loop(self):
        """Main control loop executed by timer"""

        control_output = self.pid(self.dsc)

        self.control.data = float(abs(control_output))
        self.yaw_pid_publisher.publish(self.control)
        
        # Optional: Log control values periodically
        if hasattr(self, '_last_log_time'):
            if time.time() - self._last_log_time > 5.0:  # Log every 5 seconds
                self.get_logger().info(
                    f"PID Control - Setpoint: {self.pid.setpoint:.2f}, "
                    f"Current: {self.dsc:.2f}, Output: {self.control.data:.2f}"
                )
                self._last_log_time = time.time()
        else:
            self._last_log_time = time.time()

def main(args=None):
    try:
        rclpy.init(args=args)
        pid_controller = PIDController()
        rclpy.spin(pid_controller)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.get_logger('pid_controller').error(f"Error in PID controller: {e}")
    finally:
        if 'pid_controller' in locals():
            pid_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()