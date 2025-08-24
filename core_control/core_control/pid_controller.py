#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, UInt16
# Import files
from core.utils.config import NodeConfig, Topic, SETPOINT
from simple_pid import PID
import time

class PIDController(Node):
    def __init__(self) -> None:
        """
        PID Controller for Yaw Control
        Publishes to:
        - /core/pid_controller/control_effort
        Subscribes to:
        - /core/pid_controller/state (current yaw)
        - /core/pid_controller/distance (current distance to setpoint)
        """
        super().__init__(NodeConfig.pid_controller)
        
        self.pid = PID(0.8, 0.1, 0.8, setpoint=SETPOINT.SETPOINT_YAW)
        self.pid.output_limits = (-300, 300)  # Allow negative values for bidirectional control
        #need to be tested
        
        self.control = Float64()
        self.yaw_state = float(SETPOINT.SETPOINT_YAW)
        self.yaw_dsc = 0.0
        
        # subscribers
        # self.yaw_state_subscriber = Topic.yaw_state.createSubscriber(self, self.yaw_state_callback)
        self.yaw_dsc_subscriber = Topic.yaw_dsc.createSubscriber(self, self.yaw_dsc_callback)
        
        # publisher
        self.yaw_pid_publisher = Topic.yaw_effort.createPublisher(self)
        
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        self.get_logger().info(f"[{NodeConfig.pid_controller}] PID Controller initialized")

    # def yaw_state_callback(self, msg):
    #     """Callback for yaw state updates"""
    #     self.yaw_state = msg.data

    def yaw_dsc_callback(self, msg):
        """Callback for yaw dsc updates"""
        self.yaw_dsc = msg.data

    def control_loop(self):
        """Main control loop executed by timer"""
        control_output = self.pid(self.yaw_dsc)
        self.control.data = float(control_output)
        self.yaw_pid_publisher.publish(self.control)
        
        # # Optional: Log control values periodically
        # if not hasattr(self, '_last_log_time'):
        #     self._last_log_time = time.time()
        # elif time.time() - self._last_log_time > 5.0:  # Log every 5 seconds
        #     self.get_logger().info(
        #         f"PID Control - Setpoint: {self.pid.setpoint:.2f}, "
        #         f"Current: {self.yaw_dsc:.2f}, Output: {self.control.data:.2f}"
        #     )
        #     self._last_log_time = time.time()

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