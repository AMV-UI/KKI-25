#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import traceback

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from core_msgs.msg import KillSwitch, Config

from core.utils.config import NodeConfig, Topic, Param


class PIDController:
    """
    PID Controller class for processing control errors
    """
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=(-400, 400), sample_time=0.02):
        """
        Initialize PID controller
        
        Args:
            kp (float): Proportional gain
            ki (float): Integral gain  
            kd (float): Derivative gain
            output_limits (tuple): Min and max output limits
            sample_time (float): Controller sample time in seconds
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.sample_time = sample_time
        
        # Internal variables
        self._last_error = 0.0
        self._last_time = time.time()
        self._integral = 0.0
        self._last_output = 0.0
        
        # Integral windup prevention
        self._integral_limit = output_limits[1] / max(ki, 0.001)  # Prevent division by zero
        
    def compute(self, setpoint, current_value, dt=None):
        """
        Compute PID output
        
        Args:
            setpoint (float): Desired value
            current_value (float): Current measured value
            dt (float): Time delta (optional, will calculate if not provided)
            
        Returns:
            float: Control output
        """
        current_time = time.time()
        
        if dt is None:
            dt = current_time - self._last_time
        
        # Skip computation if dt is too small
        if dt < self.sample_time:
            return self._last_output
            
        # Calculate error
        error = setpoint - current_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term with windup prevention
        self._integral += error * dt
        self._integral = max(min(self._integral, self._integral_limit), -self._integral_limit)
        integral = self.ki * self._integral
        
        # Derivative term
        if dt > 0:
            derivative = self.kd * (error - self._last_error) / dt
        else:
            derivative = 0.0
            
        # Calculate total output
        output = proportional + integral + derivative
        
        # Apply output limits
        output = max(min(output, self.output_limits[1]), self.output_limits[0])
        
        # Store values for next iteration
        self._last_error = error
        self._last_time = current_time
        self._last_output = output
        
        return output
    
    def reset(self):
        """Reset PID controller internal state"""
        self._last_error = 0.0
        self._last_time = time.time()
        self._integral = 0.0
        self._last_output = 0.0
    
    def set_tunings(self, kp, ki, kd):
        """Update PID tuning parameters"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral_limit = self.output_limits[1] / max(ki, 0.001)


class YawPIDController(Node):
    """
    ROS2 Node for Yaw PID Control
    
    SUBSCRIBES TO:
        - yaw_setpoint: Desired yaw angle/position (Float64)
        - yaw_current: Current yaw angle/position from sensors (Float64)  
        - yaw_error: Direct yaw error input (Float64)
        - config: PID parameter updates (Config)
        
    PUBLISHES:
        - yaw_effort: Control effort output (Float64)
    """
    
    def __init__(self):
        super().__init__('yaw_pid_controller')
        
        self.topic = Topic()
        
        # Initialize PID controller with default parameters
        # You can adjust these values based on your system characteristics
        self.yaw_pid = PIDController(
            kp=2.0,      # Proportional gain - how aggressively to respond to current error
            ki=0.1,      # Integral gain - how aggressively to respond to accumulated error  
            kd=0.05,     # Derivative gain - how aggressively to respond to rate of error change
            output_limits=(-400, 400),  # Output effort limits
            sample_time=0.02  # 50Hz control loop
        )
        
        # Control variables
        self.yaw_setpoint = 0.0
        self.yaw_current = 0.0
        self.yaw_error = 0.0
        self.yaw_effort = 0.0
        
        # Control mode flags
        self.use_direct_error = False  # Set to True if you want to use yaw_error directly
        self.enabled = True
        
        self._setup_communication()
        
    def _setup_communication(self):
        """Initialize ROS2 subscribers and publishers"""
        
        # Subscribers
        self.yaw_setpoint_sub = self.topic.yaw_setpoint.createSubscriber(
            self, self._yaw_setpoint_callback
        )
        self.yaw_current_sub = self.topic.yaw_current.createSubscriber(
            self, self._yaw_current_callback  
        )
        self.yaw_error_sub = self.topic.yaw_error.createSubscriber(
            self, self._yaw_error_callback
        )
        self.config_sub = self.topic.pid_config.createSubscriber(
            self, self._config_callback
        )
        self.killswitch_sub = self.topic.killswitch.createSubscriber(
            self, self._killswitch_callback
        )
        
        # Publishers
        self.yaw_effort_pub = self.topic.yaw_effort.createPublisher(self)
        
        self.get_logger().info("YawPIDController communication setup complete")
        
    def _yaw_setpoint_callback(self, msg: Float64):
        """Update yaw setpoint"""
        self.yaw_setpoint = msg.data
        
    def _yaw_current_callback(self, msg: Float64):
        """Update current yaw measurement"""
        self.yaw_current = msg.data
        
    def _yaw_error_callback(self, msg: Float64):
        """Update yaw error directly (alternative to setpoint/current)"""
        self.yaw_error = msg.data
        self.use_direct_error = True
        
    def _config_callback(self, msg: Config):
        """Update PID parameters from configuration"""
        try:
            # Assuming Config message has fields for PID parameters
            # Adjust based on your actual Config message structure
            if hasattr(msg, 'kp') and hasattr(msg, 'ki') and hasattr(msg, 'kd'):
                self.yaw_pid.set_tunings(msg.kp, msg.ki, msg.kd)
                self.get_logger().info(f"Updated PID parameters: Kp={msg.kp}, Ki={msg.ki}, Kd={msg.kd}")
        except Exception as e:
            self.get_logger().error(f"Error updating PID config: {e}")
            
    def _killswitch_callback(self, msg: KillSwitch):
        """Handle killswitch state"""
        if msg.data == KillSwitch.KILL:
            self.enabled = False
            self.yaw_pid.reset()
            self.get_logger().warn("PID Controller disabled by killswitch")
        else:
            self.enabled = True
            self.get_logger().info("PID Controller enabled")
    
    def compute_yaw_effort(self):
        """Compute yaw control effort using PID"""
        if not self.enabled:
            return 0.0
            
        if self.use_direct_error:
            # Use direct error input (from object detection, etc.)
            # In this case, setpoint is 0 (center) and current is the error
            effort = self.yaw_pid.compute(0.0, self.yaw_error)
        else:
            # Use setpoint and current value
            effort = self.yaw_pid.compute(self.yaw_setpoint, self.yaw_current)
            
        return effort
    
    def run(self):
        """Main execution loop"""
        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz
        self.get_logger().info(f"<> [{NodeConfig.yaw_pid_controller}] Successfully initialized")
        
    def control_loop(self):
        """Main control loop executed at 50Hz"""
        try:
            # Compute PID output
            self.yaw_effort = self.compute_yaw_effort()
            
            # Publish yaw effort
            effort_msg = Float64()
            effort_msg.data = self.yaw_effort
            self.yaw_effort_pub.publish(effort_msg)
            
            # Optional: Log debug info periodically
            # Uncomment the line below for debugging
            # self.get_logger().debug(f"Yaw effort: {self.yaw_effort:.2f}")
            
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {traceback.format_exc()}")
    
    def get_pid_status(self):
        """Get current PID status for debugging"""
        return {
            'kp': self.yaw_pid.kp,
            'ki': self.yaw_pid.ki, 
            'kd': self.yaw_pid.kd,
            'setpoint': self.yaw_setpoint,
            'current': self.yaw_current,
            'error': self.yaw_error if self.use_direct_error else (self.yaw_setpoint - self.yaw_current),
            'effort': self.yaw_effort,
            'enabled': self.enabled
        }


def main(args=None):
    try:
        rclpy.init(args=args)
        
        yaw_pid_controller = YawPIDController()
        yaw_pid_controller.run()
        
        # Spin the node
        rclpy.spin(yaw_pid_controller)
        
    except Exception as e:
        print(f"Error in main: {traceback.format_exc()}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()