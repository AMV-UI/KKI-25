#!/usr/bin/env python3

import rclpy
from core.utils.config import Param, SPEED
from core.utils.config import Channel

from rclpy.node import Node

# TODO: MAKE PID CONTROLLER SCRIPTS BASED ON VISION AND PX

class Motor(Node):
    # T200 Thruster PWM Constants (optimized)
    STANDBY = 1500      # Neutral position
    PWM_MIN = 1000      # Maximum reverse thrust
    PWM_MAX = 2000      # Maximum forward thrust
    
    # Deadband for precise control (optional)
    PWM_DEADBAND_LOW = 1480   # Below this = reverse
    PWM_DEADBAND_HIGH = 1520  # Above this = forward
    
    # Kill switch values
    KILL_BUTTON, KILL_ON, KILL_OFF = 1200, 1500, 1800

    def __init__(self, delta_effort=100, additional_effort=0):  # Reduced for T200
        super().__init__('motor')
        """
        delta_effort: Speed adjustment from standby (e.g., 100 for T200)
        additional_effort: Fine-tuning adjustment (e.g., 0 for T200)
        Channel: MOTOR_X (left), MOTOR_Y (right)
        """
        self.FORWARD = self.STANDBY + delta_effort
        self.BACKWARD = self.STANDBY - delta_effort
        self.motor_adjust = additional_effort

        self.channel = Channel
        
        self.declare_parameter(Param.MOTOR_SPEED, SPEED.Maximum)
        self.declare_parameter(Param.X_SPEED, SPEED.Maximum)

    def _validate_pwm(self, pwm_value, motor_name="Unknown"):
        """
        Validate PWM value and clamp to T200 safe range
        """
        if pwm_value < self.PWM_MIN:
            self.get_logger().warn(
                f"{motor_name} PWM too low: {pwm_value}, clamping to {self.PWM_MIN}"
            )
            return self.PWM_MIN
        elif pwm_value > self.PWM_MAX:
            self.get_logger().warn(
                f"{motor_name} PWM too high: {pwm_value}, clamping to {self.PWM_MAX}"
            )
            return self.PWM_MAX
        return pwm_value

    def _is_in_deadband(self, pwm_value):
        """Check if PWM value is in neutral deadband"""
        return self.PWM_DEADBAND_LOW <= pwm_value <= self.PWM_DEADBAND_HIGH

    def _check_and_log_pwm(self, motor_commands, action_name="Unknown"):
        """
        Check PWM values and validate for both motors and log the information
        """
        left_motor_pwm = motor_commands.get(self.channel.MOTOR_X, self.STANDBY)
        right_motor_pwm = motor_commands.get(self.channel.MOTOR_Y, self.STANDBY)

        validated_left = self._validate_pwm(left_motor_pwm, "Left Motor")
        validated_right = self._validate_pwm(right_motor_pwm, "Right Motor")

        self.get_logger().info(
            f"Action: {action_name} | "
            f"Left Motor PWM: {validated_left} | "
            f"Right Motor PWM: {validated_right}"
        )
        

        self._check_motor_safety(validated_left, validated_right, action_name)

        return {
            self.channel.MOTOR_X: validated_left,
            self.channel.MOTOR_Y: validated_right
        }

    def _check_motor_safety(self, left_pwm, right_pwm, action_name):
        """
        Check for potential safety issues with T200 motor commands
        """
        # Check for extreme values (beyond 80% throttle)
        max_safe_forward = self.STANDBY + (self.PWM_MAX - self.STANDBY) * 0.8
        max_safe_reverse = self.STANDBY - (self.STANDBY - self.PWM_MIN) * 0.8 
        
        if left_pwm > max_safe_forward or left_pwm < max_safe_reverse:
            self.get_logger().warn(f"Left motor high thrust detected in {action_name}: {left_pwm}")
        
        if right_pwm > max_safe_forward or right_pwm < max_safe_reverse:
            self.get_logger().warn(f"Right motor high thrust detected in {action_name}: {right_pwm}")
        
        # Check for motor synchronization issues
        speed_diff = abs(left_pwm - right_pwm)
        if speed_diff > 100 and action_name in ["forward", "backward"]:
            self.get_logger().warn(
                f"Large speed difference between motors in {action_name}: "
                f"Left={left_pwm}, Right={right_pwm}, Diff={speed_diff}"
            )
        
        # Check deadband violations for precise control
        if self._is_in_deadband(left_pwm) and action_name != "idle":
            self.get_logger().info(f"Left motor in deadband during {action_name}")
        if self._is_in_deadband(right_pwm) and action_name != "idle":
            self.get_logger().info(f"Right motor in deadband during {action_name}")

    def __adjust(self, input_pwm, adjust):
        if input_pwm > self.STANDBY:
            return input_pwm + adjust
        elif input_pwm < self.STANDBY:
            return input_pwm - adjust
        return self.STANDBY

    def __calcAdjustedSpeed(self, res):
        res[self.channel.MOTOR_X] = self.__adjust(
            res[self.channel.MOTOR_X], self.motor_adjust
        )
        res[self.channel.MOTOR_Y] = self.__adjust(
            res[self.channel.MOTOR_Y], self.motor_adjust
        )
        return res

    def getChannels(self):
        return self.channel

    def calculateSpeed(self, control_effort):
        return self.STANDBY + control_effort

    def forward(self):
        """Move forward with both motors"""
        motor_commands = self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.FORWARD,
            self.channel.MOTOR_Y: self.FORWARD,
        })
        return self._check_and_log_pwm(motor_commands, "forward")

    def backward(self):
        """Move backward with both motors"""
        motor_commands = self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.BACKWARD,
            self.channel.MOTOR_Y: self.BACKWARD,
        })
        return self._check_and_log_pwm(motor_commands, "backward")

    def turnLeft(self):
        """Turn left (left motor backward, right motor forward)"""
        motor_commands = self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.BACKWARD,
            self.channel.MOTOR_Y: self.FORWARD,
        })
        return self._check_and_log_pwm(motor_commands, "turnLeft")

    def turnRight(self):
        """Turn right (left motor forward, right motor backward)"""
        motor_commands = self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.FORWARD,
            self.channel.MOTOR_Y: self.BACKWARD,
        })
        return self._check_and_log_pwm(motor_commands, "turnRight")

    def rotateLeft(self):
        """Rotate left (same as turnRight for differential drive)"""
        motor_commands = self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.FORWARD,
            self.channel.MOTOR_Y: self.BACKWARD,
        })
        return self._check_and_log_pwm(motor_commands, "rotateLeft")

    def rotateRight(self):
        """Rotate right (same as turnLeft for differential drive)"""
        motor_commands = self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.BACKWARD,
            self.channel.MOTOR_Y: self.FORWARD,
        })
        return self._check_and_log_pwm(motor_commands, "rotateRight")

    def idle(self):
        """Stop both motors"""
        motor_commands = self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.STANDBY,
            self.channel.MOTOR_Y: self.STANDBY,
        })
        return self._check_and_log_pwm(motor_commands, "idle")

    def straight_with_conf(self, control_effort_yaw_motor=100):
        """Move straight with yaw correction"""
        motor_commands = self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.calculateSpeed(100 + control_effort_yaw_motor),
            self.channel.MOTOR_Y: self.calculateSpeed(100 + control_effort_yaw_motor),
        })
        return self._check_and_log_pwm(motor_commands, "straight_with_conf")

    def autonomous(self, yaw_effort=0, speed_effort=0):
        """
        Autonomous movement using PID outputs:
        - yaw_effort: correction from heading PID (left vs right bias), the higher the value, the sharper right turn
        - speed_effort: correction from speed PID (overall forward/backward throttle)

        Example:
        motor.autonomous(yaw_effort=0, speed_effort=100)
        Result: Both motors at 1600 PWM (equal thrust forward)

        motor.autonomous(yaw_effort=50, speed_effort=100)
        # Result: 
        # Left motor: 1500 + (100-50) = 1550 PWM
        # Right motor: 1500 + (100+50) = 1650 PWM
        # Vehicle moves forward while turning right
        """
        
        left_pwm = self.calculateSpeed(speed_effort - yaw_effort)
        right_pwm = self.calculateSpeed(speed_effort + yaw_effort)

        motor_commands = self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: left_pwm,
            self.channel.MOTOR_Y: right_pwm,
        })
        return self._check_and_log_pwm(motor_commands, "autonomous")


    def set_speed_profile(self, profile_name):
        speed_profiles = {
            'reset': (SPEED.Maximum, SPEED.Maximum),
            'half_detected': (SPEED.MediumFast, SPEED.Medium),
            'one_is_closer_detected': (SPEED.Medium, SPEED.Medium),
            'full_detected': (SPEED.Medium, SPEED.Maximum),
            'not_detected': (SPEED.Idle, SPEED.Slow),
            'searching': (SPEED.Medium, SPEED.Medium),
        }
        
        if profile_name in speed_profiles:
            motor_speed, x_speed = speed_profiles[profile_name]
            try:
                self.set_parameters([
                    rclpy.parameter.Parameter(Param.MOTOR_SPEED, rclpy.Parameter.Type.INTEGER, motor_speed),
                    rclpy.parameter.Parameter(Param.X_SPEED, rclpy.Parameter.Type.INTEGER, x_speed)
                ])
                self.get_logger().info(f"Speed profile set to: {profile_name}")
            except Exception as e:
                self.get_logger().error(f"Failed to set speed profile {profile_name}: {e}")
        else:
            self.get_logger().error(f"Unknown speed profile: {profile_name}")

    def get_pwm_status(self):
        """Get current PWM status and ranges"""
        status = {
            'pwm_range': {
                'min': self.PWM_MIN,
                'max': self.PWM_MAX,
                'standby': self.STANDBY,
                'forward': self.FORWARD,
                'backward': self.BACKWARD
            },
            'motor_adjust': self.motor_adjust,
            'channels': {
                'motor_x': self.channel.MOTOR_X,
                'motor_y': self.channel.MOTOR_Y
            }
        }
        return status

    # Keep static methods for backward compatibility but log warnings
    @staticmethod
    def reset_param():
        node = rclpy.create_node('motor_reset_param')
        node.get_logger().warn("Using deprecated static method. Use set_speed_profile('reset') instead.")
        node.declare_parameter(Param.MOTOR_SPEED, SPEED.Maximum)
        node.declare_parameter(Param.X_SPEED, SPEED.Maximum)
        node.set_parameters([
            rclpy.parameter.Parameter(Param.MOTOR_SPEED, rclpy.Parameter.Type.INTEGER, SPEED.Maximum),
            rclpy.parameter.Parameter(Param.X_SPEED, rclpy.Parameter.Type.INTEGER, SPEED.Maximum)
        ])
        node.destroy_node()

    @staticmethod
    def half_detected():
        node = rclpy.create_node('motor_half_detected')
        node.get_logger().warn("Using deprecated static method. Use set_speed_profile('half_detected') instead.")
        node.set_parameters([
            rclpy.parameter.Parameter(Param.MOTOR_SPEED, rclpy.Parameter.Type.INTEGER, SPEED.MediumFast),
            rclpy.parameter.Parameter(Param.X_SPEED, rclpy.Parameter.Type.INTEGER, SPEED.Medium)
        ])
        node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    motor = Motor()
    rclpy.spin(motor)
    motor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
