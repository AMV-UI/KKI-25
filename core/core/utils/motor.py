#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from core.utils.config import Channel, Param, SPEED


class Motor:
    STANDBY = 1500
    KILL_BUTTON, KILL_ON, KILL_OFF = 1200, 1500, 1800

    def __init__(self, node: Node, offset_horizontal=200, motor_adjust=0):
        self.node = node
        self.FORWARD = self.STANDBY + offset_horizontal
        self.BACKWARD = self.STANDBY - offset_horizontal
        self.motor_adjust = motor_adjust
        self.channel = Channel

        self.node.declare_parameter(Param.MOTOR_SPEED, SPEED.Maximum)
        self.node.declare_parameter(Param.X_SPEED, SPEED.Maximum)

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

    def straight_with_conf(self, control_effort_yaw_motor=100):
        return self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.calculateSpeed(100 + control_effort_yaw_motor),
            self.channel.MOTOR_Y: self.calculateSpeed(100 + control_effort_yaw_motor),
        })

    def autonomous(self, control_effort_x=50, control_effort_y=300):
        motor_speed = self.node.get_parameter(Param.MOTOR_SPEED).value
        x_speed = self.node.get_parameter(Param.X_SPEED).value

        return self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.calculateSpeed(int(control_effort_x * x_speed)),
            self.channel.MOTOR_Y: self.calculateSpeed(int(control_effort_y * motor_speed)),
        })

    def finding_turn(self, control_effort_x=50, control_effort_y=250):
        motor_speed = self.node.get_parameter(Param.MOTOR_SPEED).value
        x_speed = self.node.get_parameter(Param.X_SPEED).value

        return self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.calculateSpeed(int(control_effort_x)),
            self.channel.MOTOR_Y: self.calculateSpeed(int(control_effort_y * motor_speed)),
        })

    def reset_param(self):
        self.node.set_parameters([
            Parameter(Param.MOTOR_SPEED, Parameter.Type.INTEGER, SPEED.Maximum),
            Parameter(Param.X_SPEED, Parameter.Type.INTEGER, SPEED.Maximum),
        ])

    def half_detected(self):
        self.node.set_parameters([
            Parameter(Param.MOTOR_SPEED, Parameter.Type.INTEGER, SPEED.Slow),
            Parameter(Param.X_SPEED, Parameter.Type.INTEGER, SPEED.MediumFast),
        ])

    def one_is_closer_detected(self):
        self.node.set_parameters([
            Parameter(Param.MOTOR_SPEED, Parameter.Type.INTEGER, SPEED.Slow),
            Parameter(Param.X_SPEED, Parameter.Type.INTEGER, SPEED.Medium),
        ])

    def full_detected(self):
        self.node.set_parameters([
            Parameter(Param.MOTOR_SPEED, Parameter.Type.INTEGER, SPEED.MediumFast),
            Parameter(Param.X_SPEED, Parameter.Type.INTEGER, SPEED.MediumFast),
        ])

    def not_detected(self):
        self.node.set_parameters([
            Parameter(Param.MOTOR_SPEED, Parameter.Type.INTEGER, SPEED.Slow),
            Parameter(Param.X_SPEED, Parameter.Type.INTEGER, SPEED.Slow),
        ])

    def searching(self):
        self.node.set_parameters([
            Parameter(Param.MOTOR_SPEED, Parameter.Type.INTEGER, SPEED.Medium),
        ])

    def forward(self):
        if rclpy.ok():
            return {
                self.channel.MOTOR_X: self.STANDBY,
                self.channel.MOTOR_Y: self.STANDBY,
            }

    def backward(self):
        return {
            self.channel.MOTOR_X: self.STANDBY,
            self.channel.MOTOR_Y: self.STANDBY,
        }

    def turnLeft(self):
        return {
            self.channel.MOTOR_X: self.BACKWARD,
            self.channel.MOTOR_Y: self.FORWARD,
        }

    def turnRight(self):
        return {
            self.channel.MOTOR_X: self.FORWARD,
            self.channel.MOTOR_Y: self.BACKWARD,
        }

    def rotateLeft(self):
        return {
            self.channel.MOTOR_X: self.FORWARD,
            self.channel.MOTOR_Y: self.BACKWARD,
        }

    def rotateRight(self):
        return {
            self.channel.MOTOR_X: self.BACKWARD,
            self.channel.MOTOR_Y: self.FORWARD,
        }

    def idle(self):
        return self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.STANDBY,
            self.channel.MOTOR_Y: self.STANDBY,
        })


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('motor_node')
    motor = Motor(node)

    node.get_logger().info("Motor node started.")

    try:
        # Example usage
        motor.autonomous()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
