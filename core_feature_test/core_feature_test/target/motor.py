#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class Channel:
    MOTOR_X = 0
    MOTOR_Y = 2

class Param:
    KP = "/yaw_controller/yaw_controller/Kp"
    KI = "/yaw_controller/yaw_controller/Ki"
    KD = "/yaw_controller/yaw_controller/Kd"
    THRESHOLD = "/camera_front/threshold"
    TRACK = "/"
    BOW_SPEED = "/motor_controller/bow_speed"
    X_SPEED = "/motor_controller/x_speed"
    MOTOR_SPEED = "/motor_controller/motor_speed"

class SPEED:
    Maximum = 1
    MediumFast = 0.8
    Medium = 0.5
    Slow = 0.4
    Idle = 0

class Motor(Node):
    STANDBY = 1500
    # SERVO_LEFT, SERVO_RIGHT, SERVO_STANDBY = 2200, 800, 1500 # TODO : Awaiting for trial
    KILL_BUTTON, KILL_ON, KILL_OFF = 1200, 1500, 1800

    def __init__(self, offset_horizontal=200, motor_adjust=0):
        super().__init__('motor')
        self.FORWARD = self.STANDBY + offset_horizontal
        self.BACKWARD = self.STANDBY - offset_horizontal
        self.motor_adjust = motor_adjust

        self.channel = Channel

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
        return self.__calcAdjustedSpeed(
            {
                self.channel.MOTOR_X: self.calculateSpeed(
                    100 + control_effort_yaw_motor
                ),
                self.channel.MOTOR_Y: self.calculateSpeed(
                    100 + control_effort_yaw_motor
                ),
            }
        )

    def autonomous(self, control_effort_x=50):
        motor_speed = self.get_parameter(Param.MOTOR_SPEED).value
        x_speed = self.get_parameter(Param.X_SPEED).value
        
        return self.__calcAdjustedSpeed(
            {
                self.channel.MOTOR_X: self.calculateSpeed(
                    int(control_effort_x * x_speed)

                ),
                self.channel.MOTOR_Y: self.calculateSpeed(
                    int(200 * motor_speed)
                ),  # dikasih minus
            }
        )

    @staticmethod
    def reset_param():
        node = rclpy.create_node('motor_reset_param')
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
        node.set_parameters([
            rclpy.parameter.Parameter(Param.MOTOR_SPEED, rclpy.Parameter.Type.INTEGER, SPEED.MediumFast),
            rclpy.parameter.Parameter(Param.X_SPEED, rclpy.Parameter.Type.INTEGER, SPEED.Medium)
        ])
        node.destroy_node()

    @staticmethod
    def one_is_closer_detected():
        node = rclpy.create_node('motor_one_is_closer_detected')
        node.set_parameters([
            rclpy.parameter.Parameter(Param.MOTOR_SPEED, rclpy.Parameter.Type.INTEGER, SPEED.Medium),
            rclpy.parameter.Parameter(Param.X_SPEED, rclpy.Parameter.Type.INTEGER, SPEED.Medium)
        ])
        node.destroy_node()

    @staticmethod
    def full_detected():
        node = rclpy.create_node('motor_full_detected')
        node.set_parameters([
            rclpy.parameter.Parameter(Param.MOTOR_SPEED, rclpy.Parameter.Type.INTEGER, SPEED.Medium),
            rclpy.parameter.Parameter(Param.X_SPEED, rclpy.Parameter.Type.INTEGER, SPEED.Maximum)
        ])
        node.destroy_node()

    @staticmethod
    def not_detected():
        node = rclpy.create_node('motor_not_detected')
        node.set_parameters([
            rclpy.parameter.Parameter(Param.MOTOR_SPEED, rclpy.Parameter.Type.INTEGER, SPEED.Idle),
            rclpy.parameter.Parameter(Param.X_SPEED, rclpy.Parameter.Type.INTEGER, SPEED.Slow)
        ])
        node.destroy_node()

    @staticmethod
    def searching():
        node = rclpy.create_node('motor_searching')
        node.set_parameters([
            rclpy.parameter.Parameter(Param.MOTOR_SPEED, rclpy.Parameter.Type.INTEGER, SPEED.Medium)
        ])
        node.destroy_node()

    def forward(self):
        while rclpy.ok():
            return {
                self.channel.MOTOR_X: self.STANDBY,  # DIBALIK
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
        return self.__calcAdjustedSpeed(
            {
                self.channel.MOTOR_X: self.STANDBY,
                self.channel.MOTOR_Y: self.STANDBY,
            }
        )


def main(args=None):
    rclpy.init(args=args)
    motor = Motor()
    rclpy.spin(motor)
    motor.destroy_node()
    rclpy.shutdown()
