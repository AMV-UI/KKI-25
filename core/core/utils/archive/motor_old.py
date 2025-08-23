#!/usr/bin/env python3

from utils.config import Channel, Param, SPEED
import rospy


class Motor:
    STANDBY = 1500
    # SERVO_LEFT, SERVO_RIGHT, SERVO_STANDBY = 2200, 800, 1500 # TODO : Awaiting for trial
    KILL_BUTTON, KILL_ON, KILL_OFF = 1200, 1500, 1800

    def __init__(self, offset_horizontal=200, motor_adjust=0):
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

    def autonomous(self, control_effort_x=50, control_effort_y=300):
        motor_speed = rospy.get_param(Param.MOTOR_SPEED)
        x_speed = rospy.get_param(Param.X_SPEED)

        return self.__calcAdjustedSpeed(
            {
                self.channel.MOTOR_X: self.calculateSpeed(
                    int(control_effort_x * x_speed)
                ),
                self.channel.MOTOR_Y: self.calculateSpeed(
                    int(control_effort_y * motor_speed)
                ),  # dikasih minus
            }
        )

    def finding_turn(self, control_effort_x=50, control_effort_y=250):
        motor_speed = rospy.get_param(Param.MOTOR_SPEED)
        x_speed = rospy.get_param(Param.X_SPEED)

        return self.__calcAdjustedSpeed(
            {
                self.channel.MOTOR_X: self.calculateSpeed(int(control_effort_x)),
                self.channel.MOTOR_Y: self.calculateSpeed(
                    int(control_effort_y * motor_speed)
                ),  # dikasih minus
            }
        )

    @staticmethod
    def reset_param():
        rospy.set_param(Param.MOTOR_SPEED, SPEED.Maximum)
        rospy.set_param(Param.X_SPEED, SPEED.Maximum)

    @staticmethod
    def half_detected():
        rospy.set_param(Param.MOTOR_SPEED, SPEED.Slow)
        rospy.set_param(Param.X_SPEED, SPEED.MediumFast)

    @staticmethod
    def one_is_closer_detected():
        rospy.set_param(Param.MOTOR_SPEED, SPEED.Slow)
        rospy.set_param(Param.X_SPEED, SPEED.Medium)

    @staticmethod
    def full_detected():
        rospy.set_param(Param.MOTOR_SPEED, SPEED.MediumFast)
        rospy.set_param(Param.X_SPEED, SPEED.MediumFast)

    @staticmethod
    def not_detected():
        rospy.set_param(Param.MOTOR_SPEED, SPEED.Slow)
        rospy.set_param(Param.X_SPEED, SPEED.Slow)

    @staticmethod
    def searching():
        rospy.set_param(Param.MOTOR_SPEED, SPEED.Medium)

    def forward(self):
        while not rospy.is_shutdown():
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
