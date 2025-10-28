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

        # PARAM EXAMPLES
        # self.create_example = Param.TRACK.createParam(self.node, default_value=self.track)
        # self.change_example = Param.TRACK.setParam(self.node, "A")
        # self.get_example = Param.TRACK.getValue(self.node)

        Param.MOTOR_SPEED.createParam(self.node, default_value=SPEED.Maximum)
        Param.X_SPEED.createParam(self.node, default_value=SPEED.Maximum)

        self

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
        # Use getValue() instead of getParam() to get the actual value
        motor_speed = Param.MOTOR_SPEED.getValue(self.node)
        x_speed = Param.X_SPEED.getValue(self.node)

        self.node.get_logger().debug(f"Autonomous Mode: motor_speed={motor_speed}, x_speed={x_speed}")
        return self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.calculateSpeed(int(control_effort_x * x_speed)),
            self.channel.MOTOR_Y: self.calculateSpeed(int(control_effort_y * motor_speed)),
        })

    def finding_turn(self, control_effort_x=50, control_effort_y=250):
        # Use getValue() instead of getParam()
        motor_speed = Param.MOTOR_SPEED.getValue(self.node)

        return self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.calculateSpeed(int(control_effort_x)),
            self.channel.MOTOR_Y: self.calculateSpeed(int(control_effort_y * motor_speed)),
        })

    def reset_param(self):
        Param.MOTOR_SPEED.setParam(self.node, SPEED.Maximum)
        Param.X_SPEED.setParam(self.node, SPEED.Maximum)

    def half_detected(self):
        Param.MOTOR_SPEED.setParam(self.node, SPEED.Slow)
        Param.X_SPEED.setParam(self.node, SPEED.MediumFast)

    def one_is_closer_detected(self):
        Param.MOTOR_SPEED.setParam(self.node, SPEED.Slow)
        Param.X_SPEED.setParam(self.node, SPEED.Medium)

    def full_detected(self): 
        Param.MOTOR_SPEED.setParam(self.node, SPEED.MediumFast)
        Param.X_SPEED.setParam(self.node, SPEED.MediumFast)
        

    #TODO: Finish the Conversion of the Param Factory
    def not_detected(self):
        Param.MOTOR_SPEED.setParam(self.node, SPEED.Slow)
        Param.X_SPEED.setParam(self.node, SPEED.Slow)

    
    def searching(self):
        Param.MOTOR_SPEED.setParam(self.node, SPEED.MediumFast)
        
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
        motor.autonomous()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
