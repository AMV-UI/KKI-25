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

        # Param.MOTOR_SPEED.createParam(self.node, default_value=SPEED.Maximum)
        # Param.X_SPEED.createParam(self.node, default_value=SPEED.Maximum)

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

    def calculateSpeed(self, control_effort):
        return self.STANDBY + control_effort

    def autonomous(self, control_effort_x=50, control_effort_y=300):
        # motor_speed = Param.MOTOR_SPEED.getValue(self.node)
        # x_speed = Param.X_SPEED.getValue(self.node)

        motor_speed = 1
        x_speed = 1 


        self.node.get_logger().debug(f"Autonomous Mode: motor_speed={motor_speed}, x_speed={x_speed}")
        return self.__calcAdjustedSpeed({
            self.channel.MOTOR_X: self.calculateSpeed(int(control_effort_x * x_speed)),
            self.channel.MOTOR_Y: self.calculateSpeed(int(control_effort_y * motor_speed)),
        })

    # def half_detected(self):
    #     Param.MOTOR_SPEED.setParam(self.node, SPEED.Slow)
    #     Param.X_SPEED.setParam(self.node, SPEED.MediumFast)

    # def one_is_closer_detected(self):
    #     Param.MOTOR_SPEED.setParam(self.node, SPEED.Slow)
    #     Param.X_SPEED.setParam(self.node, SPEED.Medium)

    # def full_detected(self): 
    #     Param.MOTOR_SPEED.setParam(self.node, SPEED.MediumFast)
    #     Param.X_SPEED.setParam(self.node, SPEED.MediumFast)

    # def not_detected(self):
    #     Param.MOTOR_SPEED.setParam(self.node, SPEED.Slow)
    #     Param.X_SPEED.setParam(self.node, SPEED.Slow)

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
