#!/usr/bin/env python3

from statistics import variance
import rclpy
from rclpy.node import Node
import traceback
import time

from std_msgs.msg import Bool, Float64, UInt16, UInt32, UInt8
from sensor_msgs.msg import Joy
from core_msgs.msg import (
    KillSwitch,
    AutoControl,
    Config,
    ObjectCount,
    Controller,
    Pwm,
    Option,
)

from core.utils.config import NodeConfig, Topic, Param
from core.utils.motor import Motor
from core.utils.factory import TopicFactory


class MotorController(Node):
    """
    WILL BE USED IN:
    - Core Behavior Tree

    USES:
    - Motor Node in core.utils
    PUBLISHES:
    - PWM to core_control.microcontroller
    """

    def __init__(self):
        super().__init__(NodeConfig.motor_controller)
        
        # Differential Drive System
        # YAW, X component => Turn Right (+), Turn Left (-) => channel 0
        # SPEED, Y component => Forward (+) and Backward (-) => channel 2

        # YAW (Channel 0): 1700=right, 1300=left
        # SPEED (Channel 2): 1700=forward, 1300=backward

        # OFFSET, deltas between value => 1500 with delta 200 meaning that value can vary between 1400 - 1700
   
        self.motor = Motor(self, offset_horizontal=200, motor_adjust=0)

        # Fast Mode, delta nambah 100
        # self.motor = Motor(self, offset_horizontal=200, motor_adjust=100)


        self.object_counted = ObjectCount()
        self.joy_state = Joy()
        self.is_killed = False

        self.pwm = Pwm()

        self.killswitch_state = KillSwitch()
        self.killswitch_state.data = KillSwitch.DEFAULT

        self.dsc = float()
        self.dsc_flag = float()
        self.state_dst = float()
        self.yaw_effort = float()
        self.speed_effort = float()

        self._setup_communication()

    def _setup_communication(self):
        """Initialize all ROS2 subscribers and publishers"""

        ##OLD
        # # Subscribers
        # self.dsc_control_effort_sub = self.topic.dsc_control_effort.createSubscriber(self, self._dsc_control_effort_callback)
        # self.mission_sub = self.topic.auto_control.createSubscriber(self, self._autocontrol_callback)
        # self.object_counted_subscriber = self.topic.object_counted.createSubscriber(self, self._obj_counted_callback)
        # self.dsc_subscriber = self.topic.dsc.createSubscriber(self, self._dsc_callback)
        # self.state_dst_sub = self.topic.state_dst.createSubscriber(self, self._state_dst_callback)
        # self.mission_subscriber = self.topic.mission.createSubscriber(self, self.mission_callback)
        
        # # Publishers
        # self.pwm_pub = self.topic.pwm.createPublisher(self)

        ##NEW
        # Subscribers
        self.yaw_effort_sub = Topic.yaw_effort.createSubscriber(self, self._yaw_effort_callback)
        self.speed_effort_sub = Topic.speed_effort.createSubscriber(self, self._speed_effort_callback)

        # Publishers
        self.pwm_pub = Topic.pwm.createPublisher(self)

    def display_pwm_status(self):
        """Display current PWM configuration"""
        status = self.motor.get_pwm_status()
        self.get_logger().info("PWM Configuration:")
        self.get_logger().info(f"  - Range: {status['pwm_range']['min']} to {status['pwm_range']['max']}")
        self.get_logger().info(f"  - Standby: {status['pwm_range']['standby']}")
        self.get_logger().info(f"  - Forward: {status['pwm_range']['forward']}")
        self.get_logger().info(f"  - Backward: {status['pwm_range']['backward']}")
        self.get_logger().info(f"  - Motor Adjust: {status['motor_adjust']}")
        self.get_logger().info("=" * 60)

    def manual(self):
        self.get_logger().info(f"<=> [{NodeConfig.motor_controller}] MotorController Manual Mode")
        for k, v in self.motor.idle().items():
            self.pwm.channels[k] = v

    def auto(self):
        self.get_logger().info(f"<=> [{NodeConfig.motor_controller}] MotorController Auto Mode")
        for k, v in self.motor.autonomous(
            yaw_effort=self.yaw_effort,
            speed_effort=self.speed_effort
        ).items():
            self.pwm.channels[k] = v


    def _yaw_effort_callback(self, msg: Float64):
        self.yaw_effort = msg.data

    def _speed_effort_callback(self, msg: Float64):
        self.speed_effort = msg.data

    def _killswitch_callback(self, msg: KillSwitch):
        self.killswitch_state.data = msg.data

    def _joy_state_callback(self, msg: Controller):
        self.joy_state = msg


    def run(self):
        """Main execution loop"""
        # Display initial PWM status
        self.display_pwm_status()

        self.timer = self.create_timer(0.02, self.loop)  # 50Hz
        
        self.get_logger().info(f"<> [{NodeConfig.motor_controller}] Successfully initialized node")

    def loop(self):
        """Main control loop executed at 50Hz"""
        try:

            self.pwm.channels = [int(val) for val in self.pwm.channels]

            self.pwm_pub.publish(self.pwm)
            
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {traceback.format_exc()}")

    def test_sequence(self, timer):
        """Test sequence to validate motor functionality"""
        self.get_logger().info("Starting motor test sequence...")
        
        try:
            # Move forward
            self.get_logger().info("Moving forward...")
            for k, v in self.motor.autonomous(control_effort_x=0, control_effort_y=100).items():
                self.pwm.channels[k] = v
            self.pwm_pub.publish(self.pwm)
            self.get_logger().info(f"Published PWM: {self.pwm.channels}")
            time.sleep(timer)

            # Output PWM with Motor config => Motor(self, offset_horizontal=200, motor_adjust=0)
                                              # control_effort_x = 0, control_effort_y = 100

            # 1500 => index 0
            # 0
            # 1600 => index 2
            # 0
            # 0
            # 0
            # 0
            # 0


            # Move backward
            self.get_logger().info("Moving backward...")
            for k, v in self.motor.autonomous(control_effort_x=0, control_effort_y=-100).items():
                self.pwm.channels[k] = v
            self.pwm_pub.publish(self.pwm)
            self.get_logger().info(f"Published PWM: {self.pwm.channels}")
            time.sleep(timer)

            # Output PWM with Motor config => Motor(self, offset_horizontal=200, motor_adjust=0)
                                            # control_effort_x = 0, control_effort_y = -100
            # 1500 => index 0
            # 0
            # 1400 => index 2
            # 0
            # 0
            # 0
            # 0
            # 0

            # Turn right with moving forward
            self.get_logger().info("Turning right with moving forward")
            for k, v in self.motor.autonomous(control_effort_x=100, control_effort_y=100).items():
                self.pwm.channels[k] = v
            self.pwm_pub.publish(self.pwm)
            self.get_logger().info(f"Published PWM: {self.pwm.channels}")
            time.sleep(timer)

            # Output PWM with Motor config => Motor(self, offset_horizontal=200, motor_adjust=0)
                                            # control_effort_x = 100, control_effort_y = 100
            # 1600 => index 0
            # 0
            # 1600 => index 2
            # 0
            # 0
            # 0
            # 0
            # 0

            # Turn left with moving forward
            self.get_logger().info("Turning left with moving forward")
            for k, v in self.motor.autonomous(control_effort_x=-100, control_effort_y=100).items():
                self.pwm.channels[k] = v
            self.pwm_pub.publish(self.pwm)
            self.get_logger().info(f"Published PWM: {self.pwm.channels}")
            time.sleep(timer)

            # Output PWM with Motor config => Motor(self, offset_horizontal=200, motor_adjust=0)
                                            # control_effort_x = -100, control_effort_y = 100
            # 1400 => index 0
            # 0
            # 1600 => index 2
            # 0
            # 0
            # 0
            # 0
            # 0

            # Idle Backward
            self.get_logger().info("Idle backward")
            for k, v in self.motor.autonomous(control_effort_x=0, control_effort_y=-100).items():
                self.pwm.channels[k] = v
            self.pwm_pub.publish(self.pwm)
            self.get_logger().info(f"Published PWM: {self.pwm.channels}")
            time.sleep(timer)

            # Output PWM with Motor config => Motor(self, offset_horizontal=200, motor_adjust=0)
                                            # control_effort_x = 0, control_effort_y = -100
            # 1500 => index 0
            # 0
            # 1400 => index 2
            # 0
            # 0
            # 0
            # 0
            # 0

            # Stop
            self.get_logger().info("Stop")
            for k, v in self.motor.autonomous(control_effort_x=0, control_effort_y=0).items():
                self.pwm.channels[k] = v
            self.pwm_pub.publish(self.pwm)
            self.get_logger().info(f"Published PWM: {self.pwm.channels}")
            time.sleep(timer)

            # Output PWM with Motor config => Motor(self, offset_horizontal=200, motor_adjust=0)
                                            # control_effort_x = 0, control_effort_y = 0
            # 1500 => index 0
            # 0
            # 1500 => index 2
            # 0
            # 0
            # 0
            # 0
            # 0



            self.get_logger().info("Motor test sequence completed successfully.")

        except Exception as e:
            self.get_logger().error(f"Error during motor test sequence: {traceback.format_exc()}")


def main(args=None):
    try:
        rclpy.init(args=args)

        motor_control = MotorController()
        motor_control.test_sequence(10)
        rclpy.spin(motor_control)
        
    except Exception as e:
        print(f"Error in main: {traceback.format_exc()}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()