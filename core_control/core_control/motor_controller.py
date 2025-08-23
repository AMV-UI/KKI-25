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

from core.utils.config import Node as NodeConfig, Topic, Param
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
        super().__init__('motor_controller')
        
        self.motor = Motor(delta_effort=150, additional_effort=40)

        self.object_counted = ObjectCount()
        self.joy_state = Joy()
        self.is_killed = False

        self.pwm = Pwm()

        self.killswitch_state = KillSwitch()
        self.killswitch_state.data = KillSwitch.DEFAULT

        self.dsc = float()
        self.dsc_flag = float()
        self.state_dst = float()
        self.yaw_control_effort = float()
        self.dsc_control_effort = float()

        self._setup_communication()

    def _setup_communication(self):
        """Initialize all ROS2 subscribers and publishers"""
        # Subscribers
        self.dsc_control_effort_sub = TopicFactory('control_effort_dsc', Float64).createSubscriber(self, self._dsc_control_effort_callback)
        self.mission_sub = TopicFactory('auto_control', AutoControl).createSubscriber(self, self._autocontrol_callback)
        self.object_counted_subscriber = TopicFactory('object_counted', ObjectCount).createSubscriber(self, self._obj_counted_callback)
        self.dsc_subscriber = TopicFactory('dsc', Float64).createSubscriber(self, self._dsc_callback)
        self.state_dst_sub = TopicFactory('state_dst', Float64).createSubscriber(self, self._state_dst_callback)
        self.mission_subscriber = TopicFactory('mission', UInt8).createSubscriber(self, self.mission_callback)
        
        # Publishers
        self.pwm_pub = TopicFactory('pwm', Pwm).createPublisher(self)

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
        for k, v in self.motor.idle().items():
            self.pwm.channels[k] = v

    def mission_find_step_one(self):
        self.get_logger().info(f"<=> [{NodeConfig.motor_controller}] Mission_find_step_one_masuk")

        for k, v in self.motor.autonomous(
            yaw_effort=0,
            speed_effort=300,
        ).items():
            self.pwm.channels[k] = v

    def mission_step_one(self):
        self.get_logger().info(f"<=> [{NodeConfig.motor_controller}] Mission_step_one_masuk")

        for k, v in self.motor.autonomous(
            yaw_effort=0,
            speed_effort=self.dsc
        ).items():
            self.pwm.channels[k] = v

    def mission_find_step_two(self):
        self.get_logger().info(f"<=> [{NodeConfig.motor_controller}] Mission_find_step_two_masuk")

        for k, v in self.motor.autonomous(
            yaw_effort=self.state_dst,
            speed_effort=350,
        ).items():
            self.pwm.channels[k] = v

    def mission_step_two(self):
        self.get_logger().info(f"<=> [{NodeConfig.motor_controller}] Mission_step_two_masuk")

        for k, v in self.motor.autonomous(
            yaw_effort=0,
            speed_effort=self.dsc
        ).items():
            self.pwm.channels[k] = v

    def mission_find_step_three(self):
        self.get_logger().info(f"<=> [{NodeConfig.motor_controller}] Mission_find_step_three_masuk")

        for k, v in self.motor.autonomous(
            yaw_effort=self.state_dst,
            speed_effort=0
        ).items():
            self.pwm.channels[k] = v

    def mission_step_three(self):
        self.get_logger().info(f"<=> [{NodeConfig.motor_controller}] Mission_step_three_masuk")

        for k, v in self.motor.autonomous(
            yaw_effort=0,
            speed_effort=self.dsc
        ).items():
            self.pwm.channels[k] = v

    def mission_position_green_box(self):
        for k, v in self.motor.autonomous(
            yaw_effort=self.state_dst,
            speed_effort=0,
        ).items():
            self.pwm.channels[k] = v

    def mission_take_green_box(self):
        self.get_logger().info(f"<=> [{NodeConfig.motor_controller}] Mission_green_box_masuk")

        for k, v in self.motor.autonomous(
            yaw_effort=0,
            speed_effort=0,
        ).items():
            self.pwm.channels[k] = v

    def mission_position_blue_box(self):
        for k, v in self.motor.autonomous(
            yaw_effort=self.state_dst,
            speed_effort=0,
        ).items():
            self.pwm.channels[k] = v

    def mission_take_blue_box(self):
        self.get_logger().info(f"<=> [{NodeConfig.motor_controller}] Mission_blue_box_masuk")

        for k, v in self.motor.autonomous(
            yaw_effort=0,
            speed_effort=0,
        ).items():
            self.pwm.channels[k] = v

    def mission_docking(self):
        for k, v in self.motor.idle().items():
            self.pwm.channels[k] = v

    def _gcs_cam_config_callback(self, msg: Config):
        """Update motor configuration from GCS"""
        # Note: updateAdjustment method needs to be implemented in Motor class
        pass

    def _yaw_control_effort_callback(self, msg: Float64):
        self.yaw_control_effort = msg.data

    def _dsc_control_effort_callback(self, msg: Float64):
        self.dsc_control_effort = msg.data

    def _autocontrol_callback(self, msg: AutoControl):
        self.mission_state.data = msg.data

    def _killswitch_callback(self, msg: KillSwitch):
        self.killswitch_state.data = msg.data

    def _obj_counted_callback(self, msg: ObjectCount):
        self.object_counted = msg

    def _joy_state_callback(self, msg: Controller):
        self.joy_state = msg

    def _dsc_callback(self, msg: Float64):
        self.dsc = msg.data

    def _dsc_flag_callback(self, msg: Float64):
        self.dsc_flag = msg.data

    def mission_callback(self, msg: UInt8):
        """This method will be called whenever a message is received on the mission topic"""
        self.current_mission = msg.data

    def _state_dst_callback(self, msg: Float64):
        self.state_dst = msg.data

    def run(self):
        """Main execution loop"""
        # Display initial PWM status
        self.display_pwm_status()

        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz
        
        self.get_logger().info(f"<> [{NodeConfig.motor_controller}] Successfully initialized node")

    def control_loop(self):
        """Main control loop executed at 50Hz"""
        try:

            self.pwm.channels = [int(val) for val in self.pwm.channels]

            self.pwm_pub.publish(self.pwm)
            
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {traceback.format_exc()}")

    def test_sequence(self):
        """Test sequence to validate motor functionality"""
        self.get_logger().info("Starting motor test sequence...")
        
        try:
            # Move forward
            self.get_logger().info("Moving forward...")
            for k, v in self.motor.autonomous(yaw_effort=0, speed_effort=200).items():
                self.pwm.channels[k] = v
            self.pwm_pub.publish(self.pwm)
            time.sleep(2)

            # Move backward
            self.get_logger().info("Moving backward...")
            for k, v in self.motor.autonomous(yaw_effort=0, speed_effort=-200).items():
                self.pwm.channels[k] = v
            self.pwm_pub.publish(self.pwm)
            time.sleep(2)

            # Turn right
            self.get_logger().info("Turning right...")
            for k, v in self.motor.autonomous(yaw_effort=100, speed_effort=0).items():
                self.pwm.channels[k] = v
            self.pwm_pub.publish(self.pwm)
            time.sleep(2)

            # Turn left
            self.get_logger().info("Turning left...")
            for k, v in self.motor.autonomous(yaw_effort=-100, speed_effort=0).items():
                self.pwm.channels[k] = v
            self.pwm_pub.publish(self.pwm)
            time.sleep(2)

            # Stop
            self.get_logger().info("Stopping...")
            for k, v in self.motor.idle().items():
                self.pwm.channels[k] = v
            self.pwm_pub.publish(self.pwm)
            time.sleep(2)

            self.get_logger().info("Motor test sequence completed successfully.")

        except Exception as e:
            self.get_logger().error(f"Error during motor test sequence: {traceback.format_exc()}")

            
def main(args=None):
    try:
        rclpy.init(args=args)

        motor_control = MotorController()
        motor_control.run()
        motor_control.test_sequence()
        
        # Spin the node
        rclpy.spin(motor_control)
        
    except Exception as e:
        print(f"Error in main: {traceback.format_exc()}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()