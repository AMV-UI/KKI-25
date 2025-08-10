#!/usr/bin/env python3

import traceback

import rclpy
from rclpy.node import Node

from core_msgs.msg import (
    Config,
    ObjectCount,
    Controller,
    Pwm,
    Option,
    KillSwitch,
)
from std_msgs.msg import Float64, UInt8
from core.utils.config import Node as NodeName, Topic, Param, SPEED
from core.utils.motor import Motor

class MotorController(Node):

    # Docs basiclly

    # """
    # GETS A GOAL FROM:
    #     - Behavior Tree via /mission topic
    # SUBSCRIBES:
    #     - /mission (std_msgs/UInt8): Mission ID from the Behavior Tree
    #     - /kill_switch (core_msgs/KillSwitch)
    #     - /controller (core_msgs/Controller)
    #     - ...and other topics for control inputs.
    # PUBLISHES:
    #     - PWM to core_control.microcontroller
    # """

    def __init__(self):
        super().__init__(NodeName.motor_controller)

        # Declare parameters for motor speed and adjustment
        self.declare_parameter(Param.MOTOR_SPEED, SPEED.Maximum)
        self.declare_parameter(Param.X_SPEED, SPEED.Maximum)
        
        # Core motor interface 
        self.motor = Motor()

        # State variables
        self.yaw_control_effort = 0.0
        self.dsc_control_effort = 0.0
        self.object_counted = ObjectCount()
        self.joy_state = Controller()
        self.is_killed = False
        self.pwm = Pwm()
        self.killswitch_state = KillSwitch()
        self.killswitch_state.data = 0 # Default = 0
        self.dsc = 0.0
        self.dsc_flag = 0.0
        self.state_dst = 0.0
        self.current_mission = 0 # Default mission ID

        # Mapping mission IDs from Behavior Tree to Python functions
        self.mission_map = {
            0: self.manual,
            1: self.mission_find_step_one,
            2: self.mission_find_step_two,
            3: self.mission_find_step_three,
            4: self.mission_step_one,
            5: self.mission_step_two,
            6: self.mission_step_three,
            7: self.mission_position_green_box,
            8: self.mission_take_green_box,
            9: self.mission_position_blue_box,
            10: self.mission_take_blue_box,
            11: self.mission_docking,
        }

        # Subscribers
        Topic.control_effort_dsc.createSubscriber(self, self._dsc_control_effort_callback)
        Topic.object_counted.createSubscriber(self, self._obj_counted_callback)
        Topic.dsc.createSubscriber(self, self._dsc_callback)
        Topic.state_dst.createSubscriber(self, self._state_dst_callback)
        Topic.kill_switch.createSubscriber(self, self._killswitch_callback)
        Topic.controller.createSubscriber(self, self._joy_state_callback)
        Topic.gcs_config.createSubscriber(self, self._gcs_cam_config_callback)
        
        # Primary subscriber for mission ID from Behavior Tree
        Topic.mission.createSubscriber(self, self.mission_callback)

        #debug

        # Publisher
        self.pwm_pub = Topic.pwm.createPublisher(self)

        # Timer for the control loop at 50Hz
        self.timer = self.create_timer(1.0 / 50.0, self._timer_callback)

        self.get_logger().info(f"MotorController node '{NodeName.motor_controller}' initialized with BT")

    # --- Mission Functions ---
    # Fungsi-fungsi ini sekarang dipanggil berdasarkan ID yang diterima dari BT

    def manual(self):
        # Mission 0, Manual Mode
        for k, v in self.motor.idle().items():
            self.pwm.channels[k] = v

    def mission_find_step_one(self):
        # Mengambil parameter dari node lalu diberikan ke kelas Motor
        motor_speed = self.get_parameter(Param.MOTOR_SPEED).value
       
        for k, v in self.motor.autonomous(
            control_effort_x=0,
            control_effort_y=300,
            motor_speed=motor_speed
        ).items():
            self.pwm.channels[k] = v
            
    def mission_step_one(self):
        motor_speed = self.get_parameter(Param.MOTOR_SPEED).value
        x_speed = self.get_parameter(Param.X_SPEED).value
        for k, v in self.motor.autonomous(
            control_effort_x=self.dsc,
            motor_speed=motor_speed,
            x_speed=x_speed
        ).items():
            self.pwm.channels[k] = v

    def mission_find_step_two(self):
        motor_speed = self.get_parameter(Param.MOTOR_SPEED).value
        for k, v in self.motor.autonomous(
            control_effort_x=self.state_dst,
            control_effort_y=350,
            motor_speed=motor_speed
        ).items():
            self.pwm.channels[k] = v

    def mission_step_two(self):
        motor_speed = self.get_parameter(Param.MOTOR_SPEED).value
        x_speed = self.get_parameter(Param.X_SPEED).value
        for k, v in self.motor.autonomous(
            control_effort_x=self.dsc,
            motor_speed=motor_speed,
            x_speed=x_speed
        ).items():
            self.pwm.channels[k] = v

    def mission_find_step_three(self):
        motor_speed = self.get_parameter(Param.MOTOR_SPEED).value
        for k, v in self.motor.autonomous(
            control_effort_x=self.state_dst,
            motor_speed=motor_speed
        ).items():
            self.pwm.channels[k] = v

    def mission_step_three(self):
        motor_speed = self.get_parameter(Param.MOTOR_SPEED).value
        x_speed = self.get_parameter(Param.X_SPEED).value
        for k, v in self.motor.autonomous(
            control_effort_x=self.dsc,
            motor_speed=motor_speed,
            x_speed=x_speed
        ).items():
            self.pwm.channels[k] = v

    def mission_position_green_box(self):
        motor_speed = self.get_parameter(Param.MOTOR_SPEED).value
        for k, v in self.motor.autonomous(
            control_effort_x=self.state_dst,
            motor_speed=motor_speed
        ).items():
            self.pwm.channels[k] = v

    def mission_take_green_box(self):
        motor_speed = self.get_parameter(Param.MOTOR_SPEED).value
        for k, v in self.motor.autonomous(
            control_effort_x=0,
            control_effort_y=0,
            motor_speed=motor_speed
        ).items():
            self.pwm.channels[k] = v

    def mission_position_blue_box(self):
        motor_speed = self.get_parameter(Param.MOTOR_SPEED).value
        for k, v in self.motor.autonomous(
            control_effort_x=self.state_dst,
            motor_speed=motor_speed
        ).items():
            self.pwm.channels[k] = v

    def mission_take_blue_box(self):
        motor_speed = self.get_parameter(Param.MOTOR_SPEED).value
        for k, v in self.motor.autonomous(
            control_effort_x=0,
            control_effort_y=0,
            motor_speed=motor_speed
        ).items():
            self.pwm.channels[k] = v

    def mission_docking(self):
        pass

    def execute_mission(self, mission_id):
        # Executes the mission function based dari ID yang di received
        try:
            mission_func = self.mission_map.get(mission_id)
            if mission_func:
                mission_func()
            else:
                self.get_logger().warn(f"Mission ID '{mission_id}' not found. Defaulting to manual/idle.")
                self.manual()
        except Exception:
            self.get_logger().error(f"Error executing mission: {traceback.format_exc()}")
            self.manual() # Fallback to manual/idle jika error

    # --- Callback Methodsnya ---

    def _gcs_cam_config_callback(self, msg: Config):
        self.motor.updateAdjustment(msg.back_adjust, msg.bow_adjust, msg.azimuth_adjust)

    def _yaw_control_effort_callback(self, msg: Float64):
        self.yaw_control_effort = msg.data

    def _dsc_control_effort_callback(self, msg: Float64):
        self.dsc_control_effort = msg.data

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
        #Callback for mission ID from Behavior Tree
        self.get_logger().info(f"Received new mission ID: {msg.data}")
        self.current_mission = msg.data
        
    def _joy_state_callback(self, msg: Controller):
        self.joy_state = msg
        self.get_logger().info(
            f"[JOY] Controller received: data={msg.data}, linear_y={msg.linear_y}, angular_z={msg.angular_z}"
        )


    def _state_dst_callback(self, msg: Float64):
        self.state_dst = msg.data

    # --- Main Loop (Timer) ---

    def _timer_callback(self):
        self.execute_mission(self.current_mission)
        self.pwm.channels = [int(val) for val in self.pwm.channels]
        self.pwm_pub.publish(self.pwm)


def main(args=None):
    rclpy.init(args=args)
    motor_ctrl = MotorController()
    try:
        rclpy.spin(motor_ctrl)
    except KeyboardInterrupt:
        pass
    finally:
        motor_ctrl.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()