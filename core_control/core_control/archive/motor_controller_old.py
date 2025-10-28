#!/usr/bin/env python3

from statistics import variance
import rospy
import traceback
import actionlib
import time

from std_msgs.msg import Bool, Float64, UInt16, UInt32, UInt8
from sensor_msgs.msg import Joy
from kki24.msg import (
    KillSwitch,
    AutoControl,
    Config,
    ObjectCount,
    Controller,
    Pwm,
    Option,
)

from utils.config import Node, Topic, Param
from utils.motor import Motor
from utils.converter import autocontrolToString

from kki24.msg import (
    SendMissionAction,
    SendMissionActionFeedback,
    SendMissionActionGoal,
    SendMissionActionResult,
    SendMissionFeedback,
    SendMissionGoal,
    SendMissionResult,
)


""" Mappings """
# _kill_switch_map = {
#     KillSwitch.HARDWARE: Motor.KILL_BUTTON,
#     KillSwitch.ON: Motor.KILL_ON,
#     KillSwitch.OFF: Motor.KILL_OFF,
# }


class MotorController:
    """
    GET A GOAL FROM:
        - misssion_controller
    SUBSCRIBES:
        - TODO:
    PUBLISHES:
        - PWM to microcontroller
    """

    def __init__(self):
        # actionlib
        self.motor = Motor()

        # Initialize state variables

        self.yaw_control_effort = float()
        self.dsc_control_effort = float()
        # self.killswitch_state = KillSwitch()

        self.object_counted = ObjectCount()
        self.joy_state = Joy()
        self.is_killed = False

        self.pwm = Pwm()

        self.killswitch_state = KillSwitch()
        self.killswitch_state.data = KillSwitch.default

        # self.bow = rospy.get_param(Param.MOTOR_SPEED)
        # self.x = rospy.get_param(Param.X_SPEED)

        self.dsc = float()
        self.dsc_flag = float()
        self.state_dst = float()

        self.mission_state = AutoControl()

        self.current_strat = Option()
        self.strat_map = {
            AutoControl.MANUAL: self.manual,
            AutoControl.MISSION_FIND_STEP_ONE: self.mission_find_step_one,
            AutoControl.MISSION_FIND_STEP_TWO: self.mission_find_step_two,
            AutoControl.MISSION_FIND_STEP_THREE: self.mission_find_step_three,
            AutoControl.MISSION_STEP_ONE: self.mission_step_one,
            AutoControl.MISSION_STEP_TWO: self.mission_step_two,
            AutoControl.MISSION_STEP_THREE: self.mission_step_three,
            AutoControl.MISSION_POSITION_GREEN_BOX: self.mission_position_green_box,
            AutoControl.MISSION_TAKE_GREEN_BOX: self.mission_take_green_box,
            AutoControl.MISSION_POSITION_BLUE_BOX: self.mission_position_blue_box,
            AutoControl.MISSION_TAKE_BLUE_BOX: self.mission_take_blue_box,
            AutoControl.MISSION_DOCKING: self.mission_docking,
            # AutoControl.MISSION_DONE : self.MISSION_DONE
        }

        self.current_mission = AutoControl.MISSION_FIND_STEP_ONE

    def manual(self):
        for k, v in self.motor.idle().items():
            self.pwm.channels[k] = v

    def mission_find_step_one(self):
        # rospy.loginfo(f"<=> [{Node.motor_controller}] Mission_find_step_one_masuk")

        for k, v in self.motor.autonomous(
            # TODO: yaw control effort reconfig value
            # control_effort_yaw_bow=self.yaw_control_effort,
            control_effort_x=0,
            control_effort_y=300,
        ).items():
            self.pwm.channels[k] = v

    def mission_step_one(self):
        # rospy.loginfo(f"<=> [{Node.motor_controller}] Mission_step_one_masuk")

        for k, v in self.motor.autonomous(
            # TODO: yaw control effort reconfig value
            # control_effort_x=dcs
            control_effort_x=self.dsc
        ).items():
            self.pwm.channels[k] = v

    def mission_find_step_two(self):
        # rorspy.loginfo(f"<=> [{Node.motor_controller}] Mission_find_step_two_masuk")

        for k, v in self.motor.finding_turn(
            # control_effort_yaw_bow=self.yaw_control_effort,
            control_effort_x=self.state_dst,
            control_effort_y=350,
        ).items():
            self.pwm.channels[k] = v

    def mission_step_two(self):
        # rospy.loginfo(f"<=> [{Node.motor_controller}] Mission_step_two_masuk")

        for k, v in self.motor.autonomous(
            # TODO: yaw control effort reconfig value
            # control_effort_yaw_bow=self.yaw_control_effort,
            control_effort_x=self.dsc
        ).items():
            self.pwm.channels[k] = v

    def mission_find_step_three(self):
        # rospy.loginfo(f"<=> [{Node.motor_controller}] Mission_find_step_three_masuk")

        for k, v in self.motor.finding_turn(
            # TODO: yaw control effort reconfig value
            # control_effort_yaw_bow=self.yaw_control_effort,
            control_effort_x=self.state_dst
        ).items():
            self.pwm.channels[k] = v

    def mission_step_three(self):
        # rospy.loginfo(f"<=> [{Node.motor_controller}] Mission_step_three_masuk")

        for k, v in self.motor.autonomous(
            # TODO: yaw control effort reconfig value
            # control_effort_yaw_bow=self.yaw_control_effort,
            control_effort_x=self.dsc
        ).items():
            self.pwm.channels[k] = v

    def mission_position_green_box(self):
        for k, v in self.motor.finding_turn(
            # TODO: yaw control effort reconfig value
            # control_effort_yaw_bow=self.yaw_control_effort,
            control_effort_x=self.state_dst,
        ).items():
            self.pwm.channels[k] = v

    def mission_take_green_box(self):
        # rospy.loginfo(f"<=> [{Node.motor_controller}] Mission_green_box_masuk")

        for k, v in self.motor.finding_turn(
            # TODO: yaw control effort reconfig value
            # control_effort_yaw_bow=self.yaw_control_effort,
            control_effort_x=0,
            control_effort_y=0,
        ).items():
            self.pwm.channels[k] = v

    def mission_position_blue_box(self):
        for k, v in self.motor.finding_turn(
            # TODO: yaw control effort reconfig value
            # control_effort_yaw_bow=self.yaw_control_effort,
            control_effort_x=self.state_dst,
        ).items():
            self.pwm.channels[k] = v

    def mission_take_blue_box(self):
        # rospy.loginfo(f"<=> [{Node.motor_controller}] Mission_blue_box_masuk")

        for k, v in self.motor.finding_turn(
            # TODO: yaw control effort reconfig value
            # control_effort_yaw_bow=self.yaw_control_effort,
            control_effort_x=0,
            control_effort_y=0,
        ).items():
            self.pwm.channels[k] = v

    def mission_docking(self):
        pass

    # def do_motor(self):
    #     while not rospy.is_shutdown():
    #         self.pwm.channels = [int(val) for val in self.pwm.channels]
    #         # Publish pwm
    #         self.pwm_pub.publish(self.pwm)

    def execute_strategy(self, auto_control_state):
        """
        Lookup and executes the strategy based on the current mission state (auto_control_state).
        """
        try:
            # lookup strat
            strategy_function = self.strat_map.get(auto_control_state, None)
            # print(strategy_function)

            if strategy_function:
                strategy_function()
            # else:
            #     rospy.logwarn(f"Unknown AutoControl state: {auto_control_state}")

        except KeyError as e:
            rospy.logerr(f"Strategy not found for AutoControl state: {e}")
        except Exception as e:
            rospy.logerr(f"Error while executing strategy: {traceback.format_exc()}")

    def _gcs_cam_config_callback(self, msg: Config):
        """Update motor configuration from GCS"""
        self.motor.updateAdjustment(msg.back_adjust, msg.bow_adjust, msg.azimuth_adjust)

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
        # This method will be called whenever a message is received on the mission topic
        # self.mission_sub = msg.data
        self.current_mission = msg.data

    def _state_dst_callback(self, msg: Float64):
        self.state_dst = msg.data

    def main(self) -> None:
        # SUBCRIBERS
        dsc_control_effort_sub = Topic.control_effort_dsc.createSubscriber(
            self._dsc_control_effort_callback
        )
        mission_sub = Topic.auto_control.createSubscriber(self._autocontrol_callback)
        object_counted_subscriber = Topic.object_counted.createSubscriber(
            self._obj_counted_callback
        )
        dsc_subscriber = Topic.dsc.createSubscriber(self._dsc_callback)
        state_dst_sub = Topic.state_dst.createSubscriber(self._state_dst_callback)
        self.pwm_pub = Topic.pwm.createPublisher()
        self.mission_sub = Topic.mission.createSubscriber(self.mission_callback)

        while not rospy.is_shutdown():
            self.execute_strategy(self.current_mission)

            self.pwm.channels = [int(val) for val in self.pwm.channels]
            # Publish pwm
            self.pwm_pub.publish(self.pwm)

            rospy.loginfo_once(
                f"<> [{Node.motor_controller}] Successfully initialized node"
            )

            rospy.Rate(50).sleep()


if __name__ == "__main__":
    try:
        # Initialize node
        rospy.init_node(Node.motor_controller)

        motor_control = MotorController()
        motor_control.main()

    except Exception as e:
        rospy.logerr(traceback.format_exc())