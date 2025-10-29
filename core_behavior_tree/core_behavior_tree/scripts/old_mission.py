#!/usr/bin/env  python3

import rospy
import time

from std_msgs.msg import UInt8, Bool
from kki24.msg import Option, AutoControl, StateObject, Pixhawk
from utils.config import Node, PxMode
from utils.converter import autocontrolToString
from utils.frame_counter import FrameCounter
from utils.config import Param
# from camera_front import FrontCamera

from utils.config import Topic, Direction, Param
from std_msgs.msg import Float64


class FindMode:
    def __init__(self):
        self.initial_heading = -1
        self.current_heading = -1

        self.treshold = 90

        self.range_low = -1
        self.range_high = -1

        self.find_status = "Left" if rospy.get_param(Param.TRACK) == "A" else "Right"

        self.GO_LEFT = -200  # STATE
        self.GO_RIGHT = 200
        self.dir_map = {
            "Right": self.GO_RIGHT,
            "Left": self.GO_LEFT,
        }

    def set_initial_heading(self, heading):
        self.initial_heading = heading

    def set_range(self, direction):
        if rospy.get_param(Param.TRACK) == "A":
            self.range_low = Direction.A[direction] - self.treshold
            self.range_high = Direction.A[direction] + self.treshold
        else:
            self.range_low = Direction.B[direction] - self.treshold
            self.range_high = Direction.B[direction] + self.treshold

    def get_heading(self, raw_heading):
        heading = raw_heading - self.initial_heading
        if heading < 0:
            heading = 360 + heading
        return heading

    def get_state(self, raw_heading):
        self.current_heading = self.get_heading(raw_heading)
        # kasus diluar bound

        # lb => lower bound
        # ub => 
        lb = abs(self.current_heading - self.range_low)
        if self.range_low == 0:
            lb = min(lb, abs(self.current_heading - 360))

        ub = abs(self.current_heading - self.range_high)
        if self.range_high == 0:
            ub = min(ub, abs(self.current_heading - 360))

        if (
            not (self.range_low if self.range_low == 360 else 0)
            < self.current_heading
            < (self.range_high if self.range_high != 0 else 360)
        ):
            if lb < ub:
                self.find_status = "Right"
            else:
                self.find_status = "Left"

        return self.dir_map[self.find_status]


class Mission:
    def __init__(self):
        # m
        self.mission_received = AutoControl()
        self.mission_strat = Option()
        self.mission_strat.data = Option.NONE

        self.pixhawk = Pixhawk()
        # Find target
        self.find_mode = FindMode()
        self.frame_counter = FrameCounter(
            2
        )  # 25 frame for some reason to consistently detect the buoy

        self.frame_counter_manuver = FrameCounter(3)

        self.px_heading = -1
        self.pxmode = PxMode.HOLD

        # self.current_state = StateObject()
        self.detected = False
        self.isChange = False
        self.dsc = float(160)
        self.current_mission = AutoControl.MISSION_STEP_THREE

        self.manuver_detected = False

        # Data misi
        self.mapping_job = {
            AutoControl.MISSION_FIND_STEP_ONE: self.mission_find_step_one,
            AutoControl.MISSION_FIND_STEP_TWO: self.mission_find_step_two,
            AutoControl.MISSION_FIND_STEP_THREE: self.mission_find_step_three,
            AutoControl.MISSION_STEP_ONE: self.mission_step_one,
            AutoControl.MISSION_STEP_TWO: self.mission_step_two,
            AutoControl.MISSION_STEP_THREE: self.mission_step_three,
            AutoControl.MISSION_MANUVER: self.mission_manuver,
            AutoControl.MISSION_POSITION_GREEN_BOX: self.mission_position_green_box,
            AutoControl.MISSION_TAKE_GREEN_BOX: self.mission_take_green_box,
            AutoControl.MISSION_POSITION_BLUE_BOX: self.mission_position_blue_box,
            AutoControl.MISSION_TAKE_BLUE_BOX: self.mission_take_blue_box,
            AutoControl.MISSION_DOCKING: self.mission_docking,
            # AutoControl.MISSION_DONE : self.MISSION_DONE
        }

    def mission_find_step_one(self):
        rospy.loginfo_throttle(
            5, f"<=> [{Node.mission}] Entering improc for mission mencari buoy tahap 1"
        )
        rospy.loginfo_throttle(1, f"<=> [{Node.mission}] blalvavla {self.detected}")

        dsc_state = 0
        dsc_state = self.dsc
        self.state_dst_pub.publish(dsc_state)

        if self.detected:
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.mission_pub.publish(AutoControl.MISSION_STEP_ONE)
                rospy.loginfo_throttle(
                    1, f"<=> [{Node.mission}] {self.detected} Detecting..."
                )
                self.frame_counter.reset()
                return True
        else:
            self.frame_counter.reset()
        self.find_mode.set_initial_heading(self.px_heading)

        return False

    def mission_step_one(self):
        rospy.loginfo_once(f"<=> [{Node.mission}] Entering improc for mission step 1")
        rospy.loginfo_throttle(1, f"<=> [{Node.mission}] {self.detected}")

        dsc_state = self.dsc

        if not self.detected:
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                self.mission_pub.publish(AutoControl.MISSION_FIND_STEP_TWO)
                rospy.loginfo_throttle(2, f"<=> [{Node.mission}] {self.detected}")
                return True

        else:
            self.frame_counter.reset()
        self.state_dst_pub.publish(dsc_state)
        return False

    def mission_find_step_two(self):
        rospy.loginfo_once(
            f"<=> [{Node.mission}] Entering improc for mission mencari buoy tahap 2"
        )
        self.find_mode.current_heading = self.find_mode.get_heading(self.px_heading)

        dsc_state = self.dsc
        self.find_mode.set_range(1)

        # TODO: Code is not clean enough, clean it later :)
        if self.detected:  # Tower Found
            self.frame_counter.is_started()

            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                self.mission_pub.publish(AutoControl.MISSION_STEP_TWO)
                rospy.loginfo_throttle(2, f"<=> [{Node.mission}] {self.detected}")
                return True
        else:
            self.frame_counter.reset()

            if not self.detected:
                dsc_state = self.find_mode.get_state(self.px_heading)

                rospy.loginfo_throttle(5, f"[{Node.mission}] {dsc_state}")

        rospy.loginfo_throttle(
            1,
            f"[{Node.mission}] Current Heading : {self.px_heading} {self.find_mode.range_low} {self.find_mode.range_high}",
        )
        rospy.loginfo_throttle(
            1, f"[{Node.mission}] Initial Heading : {self.find_mode.initial_heading}"
        )
        rospy.loginfo_throttle(
            1, f"[{Node.mission}] Base Heading : {self.find_mode.current_heading}"
        )

        self.state_dst_pub.publish(dsc_state)
        return False

    def mission_step_two(self):
        rospy.loginfo_once(
            f"<=> [{Node.mission}] Entering improc for mission buoy tahap 2"
        )
        dsc_state = self.dsc

        if not self.detected:
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                self.mission_pub.publish(AutoControl.MISSION_FIND_STEP_THREE)
                rospy.loginfo_throttle(2, f"<=> [{Node.mission}] {self.detected}")
                return True
        else:
            self.frame_counter.reset()

        dsc_state = self.dsc

        self.state_dst_pub.publish(dsc_state)
        return False

    def mission_find_step_three(self):
        rospy.loginfo_once(
            f"<=> [{Node.mission}] Entering improc for mission mencari buoy tahap 3"
        )
        self.find_mode.set_range(2)
        self.find_mode.current_heading = self.find_mode.get_heading(self.px_heading)

        # dsc_state = self.dsc_state
        dsc_state = self.dsc

        # # TODO: Code is not clean enough, clean it later :)
        if self.detected:
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                self.mission_pub.publish(AutoControl.MISSION_STEP_THREE)
                return True
        else:
            self.frame_counter.reset()

            if not self.detected:  # removed: and dsc_state == 160
                dsc_state = self.find_mode.get_state(self.px_heading)

        rospy.loginfo_throttle(1, f"[{Node.mission}] {self.find_mode.initial_heading}")
        self.state_dst_pub.publish(dsc_state)

        return False

    def mission_step_three(self):
        rospy.logerr_throttle(
            5, f"<=> [{Node.mission}] Entering improc for mission buoy tahap 3"
        )

        if rospy.get_param(Param.TRACK) == "A" and not self.isChange:
            rospy.set_param(Param.TRACK, "B")
            self.isChange = True
        elif rospy.get_param(Param.TRACK) == "B" and not self.isChange:
            rospy.set_param(Param.TRACK, "A")
            self.isChange = True

        dsc_state = self.dsc

        if not self.detected:
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                # self.mission_pub.publish(AutoControl.MISSION_POSITION_GREEN_BOX)
                self.mission_pub.publish(AutoControl.MISSION_MANUVER)
                return True
        else:
            self.frame_counter.reset()

        self.state_dst_pub.publish(dsc_state)
        return False

    # Untested
    def mission_manuver(self):
        rospy.logerr_throttle(
            5, f"<=> [{Node.mission}] Entering improc for mission manuver"
        )
        # Kasus detect
        if self.detected:
            self.manuver_detected = True
            # Flip di mission 3
            if rospy.get_param(Param.TRACK) == "A":
                # Ke kiri
                self.state_dst_pub.publish(-200)
            else:
                # Ke kanan
                self.state_dst_pub.publish(200)
        elif self.manuver_detected == True and self.detected == False:
            self.frame_counter_manuver.is_started()
            if rospy.get_param(Param.TRACK) == "A":
                # Ke kiri
                self.state_dst_pub.publish(-200)
            else:
                # Ke kanan
                self.state_dst_pub.publish(200)

            if self.frame_counter_manuver.is_enough():
                self.frame_counter.reset()
                self.mission_pub.publish(AutoControl.MISSION_POSITION_GREEN_BOX)
                return True

        # Kasus ngga detek sama sekali dari awal
        elif self.manuver_detected == False:
            # Lupa logic tapi harusnya lurus doang selama 3 FC
            self.frame_counter_manuver.is_started()
            if self.frame_counter_manuver.is_enough():
                self.frame_counter.reset()
                self.mission_pub.publish(AutoControl.MISSION_POSITION_GREEN_BOX)
                return True

        return False

    def mission_position_green_box(self):  # TODO : KERJAKAN
        rospy.logerr_throttle(
            5, f"<=> [{Node.mission}] Entering improc for mission green box"
        )

        # Bakal find river kearah 90 / 270
        self.find_mode.set_range(3)
        self.find_mode.current_heading = self.find_mode.get_heading(self.px_heading)
        dsc_state = self.dsc

        if self.detected:
            # udah keliatan box
            self.mission_pub.publish(AutoControl.MISSION_TAKE_GREEN_BOX)
            return True

        dsc_state = self.find_mode.get_state(self.px_heading)
        self.state_dst_pub.publish(dsc_state)
        return False

    def mission_take_green_box(self):  # TODO
        rospy.logerr_throttle(
            5, f"<=> [{Node.mission}] Entering improc for mission take green box"
        )

        self.frame_counter.is_started()

        if self.frame_counter.is_enough():
            self.image_green_box.publish(self.image)
            self.frame_counter.reset()
            self.mission_pub.publish(AutoControl.MISSION_POSITION_BLUE_BOX)
            return True

        return False

    def mission_position_blue_box(self):
        rospy.logerr_throttle(
            5, f"<=> [{Node.mission}] Entering improc for mission blue box"
        )

        self.find_mode.set_range(3)
        self.find_mode.current_heading = self.find_mode.get_heading(self.px_heading)
        dsc_state = self.dsc

        if self.detected:
            # udah keliatan box
            self.mission_pub.publish(AutoControl.MISSION_TAKE_BLUE_BOX)
            return True

        dsc_state = self.find_mode.get_state(self.px_heading)
        self.state_dst_pub(dsc_state)
        return False

    def mission_take_blue_box(self):
        rospy.logerr_throttle(
            5, f"<=> [{Node.mission}] Entering improc for mission take blue box"
        )

        self.frame_counter.is_started()

        if self.frame_counter.is_enough():
            self.image_blue_box.publish(self.camera_bottom)
            self.frame_counter.reset()
            self.mission_pub.publish(AutoControl.MISSION_DOCKING)
            return True

        return False

    def mission_find_docking(self):
        rospy.logerr_once(f"<=> [{Node.mission}] Entering improc for find docking")
        dsc_state = self.dsc
        if not self.detected:
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                # self.mission_pub.publish(AutoControl.MISSION_POSITION_GREEN_BOX)
                self.mission_pub.publish(AutoControl.MISSION_MANUVER)
                return True
        else:
            self.frame_counter.reset()

        self.state_dst_pub.publish(dsc_state)
        return False

    def mission_docking(self):  # TODO: Mission Docking
        rospy.logerr_once(f"<=> [{Node.mission}] Entering improc for mission docking")
        # angle = self.calculate_angle()

        self.find_mode.set_range(2)
        self.find_mode.current_heading = self.find_mode.get_heading(self.px_heading)

        # dsc_state = self.dsc_state
        dsc_state = 0

        # # TODO: Code is not clean enough, clean it later :)
        if not self.detected:
            dsc_state = self.find_mode.get_state(self.px_heading)

        self.state_dst_pub.publish(dsc_state)

        return True

    def heading_callback(self, msg):
        self.px_heading = msg.data

    def mission_callback(self, msg: UInt8):
        # This method will be called whenever a message is received on the mission topic
        self.mission_sub = msg.data
        self.current_mission = self.mission_sub

    def state_callback(self, msg: StateObject):
        self.current_state = msg

    def detected_callback(self, msg: Bool):
        self.detected = msg.data

    def image_callback(self, msg):
        self.image = msg.data

    def dsc_callback(self, msg):
        self.dsc = msg.data

    def pixhawk_callback(self, msg: Pixhawk):
        self.pixhawk = msg

    def pxmode_callback(self, msg):
        self.pxmode = msg.data

    def camera_bottom_callback(self, msg):
        self.camera_bottom = msg.data

    def main(self):
        # PUBLISHERS
        self.mission_pub = Topic.mission.createPublisher()
        self.state_dst_pub = Topic.state_dst.createPublisher()
        self.image_green_box = Topic.image_green_box.createPublisher()
        self.image_blue_box = Topic.image_blue_box.createPublisher()
        self.y_state_pub = Topic.y_state.createPublisher()
        self.base_heading_pub = Topic.base_heading.createPublisher()

        # self.object_counted_pub = Topic.object_counted.createPublisher()

        # SUBSCRIBERS
        self.pixhawk_sub = Topic.pixhawk.createSubscriber(self.pixhawk_callback)
        self.pxmode_sub = Topic.pxmode.createSubscriber(self.pxmode_callback)
        self.heading_sub = Topic.heading_deg.createSubscriber(self.heading_callback)
        self.mission_sub = Topic.mission.createSubscriber(self.mission_callback)
        # self.state_sub = Topic.state_object.createSubscriber(self.state_callback)
        self.detected_sub = Topic.detected.createSubscriber(self.detected_callback)
        self.image_sub = Topic.camera_processed.createSubscriber(self.image_callback)
        self.dsc_sub = Topic.dsc.createSubscriber(self.dsc_callback)
        self.camera_bottom_sub = Topic.camera_bottom.createSubscriber(
            self.camera_bottom_callback
        )
        # self.pixhawk_sub = Topic.pixhawk.createSubscriber(self.pixhawk_callback)

        while (
            not rospy.is_shutdown()
            and self.current_mission < AutoControl.MISSION_DOCKING + 1
        ):
            # self.do_impros()
            # if self.pxmode != PxMode.HOLD:  # Testing
            if self.pxmode == PxMode.HOLD:  # Testing
                mission = self.mapping_job.get(self.current_mission)
                if mission():
                    rospy.loginfo(
                        f"Mission {self.current_mission - 1} completed. Moving to the next mission."
                    )
            else:
                rospy.loginfo_throttle(
                    5,
                    f"<=> [{Node.microcontroller}] Resetting Mission in {self.pxmode}.",
                )
                self.find_mode.set_initial_heading(self.px_heading)
                self.current_mission = AutoControl.MISSION_FIND_STEP_ONE
                rospy.loginfo_throttle(
                    5, f"[{Node.mission}] {self.find_mode.initial_heading}"
                )

            # Publish
            self.base_heading_pub.publish(self.find_mode.current_heading)

        rospy.loginfo_once(f"<> [{Node.mission}] Successfully initialized Node!")


if __name__ == "__main__":
    # try:
    # Initialize node
    rospy.init_node(Node.mission)

    mission_real = Mission()
    mission_real.main()