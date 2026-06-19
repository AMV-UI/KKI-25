
from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64, UInt8, String
from core.mission.find_mode import FindMode
from core.mission.frame_counter import FrameCounter
from core_msgs.msg import Pixhawk
from core.utils.config import Topic, MissionParams

import time


class Finding_Execution(BaseExecution):
    """
    Combined behavior: FIND_STEP_TWO -> STEP_TWO
    - Phase "finding": search with find_mode range(1), wait until detected for N frames
    - Phase "approach": navigate to target, when lost for N frames -> SUCCESS (ready for docking)
    """
    def __init__(self, name, node=None, mission = None):
        super().__init__(name, node=node)
        self.node = node
        self.find_mode = None
        self.frame_counter = None
        self.detected = False
        self.arena = "B"
        self.effort = MissionParams.finding_yaw_effort
        self.dsc = self.effort * (-1 if self.arena == "A" else 1)  #Reverse effort untuk mission 2
        self.mission = mission
        self.time_threshold = MissionParams.finding_time_threshold

        self.px_heading = 0.0
        self.phase = "finding"        

        self.speed_effort = MissionParams.finding_speed_effort

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.find_mode = FindMode(self.node, self.arena)
        self.frame_counter = FrameCounter(self.time_threshold)

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
        self.mission_pub = Topic.mission.createPublisher(self.node)

        self.arena_sub = Topic.arena.createSubscriber(self.node, self._arena_cb)
        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.dsc_sub = Topic.dsc.createSubscriber(self.node, self._dsc_cb)
        self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)

    def _detected_cb(self, msg: Bool):
        self.detected = bool(msg.data)

    def _dsc_cb(self, msg: Float64):
        self.dsc = float(msg.data)

    def _heading_cb(self, msg: Float64):
        self.px_heading = float(msg.data)

    def execute(self) -> Status:
        self.node.get_logger().info(f"[{self.name}] We are executing Finding... track: {self.arena}", throttle_duration_sec=1.0)        
        if self.detected:
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                self.node.get_logger().info(
                    f"[{self.name}] Finding Complete"
                )
                self.mission_pub.publish(UInt8(data=self.mission))
                return Status.SUCCESS
        else:
            self.frame_counter.reset()

        # Reverse yaw effort to counter Turn_Next_Buoy overshoot
        # Arena A: Turn Right (Positive), Arena B: Turn Left (Negative)
        yaw_val = float(self.effort * (-1 if self.arena == "B" else 1))
        self.yaw_effort_pub.publish(Float64(data=yaw_val))
        
        # Zero speed effort during finding phase, just yaw
        self.speed_effort_pub.publish(Float64(data=0.0))
        return Status.RUNNING

class Finding_Fallback(BaseFallback):
    """
    """
    def __init__(self, name: str = "Mission2_Fallback", node=None):
        super().__init__(name, node=node)
    
    def fallback(self) -> Status:
        return Status.FAILURE
