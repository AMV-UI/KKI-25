
from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64, UInt8, String
from core.mission.find_mode import FindMode
from core.mission.frame_counter import FrameCounter
from core_msgs.msg import Pixhawk
from core.utils.config import Topic

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
        self.effort = 150
        self.dsc = self.effort * (-1 if self.arena == "A" else 1)  #Reverse effort untuk mission 2
        self.mission = mission
        self.time_threshold = 0.2

        self.px_heading = 0.0
        self.phase = "finding"        

        self.speed_effort = 70.0

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
       
        self.effort_sub = Topic.tuning_effort_tn.createSubscriber(
            self.node,
            self._effort_cb
        )     

        self.speed_sub = Topic.tuning_effort_st.createSubscriber(
            self.node,
            self._speed_cb
        )
        
    def _speed_cb(self, msg: Float64):
        self.speed_effort = float(msg.data)

    def _effort_cb(self, msg: Float64):
        self.effort = float(msg.data)

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)

    def _detected_cb(self, msg: Bool):
        self.detected = bool(msg.data)

    def _dsc_cb(self, msg: Float64):
        self.dsc = float(msg.data)

    def _heading_cb(self, msg: Float64):
        self.px_heading = float(msg.data)

    def execute(self) -> Status:
        self.node.get_logger().info(f"[{self.name}] We are executing Finding... track: {self.arena}")        
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

        self.yaw_effort_pub.publish(Float64(data=float(self.effort * (1 if self.arena == "B" else -1))))
        self.speed_effort_pub.publish(Float64(data=self.speed_effort))
        return Status.RUNNING

class Finding_Fallback(BaseFallback):
    """
    """
    def __init__(self, name: str = "Mission2_Fallback", node=None):
        super().__init__(name, node=node)
    
    def fallback(self) -> Status:
        return Status.FAILURE
