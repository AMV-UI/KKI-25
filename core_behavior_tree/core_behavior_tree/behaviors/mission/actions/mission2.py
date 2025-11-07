
from ..mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64, UInt8
from core.utils.config import Topic, Param
from core.mission.find_mode import FindMode
from core.mission.frame_counter import FrameCounter
from core_msgs.msg import Pixhawk

import time


class Mission2_Execution(BaseExecution):
    """
    Combined behavior: FIND_STEP_TWO -> STEP_TWO
    - Phase "finding": search with find_mode range(1), wait until detected for N frames
    - Phase "approach": navigate to target, when lost for N frames -> SUCCESS (ready for docking)
    """
    def __init__(self, name: str = "Mission2_Execution", node=None):
        super().__init__(name, node=node)
        self.node = node
        self.find_mode = None
        self.frame_counter = None
        self.detected = False
        self.arena = Param.TRACK.getValue(self.node)
        self.dsc = -160.0 if self.arena == "A" else 160.0  #Reverse effort untuk mission 2

        self.px_heading = 0.0
        self.phase = "finding"        

        self.speed_effort = 100.0

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.find_mode = FindMode(self.node)
        self.frame_counter = FrameCounter(2)
        self.target_lat = Param.DOCKING_LAT.getValue(self.node)
        self.target_lon = Param.DOCKING_LON.getValue(self.node)

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.dsc_sub = Topic.dsc.createSubscriber(self.node, self._dsc_cb)
        self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)
        
        self.mission_pub = Topic.mission.createPublisher(self.node)
        self.mission_pub.publish(UInt8(data=2))

    def _detected_cb(self, msg: Bool):
        self.detected = bool(msg.data)

    def _dsc_cb(self, msg: Float64):
        self.dsc = float(msg.data)

    def _heading_cb(self, msg: Float64):
        self.px_heading = float(msg.data)

    def execute(self) -> Status:
        if self.phase == "finding":
            self.node.get_logger().info(f"[{self.name}] We are executing Mission 2...")
            self.find_mode.current_heading = self.find_mode.get_heading(self.px_heading)
            self.find_mode.set_range(1)
            
            if self.detected:
                self.frame_counter.is_started()
                if self.frame_counter.is_enough():
                    self.frame_counter.reset()
                    self.phase = "approach"
                    self.node.get_logger().info(f"[{self.name}] Tower found -> switching to APPROACH")
                    yaw_effort = self.dsc
            else:
                self.frame_counter.reset()
                yaw_effort = self.find_mode.get_state(self.px_heading)

            self.yaw_effort_pub.publish(Float64(data=float(yaw_effort)))
            self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))
            
            self.node.get_logger().info(
                f"[{self.name}] Current Heading: {self.px_heading}, Range: [{self.find_mode.range_low}, {self.find_mode.range_high}]",
                throttle_duration_sec=1.0
            )
            
            return Status.RUNNING

        # phase == "approach"
        yaw_effort = self.dsc
        
        if not self.detected:
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                self.node.get_logger().info(
                    f"[{self.name}] Lost tower -> STEP_TWO complete, ready for docking"
                )
                return Status.SUCCESS
        else:
            self.frame_counter.reset()

        self.yaw_effort_pub.publish(yaw_effort)
        self.speed_effort_pub.publish(Float64(data=self.speed_effort))
        return Status.RUNNING

        # self.node.get_logger().info(f"[{self.name}] We are executing Mission 2...")

        # if not self.flag:
        #     self.node.get_logger().info(f"[{self.name}] Mission 2 flag is False -> Switching to FALLBACK")
        #     return Status.FAILURE

        # else:
        #     self.node.get_logger().info(f"[{self.name}] Mission 2 flag is True -> Mission SUCCESS")
        #     time.sleep(2)  # Simulate some processing time
        #     return Status.SUCCESS


class Mission2_Fallback(BaseFallback):
    """
    Fallback for Mission 2: search pattern with range(1)
    """
    def __init__(self, name: str = "Mission2_Fallback", node=None):
        super().__init__(name, node=node)
        self.node = node
        self.find_mode = None
        self.px_heading = 0.0
        self.dsc = 160.0
        self.speed_effort = 100.0

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.find_mode = FindMode(self.node)

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)

        self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)
        self.dsc_sub = Topic.dsc.createSubscriber(self.node, self._dsc_cb)

    def _heading_cb(self, msg: Float64):
        self.px_heading = float(msg.data)

    def _dsc_cb(self, msg: Float64):
        self.dsc = float(msg.data)

    def fallback(self) -> Status:
        try:
            self.find_mode.set_range(1)
            self.find_mode.current_heading = self.find_mode.get_heading(self.px_heading)

            yaw_effort_searching = self.find_mode.get_state(self.px_heading)

            self.yaw_effort_pub.publish(Float64(data=yaw_effort_searching))
            self.speed_effort_pub.publish(Float64(data=self.speed_effort))


            self.node.get_logger().info(
                f"[{self.name}] Search fallback range(1) (state={yaw_effort_searching})",
                throttle_duration_sec=5.0
            )
        except Exception:
            self.yaw_effort_pub.publish(Float64(data=self.dsc))
            self.speed_effort_pub.publish(Float64(data=self.speed_effort))

            self.node.get_logger().info(f"[{self.name}] Fallback holding (dsc={self.dsc})")

        return Status.RUNNING
        # self.node.get_logger().info(f"[{self.name}] We are in FALLBACK mode for Mission 2...")
        # self.node.get_logger().info(f"[{self.name}] Entering FALLBACK mode, doing search...")
        # time.sleep(2)  # Simulate search time
        # self.node.get_logger().info(f"[{self.name}] Condition Satisfied -> Switching to EXECUTION")
        # Param.FLAG.setParam(self.node, True)