from ..mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64
from core.utils.config import Topic
from core.utils.mission.find_mode import FindMode
from core.utils.mission.frame_counter import FrameCounter

class Mission2_Execution(BaseExecution):
    """
    Combined behavior: FIND_STEP_TWO -> STEP_TWO
    - Phase "finding": search with find_mode range(1), wait until detected for N frames
    - Phase "approach": navigate to target, when lost for N frames -> SUCCESS
    """
    def __init__(self, name: str = "Mission2_Execution"):
        super().__init__(name)
        self.find_mode = None
        self.frame_counter = None
        self.detected = False
        self.dsc = 160.0
        self.px_heading = 0.0
        self.phase = "finding"  # or "approach"

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.find_mode = FindMode(self.node)
        self.frame_counter = FrameCounter(2)

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.dsc_sub = Topic.dsc.createSubscriber(self.node, self._dsc_cb)
        self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)

    def _detected_cb(self, msg: Bool):
        self.detected = bool(msg.data)

    def _dsc_cb(self, msg: Float64):
        self.dsc = float(msg.data)

    def _heading_cb(self, msg: Float64):
        self.px_heading = float(msg.data)

    def execute(self) -> Status:
        if self.phase == "finding":
            # mission_find_step_two logic
            self.find_mode.current_heading = self.find_mode.get_heading(self.px_heading)
            self.find_mode.set_range(1)
            
            if self.detected:
                self.frame_counter.is_started()
                if self.frame_counter.is_enough():
                    self.frame_counter.reset()
                    self.phase = "approach"
                    self.node.get_logger().info(f"[{self.name}] Tower found -> switching to APPROACH")
                    dsc_state = self.dsc
            else:
                self.frame_counter.reset()
                dsc_state = self.find_mode.get_state(self.px_heading)
                
            self.yaw_effort_pub.publish(Float64(data=dsc_state))
            
            self.node.get_logger().info(
                f"[{self.name}] Current Heading: {self.px_heading}, Range: [{self.find_mode.range_low}, {self.find_mode.range_high}]",
                throttle_duration_sec=1.0
            )
            
            return Status.RUNNING

        # phase == "approach"
        # mission_step_two logic
        dsc_state = self.dsc
        
        if not self.detected:
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                self.node.get_logger().info(f"[{self.name}] Lost tower -> STEP_TWO complete")
                return Status.SUCCESS
        else:
            self.frame_counter.reset()

        self.yaw_effort_pub.publish(Float64(data=dsc_state))
        return Status.RUNNING


class Mission2_Fallback(BaseFallback):
    """
    Fallback for Mission 2: search pattern with range(1)
    """
    def __init__(self, name: str = "Mission2_Fallback"):
        super().__init__(name)
        self.find_mode = None
        self.px_heading = 0.0
        self.dsc = 160.0

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
            search_state = self.find_mode.get_state(self.px_heading)
            self.yaw_effort_pub.publish(Float64(data=search_state))
            self.node.get_logger().info(
                f"[{self.name}] Search fallback range(1) (state={search_state})",
                throttle_duration_sec=5.0
            )
        except Exception:
            self.yaw_effort_pub.publish(Float64(data=self.dsc))
            self.node.get_logger().info(f"[{self.name}] Fallback holding (dsc={self.dsc})")

        return Status.RUNNING
