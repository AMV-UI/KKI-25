from ..mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64
from core.utils.config import Topic, Param
from core.utils.mission.find_mode import FindMode
from core.utils.mission.frame_counter import FrameCounter


class Mission3_Execution(BaseExecution):
    """
    Combined behavior: FIND_STEP_THREE -> STEP_THREE -> MANUVER
    - Phase "finding": search with find_mode range(2), wait until detected
    - Phase "approach": navigate to target, when lost -> switch track param -> MANUVER phase
    - Phase "manuver": perform maneuver based on detection status
    """

    def __init__(self, name: str = "Mission3_Execution"):
        super().__init__(name)
        self.find_mode = None
        self.frame_counter = None
        self.frame_counter_manuver = None
        self.detected = False
        self.dsc = 160.0
        self.px_heading = 0.0
        self.phase = "finding"  # or "approach" or "manuver"
        self.isChange = False
        self.manuver_detected = False

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.find_mode = FindMode(self.node)
        self.frame_counter = FrameCounter(2)
        self.frame_counter_manuver = FrameCounter(3)

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
            # mission_find_step_three logic
            self.find_mode.set_range(2)
            self.find_mode.current_heading = self.find_mode.get_heading(self.px_heading)

            if self.detected:
                self.frame_counter.is_started()
                if self.frame_counter.is_enough():
                    self.frame_counter.reset()
                    self.phase = "approach"
                    self.node.get_logger().info(f"[{self.name}] Buoy 3 found -> switching to APPROACH")
                    dsc_state = self.dsc
            else:
                self.frame_counter.reset()
                dsc_state = self.find_mode.get_state(self.px_heading)

            self.yaw_effort_pub.publish(Float64(data=dsc_state))
            self.node.get_logger().info(
                f"[{self.name}] Initial heading: {self.find_mode.initial_heading}",
                throttle_duration_sec=1.0
            )
            return Status.RUNNING

        if self.phase == "approach":
            # mission_step_three logic
            # Flip track parameter
            if self.node.get_parameter(Param.TRACK).value == "A" and not self.isChange:
                self.node.set_parameters([rclpy.parameter.Parameter(Param.TRACK, rclpy.Parameter.Type.STRING, "B")])
                self.isChange = True
                self.node.get_logger().info(f"[{self.name}] Track switched A -> B")
            elif self.node.get_parameter(Param.TRACK).value == "B" and not self.isChange:
                self.node.set_parameters([rclpy.parameter.Parameter(Param.TRACK, rclpy.Parameter.Type.STRING, "A")])
                self.isChange = True
                self.node.get_logger().info(f"[{self.name}] Track switched B -> A")

            dsc_state = self.dsc

            if not self.detected:
                self.frame_counter.is_started()
                if self.frame_counter.is_enough():
                    self.frame_counter.reset()
                    self.phase = "manuver"
                    self.node.get_logger().info(f"[{self.name}] Lost buoy -> starting MANUVER")
            else:
                self.frame_counter.reset()

            self.yaw_effort_pub.publish(Float64(data=dsc_state))
            return Status.RUNNING

        # phase == "manuver"
        # mission_manuver logic
        track = self.node.get_parameter(Param.TRACK).value

        # Case: detected during manuver
        if self.detected:
            self.manuver_detected = True
            dsc_state = -200 if track == "A" else 200  # Left if A, Right if B

        # Case: was detecting, now lost
        elif self.manuver_detected and not self.detected:
            self.frame_counter_manuver.is_started()
            dsc_state = -200 if track == "A" else 200

            if self.frame_counter_manuver.is_enough():
                self.frame_counter_manuver.reset()
                self.node.get_logger().info(f"[{self.name}] Manuver complete -> SUCCESS")
                return Status.SUCCESS

        # Case: never detected
        else:
            self.frame_counter_manuver.is_started()
            dsc_state = self.dsc  # Go straight

            if self.frame_counter_manuver.is_enough():
                self.frame_counter_manuver.reset()
                self.node.get_logger().info(f"[{self.name}] Manuver timeout -> SUCCESS")
                return Status.SUCCESS

        self.yaw_effort_pub.publish(Float64(data=dsc_state))
        self.node.get_logger().info(
            f"[{self.name}] Manuver phase - detected: {self.manuver_detected}",
            throttle_duration_sec=5.0
        )
        return Status.RUNNING


class Mission3_Fallback(BaseFallback):
    """
    Fallback for Mission 3: search pattern with range(2)
    """

    def __init__(self, name: str = "Mission3_Fallback"):
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
            self.find_mode.set_range(2)
            self.find_mode.current_heading = self.find_mode.get_heading(self.px_heading)
            search_state = self.find_mode.get_state(self.px_heading)
            self.yaw_effort_pub.publish(Float64(data=search_state))
            self.node.get_logger().info(
                f"[{self.name}] Search fallback range(2) (state={search_state})",
                throttle_duration_sec=5.0
            )
        except Exception:
            self.yaw_effort_pub.publish(Float64(data=self.dsc))
            self.node.get_logger().info(f"[{self.name}] Fallback holding (dsc={self.dsc})")

        return Status.RUNNING
