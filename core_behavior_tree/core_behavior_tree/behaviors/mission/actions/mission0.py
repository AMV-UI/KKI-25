from ..mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64, UInt8, String
from core.utils.config import Topic
from core.mission.find_mode import FindMode
from core.mission.frame_counter import FrameCounter
from core.mission.docking import Docking
from core_msgs.msg import Pixhawk
from core.utils.config import Param, PxMode


import time
class Mission0_Execution(BaseExecution):
    """
    Main execution: Set inital heading and finding the buoy
    - Fallback: if pxmode is still on hold
    """
    def __init__(self, name: str = "Mission0_Execution", node=None):
        super().__init__(name, node=node)
        self.node = node
        self.pixhawk = None
        self.initial_heading = -361  # (Max -360 until 360) So means is not setup yet
        self.gps_ready = False
        self.arena = Param.TRACK.getValue(self.node)
        self.hold = True

        self.time_threshold = 1
        self.frame_counter = FrameCounter(self.time_threshold)

        self.dsc = 160.0 if self.arena == "A" else -160.0
        self.speed_effort = 100.0


    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.pixhawk = Pixhawk()
        
        self.initial_heading_pub = Topic.initial_heading.createPublisher(self.node)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)

        self.pxmode_sub = Topic.pxmode.createSubscriber(self.node, self._pxmode_cb)
        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)

    def _pxmode_cb(self, msg: String):
        if msg.data != PxMode.HOLD:
            self.hold = False
        else:
            self.hold = True

    def _heading_cb(self, msg: Float64):
        self.px_heading = float(msg.data)

    def _detected_cb(self, msg: Bool):
        self.detected = bool(msg.data)

    def execute(self) -> Status:
        self.node.get_logger().info(f"[{self.name}] Initial EXECUTION mode active... GPS Ready: {self.gps_ready}")

        if self.hold:
            return Status.FAILURE

        self.yaw_effort_pub.publish(Float64(data=self.dsc))
        self.speed_effort_pub.publish(Float64(data=self.speed_effort))
        
        if self.detected:
            self.initial_heading_pub.publish(self.heading_deg)
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                self.node.get_logger().info(f"[{self.name}] Target found -> switching to execution")
                return Status.SUCCESS
        else:
            self.frame_counter.reset()
        
        self.node.get_logger().info(
            f"[{self.name}] Searching for target (detected: {self.detected})",
            throttle_duration_sec=5.0
        )

        return Status.RUNNING


class Mission0_Fallback(BaseFallback):
    """
    Fallback: Remote still on hold and updating docking position
    - Execution: if remote change other than hold
    """
    def __init__(self, name: str = "Mission0_Fallback", node=None):
        super().__init__(name, node=node)
        self.node = node
        self.pixhawk = Pixhawk()
        self.gps_ready = False
        self.hold = True

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.pxmode_sub = Topic.pxmode.createSubscriber(self.node, self._pxmode_cb)
        self.pixhawk_sub = Topic.pixhawk.createSubscriber(self.node, self._pixhawk_cb)

    def _pxmode_cb(self, msg: String):
        if msg.data != PxMode.HOLD:
            self.hold = False
        else:
            self.hold = True

    def _pixhawk_cb(self, msg: Pixhawk):
        self.pixhawk = msg
        if msg.lat != 0.0 and msg.lon != 0.0:
            self.gps_ready = True

    def set_docking_coordinates(self):
        """Save Pixhawk coordinates for docking mission"""
        Param.DOCKING_LAT.setParam(self.node, self.pixhawk.lat)
        Param.DOCKING_LON.setParam(self.node, self.pixhawk.lon)

    def fallback(self) -> Status:        
        self.node.get_logger().info(f"[{self.name}] Initial EXECUTION mode active... GPS Ready: {self.gps_ready}")

        if self.gps_ready:
            self.set_docking_coordinates(self)
        if not self.hold:
            Status.FAILURE

        return Status.RUNNING