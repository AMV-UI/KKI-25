from ..mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64, UInt8, String
from core.utils.config import Topic
from core.mission.frame_counter import FrameCounter
from core_msgs.msg import Pixhawk


import time
class Mission1_Execution(BaseExecution):
    """
    Main execution: APPROACH to 2 Buoys (Different color, Green and Red) phase only
    - Navigate toward detected target using DSC from vision
    - When target is lost for N frames -> SUCCESS (mission complete)
    - If target not detected -> FAILURE (triggers fallback to search)
    - Store Pixhawk coordinate to docking station after to use in the last mission
    """
    def __init__(self, name, node=None):
        super().__init__(name, node=node)
        self.node = node
        self.frame_counter = None
        self.detected = True
        self.dsc = 0.0
        self.speed_effort = 100.0
        self.pixhawk = None
        self.arena = "B"
        self.initial_heading = -361
        self.gps_ready = False
        self.time_threshold = 2 # in Seconds    
        
    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.frame_counter = FrameCounter(self.time_threshold)
        self.pixhawk = Pixhawk()

        self.mission_pub = Topic.mission.createPublisher(self.node)
        self.mission_pub.publish(UInt8(data=1))

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
        self.arena_sub = Topic.arena.createSubscriber(self.node, self._arena_cb)
        self.pixhawk_sub = Topic.pixhawk.createSubscriber(self.node, self._pixhawk_cb)
        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.dsc_sub = Topic.dsc.createSubscriber(self.node, self._dsc_cb)
    
    def _pixhawk_cb(self, msg: Pixhawk):
        self.pixhawk = msg
        if msg.lat != 0.0 and msg.lon != 0.0:
            self.gps_ready = True

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)

    def _detected_cb(self, msg: Bool):
        self.detected = bool(msg.data)

    def _dsc_cb(self, msg: Float64):
        self.dsc = float(msg.data)

    def execute(self) -> Status:
        self.node.get_logger().info(f"[{self.name}] Mission 1 EXECUTION mode active... GPS Ready: {self.gps_ready}")

        if not self.detected:
            self.frame_counter.is_started() 
            self.counter = time.time()
            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                self.node.get_logger().info(f"[{self.name}] Condition Succeeded from EXECUTION -> Mission COMPLETE")
                return Status.SUCCESS

            self.node.get_logger().info(f"[{self.name}] Condition Failed -> Switching to FALLBACK")
            return Status.RUNNING

        self.frame_counter.reset()

        self.yaw_effort_pub.publish(Float64(data=self.dsc))
        self.speed_effort_pub.publish(Float64(data=self.speed_effort))
            
        self.node.get_logger().info(
            f"[{self.name}] Approaching target - DSC: {self.dsc}",
            throttle_duration_sec=2.0
            )

        
        return Status.RUNNING


class Mission1_Fallback(BaseFallback):
    """
    Fallback: FINDING phase
    - Search for target using constant yaw + find_mode
    - When target found consistently -> returns SUCCESS (lets execution take over)
    """
    def __init__(self, name: str = "Mission1_Fallback", node=None):
        super().__init__(name, node=node)
        self.node = node
        self.frame_counter = None
        self.detected = False

        self.px_heading = 0.0
        self.arena = "B"
        self.dsc = 160.0 if self.arena == "A" else -160.0
        self.speed_effort = 100.0
        self.time_threshold = 2 # in Seconds

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.frame_counter = FrameCounter(self.time_threshold)

        self.initial_heading_pub = Topic.initial_heading.createPublisher(self.node)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)

        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)
        self.arena_sub = Topic.arena.createSubscriber(self.node, self._arena_cb)

        
    def _detected_cb(self, msg: Bool):
        self.detected = bool(msg.data)

    def _heading_cb(self, msg: Float64):
        self.px_heading = float(msg.data)
        self.initial_heading_pub.publish(self.px_heading)

    def fallback(self) -> Status:

        self.node.get_logger().info(f"[{self.name}] We are in FALLBACK mode for Mission 1...")

        self.yaw_effort_pub.publish(Float64(data=self.dsc))
        self.speed_effort_pub.publish(Float64(data=self.speed_effort))
        
        if self.detected:
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                self.node.get_logger().info(f"[{self.name}] Target found -> switching to execution")
                return Status.FAILURE
        else:
            self.frame_counter.reset()
        
        self.node.get_logger().info(
            f"[{self.name}] Searching for target (detected: {self.detected})",
            throttle_duration_sec=5.0
        )
        
        return Status.RUNNING