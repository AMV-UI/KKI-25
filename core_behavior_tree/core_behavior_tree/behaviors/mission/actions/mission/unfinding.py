from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64, UInt8, String
from core.utils.config import Topic, MissionParams
from core.mission.frame_counter import FrameCounter
from core_msgs.msg import Pixhawk


import time
class Unfinding_Execution(BaseExecution):
    """
    Main execution: APPROACH to 2 Buoys (Different color, Green and Red) phase only
    - Navigate toward detected target using DSC from vision
    - When target is lost for N frames -> SUCCESS (mission complete)
    - If target not detected -> FAILURE (triggers fallback to search)
    - Store Pixhawk coordinate to docking station after to use in the last mission
    """
    def __init__(self, name: str = "Mission1_Execution", node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node
        self.frame_counter = None
        self.detected = True
        self.yaw_effort = MissionParams.unfinding_yaw_effort
        self.speed_effort = MissionParams.unfinding_speed_effort
        self.arena = "B"
        self.time_threshold = MissionParams.unfinding_time_threshold
        self.mission = mission
        
    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.frame_counter = FrameCounter(self.time_threshold)

        self.mission_pub = Topic.mission.createPublisher(self.node)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
        self.arena_sub = Topic.arena.createSubscriber(
            self.node, 
            self._arena_cb
        )
        self.detected_sub = Topic.detected.createSubscriber(
            self.node, 
            self._detected_cb
        )

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)

    def _detected_cb(self, msg: Bool):
        self.detected = bool(msg.data)

    def initialise(self) -> None:
        self.start_time = time.time()
        self.node.get_logger().info(f"[{self.name}] Initializing Unfinding Execution for {getattr(MissionParams, 'unfinding_duration', 5.0)}s")

    def execute(self) -> Status:
        elapsed = time.time() - self.start_time
        
        if elapsed >= getattr(MissionParams, 'unfinding_duration', 5.0):
            self.node.get_logger().info(f"[{self.name}] Unfinding maneuver complete.")
            # Stop the boat
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.speed_effort_pub.publish(Float64(data=0.0))
            
            # Move to next mission state
            if self.mission is not None:
                from std_msgs.msg import UInt8
                self.mission_pub.publish(UInt8(data=self.mission))
            return Status.SUCCESS

        # Turn maneuver similar to turn_next_buoy
        # Arena A -> Positive yaw, Arena B -> Negative yaw
        yaw = float(self.yaw_effort if self.arena == "A" else -self.yaw_effort)
        
        self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))
        self.yaw_effort_pub.publish(Float64(data=yaw))
        
        self.node.get_logger().info(f"[{self.name}] Unfinding... elapsed: {elapsed:.1f}s / {getattr(MissionParams, 'unfinding_duration', 5.0)}s", throttle_duration_sec=1.0)
        return Status.RUNNING


class Unfinding_Fallback(BaseFallback):
    """
    """
    def __init__(self, name: str = "Mission1_Fallback", node=None):
        super().__init__(name, node=node)

    def fallback(self) -> Status:
        return Status.Failure
