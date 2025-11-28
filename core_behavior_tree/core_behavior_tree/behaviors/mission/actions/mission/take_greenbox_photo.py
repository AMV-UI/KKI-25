from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import String, Float64, UInt8
from core.utils.config import Topic, MissionStatus
from core.mission.frame_counter import FrameCounter
import time


class TakeGreenBoxPhoto_Execution(BaseExecution):
    """
    Main execution: Set inital heading and finding the buoy
    - Fallback: if pxmode is still on hold
    """
    def __init__(self, name, node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node
        self.freeze_time = 2  # seconds to freeze while taking photo

    def initialise(self):
        self.start_time = time.time()
        self.mission_pub.publish(UInt8(data=MissionStatus.TAKE_GREEN_BOX_PHOTO.value))
        return super().initialise()

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.mission_pub = Topic.mission.createPublisher(self.node)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)

    def execute(self) -> Status:
        # Freeze position and take photo
        if time.time() - self.start_time < self.freeze_time:
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.speed_effort_pub.publish(Float64(data=0.0))
            self.node.get_logger().info(f"[{self.name}] Taking photo... ({time.time() - self.start_time:.2f}s/{self.freeze_time}s)", throttle_duration_sec=1.0)
            return Status.RUNNING
        else:
            self.node.get_logger().info(f"[{self.name}] {self.freeze_time}s freeze time elapsed! MISSION SUCCESS", throttle_duration_sec=1.0)
            return Status.SUCCESS


class TakeGreenBoxPhoto_Fallback(BaseFallback):
    """
    Fallback: Remote still on hold and updating docking position
    - Execution: if remote change other than hold
    """
    def __init__(self, name, node=None):
        super().__init__(name, node=node)

    def fallback(self) -> Status:        
        return Status.FAILURE
