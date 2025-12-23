from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import String, Float64, UInt8
from core.utils.config import Topic, MissionStatus
from core.mission.frame_counter import FrameCounter
import time


class FindGreenBox_Execution(BaseExecution):
    """
    Main execution: Set inital heading and finding the buoy
    - Fallback: if pxmode is still on hold
    """
    def __init__(self, name, node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node
        self.mission = MissionStatus.FIND_GREEN_BOX.value
        self.dsc = 0.0
        self.detected = True
        self.speed_effort = 150
        self.time_threshold = 1

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.frame_counter = FrameCounter(self.time_threshold)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.mission_pub = Topic.mission.createPublisher(self.node)
        self.dsc_sub = Topic.dsc.createSubscriber(self.node, self._dsc_cb)

    def _dsc_cb(self, msg):
        self.dsc = float(msg.data)
    
    def _detected_cb(self, msg):
        self.detected = bool(msg.data)

    def execute(self) -> Status:
        # Move according to dsc
        # Assuming greenbox always detected in this mission (also at the start)
        # SUCCESS when close enough (handled in inference) (detected = False)
        # Terminates mission even if not found

        self.mission_pub.publish(UInt8(data=self.mission))

        self.node.get_logger().info(f"[{self.name}] Executing Finding Green Box", throttle_duration_sec=1.0)
        if not self.detected:
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.speed_effort_pub.publish(Float64(data=0.0))
            self.node.get_logger().info(f"[{self.name}] Waiting for green box undetection... (Motor effort halted)", throttle_duration_sec=1.0)
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.node.get_logger().info(f"[{self.name}] Green box undetected! MISSION SUCCESS", throttle_duration_sec=1.0)
                self.frame_counter.reset()
                return Status.SUCCESS
            return Status.RUNNING
        self.frame_counter.reset()

        self.node.get_logger().info(f"[{self.name}] Greenbox still detected, adjusting position... dsc: {self.dsc}", throttle_duration_sec=1.0)
        self.yaw_effort_pub.publish(Float64(data=float(self.dsc)))
        self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))
        return Status.RUNNING


class FindGreenBox_Fallback(BaseFallback):
    """
    Fallback: Remote still on hold and updating docking position
    - Execution: if remote change other than hold
    """
    def __init__(self, name, node=None):
        super().__init__(name, node=node)

    def fallback(self) -> Status:        
        return Status.FAILURE
