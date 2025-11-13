from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64, UInt8, String, UInt32
from core.utils.config import Topic
from core.mission.find_mode import FindMode
from core.mission.frame_counter import FrameCounter
from core.mission.docking import Docking
from core_msgs.msg import Pixhawk
from core.utils.config import Param, PxMode


import time
class Straight_Execution(BaseExecution):
    """
    Main execution: Set inital heading and finding the buoy
    - Fallback: if pxmode is still on hold
    """
    def __init__(self, name: str = "Straight_Execution", node=None):
        super().__init__(name, node=node)
        self.node = node
        self.success = False
        self.effort = 100.0

    def initialise(self):
        self.effort = 100.0
        self.success = False

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        
        self.mission_sub = Topic.tuning_mission.createSubscriber(
            self.node,
            self._mission_cb
        )        
        self.effort_sub = Topic.tuning_effort_st.createSubscriber(
            self.node,
            self._effort_cb
        )        

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)

    def _mission_cb(self, msg: UInt8):
        if(msg.data == 1):
            self.success = True

    def _effort_cb(self, msg: Float64):
        self.effort = float(msg.data)

    def execute(self) -> Status:
        self.node.get_logger().info(f"[{self.name}] Tuning")
        if(self.success):
            return Status.SUCCESS

        self.yaw_effort_pub.publish(Float64(data=0.0))
        self.speed_effort_pub.publish(Float64(data=self.effort))

        return Status.RUNNING


class Straight_Fallback(BaseFallback):
    """
    Fallback: Remote still on hold and updating docking position
    - Execution: if remote change other than hold
    """
    def __init__(self, name: str = "Straight_Fallback", node=None):
        super().__init__(name, node=node)

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)

    def fallback(self) -> Status:        
        return Status.FAILURE