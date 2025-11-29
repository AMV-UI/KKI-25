from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import UInt8
from core.utils.config import Topic

class Idle_Execution(BaseExecution):
    """
    """
    def __init__(self, name, node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node
        self.mission = -1

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.mission_sub = Topic.mission.createSubscriber(self.node, self._mission_cb)

    def _mission_cb(self, msg: UInt8):
        self.mission = int(msg.data)

    def execute(self) -> Status:
        if self.mission == 100:
            self.node.get_logger().info(f"[{self.name}] Idle mission...{self.mission}", throttle_duration_sec=1.0)
            return Status.SUCCESS

        return Status.RUNNING


class Idle_Fallback(BaseFallback):
    """
    Fallback: Remote still on hold and updating docking position
    - Execution: if remote change other than hold
    """
    def __init__(self, name, node=None):
        super().__init__(name, node=node)

    def fallback(self) -> Status:        
        return Status.FAILURE