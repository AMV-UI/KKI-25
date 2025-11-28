from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import String
from core.utils.config import Topic

class Reverse_Execution(BaseExecution):
    """
    Main execution: Set inital heading and finding the buoy
    - Fallback: if pxmode is still on hold
    """
    def __init__(self, name, node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node
        self.arena = "B"

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.arena_pub = Topic.arena.createPublisher(self.node)
        self.arena_sub = Topic.arena.createSubscriber(self.node, self._arena_cb)

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)

    def execute(self) -> Status:
        self.node.get_logger().info(f"[{self.name}] Reverse arena...{self.arena}", throttle_duration_sec=1.0)
        self.arena_pub.publish(String(data="A" if self.arena == "B" else "B"))
        return Status.SUCCESS


class Reverse_Fallback(BaseFallback):
    """
    Fallback: Remote still on hold and updating docking position
    - Execution: if remote change other than hold
    """
    def __init__(self, name, node=None):
        super().__init__(name, node=node)

    def fallback(self) -> Status:        
        return Status.FAILURE