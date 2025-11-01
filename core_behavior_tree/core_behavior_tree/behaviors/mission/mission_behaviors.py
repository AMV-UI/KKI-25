from ..base_behavior import BaseBehavior
from py_trees.common import Status
from std_msgs.msg import UInt8
from core.utils.config import Topic

class BaseExecution(BaseBehavior):
    """Base class for mission execution behaviors"""
    
    def __init__(self, name: str, node=None):
        super().__init__(name, node=node)
        self.counter_pub = None
        self.mission_counter = 0
    
    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.mission_counter_pub = Topic.mission_counter.createPublisher(self.node)
    
    def update(self) -> Status:
        status = self.execute()

        match status:
            case Status.SUCCESS:
                self.node.get_logger().info(f"[{self.name}] MISSION {self.mission_counter} SUCCESS")
                self.mission_counter += 1
                self.mission_counter_pub.publish(UInt8(data=self.mission_counter))
            case Status.RUNNING:
                self.node.get_logger().info(f"[{self.name}] Mission {self.mission_counter} RUNNING")
            case Status.FAILURE:
                self.node.get_logger().info(f"[{self.name}] Mission {self.mission_counter} FAILURE")

        return status
    
    def execute(self) -> Status:
        """Must be overridden in child classes"""
        raise NotImplementedError("You must implement execute() in your subclass.")


class BaseFallback(BaseBehavior):
    """Base class for fallback actions"""

    def __init__(self, name: str, node=None):
        super().__init__(name, node=node)

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
    
    def update(self) -> Status:
        return self.fallback()

    def fallback(self) -> Status:
        """Must be overridden in child classes"""
        raise NotImplementedError("You must implement fallback() in your subclass.")
