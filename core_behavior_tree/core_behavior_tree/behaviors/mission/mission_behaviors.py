from ..base_behavior import BaseBehavior
from py_trees.common import Status
from std_msgs.msg import UInt8
from core.utils.config import Topic

class BaseExecution(BaseBehavior):
    """Base class for mission execution behaviors"""
    
    def __init__(self, name: str, node=None, mission=None):
        super().__init__(name, node=node)
        self.counter_pub = None
        self.mission = mission
        self.has_published_mission = False
    
    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.base_mission_pub = Topic.mission.createPublisher(self.node)
    
    def initialise(self) -> None:
        pass
    
    def update(self) -> Status:
        if self.mission is not None and not self.has_published_mission:
            self.base_mission_pub.publish(UInt8(data=self.mission))
            self.node.get_logger().info(f"[{self.name}] Published mission ID: {self.mission}")
            self.has_published_mission = True
            
        status = self.execute()
        
        # Reset publish flag if we are no longer running (so next time we start we publish again)
        if status != Status.RUNNING:
            self.has_published_mission = False
            
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
