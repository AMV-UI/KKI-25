from ..base_behavior import BaseBehavior
from py_trees.common import Status
from std_msgs.msg import UInt8
from core.utils.config import Topic

class BaseExecution(BaseBehavior):
    """Base class for mission execution behaviors"""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.counter_pub = None
    
    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
    
    def update(self) -> Status:
        status = self.execute()
        return status
    
    def execute(self) -> Status:
        """Must be overridden in child classes"""
        raise NotImplementedError("You must implement execute() in your subclass.")


class BaseFallback(BaseBehavior):
    """Base class for fallback actions"""
    
    def __init__(self, name: str):
        super().__init__(name)
    
    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
    
    def update(self) -> Status:
        return self.fallback()

    def fallback(self) -> Status:
        """Must be overridden in child classes"""
        raise NotImplementedError("You must implement fallback() in your subclass.")
