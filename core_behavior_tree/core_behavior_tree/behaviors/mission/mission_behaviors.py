from ..base_behavior import BaseBehavior
from py_trees.common import Status
from std_msgs.msg import UInt8
from core.utils.config import BT, TopicFactory

class BaseExecution(BaseBehavior):
    """
    Base class for mission main execution behaviors that test conditions and report results.
    
    Required Blackboard Keys:
    - "mission_counter" (read): tracks current mission step.
    
    Child classes must:
    - Add any extra blackboard keys via `extra_keys`
    - Implement `execute()` to return Status
    """
    
    def __init__(self, name: str, extra_keys: dict = None):
        base_keys = {"mission_counter": BT.read}
        if extra_keys:
            base_keys.update(extra_keys)
        super().__init__(name, base_keys)
    
    def setup(self, **kwargs) -> None:
        self.counter_pub = TopicFactory("/mission_counter", UInt8).createPublisher(self.node)
    
    def update(self) -> Status:
        status = self.execute()
        counter = self.blackboard.mission_counter

        match status:
            case Status.SUCCESS:
                self.node.get_logger().info(f"[{self.name}] MISSION {counter} SUCCESS")
                self.counter_pub.publish(UInt8(data=counter + 1))
            case Status.RUNNING:
                self.node.get_logger().info(f"[{self.name}] Mission {counter} RUNNING")
            case Status.FAILURE:
                self.node.get_logger().info(f"[{self.name}] Mission {counter} FAILURE")

        return status
    
    def execute(self) -> Status:
        """
        Must be overridden in child classes to implement mission-specific logic.
        """
        raise NotImplementedError("You must implement execute() in your subclass.")


class BaseFallback(BaseBehavior):
    """
    Base class for fallback actions triggered after a mission failure.

    Required Blackboard Keys:
    - "mission_counter" (read): tracks the mission ID that failed.

    Child classes must:
    - Add any extra blackboard keys via `extra_keys`
    - Implement `fallback()` to define fallback action
    """
    
    def __init__(self, name: str, extra_keys: dict = None):
        base_keys = {"mission_counter": BT.read}
        if extra_keys:
            base_keys.update(extra_keys)
        super().__init__(name, base_keys)
    
    def update(self) -> Status:
        self.fallback()
        return Status.FAILURE

    def fallback(self):
        raise NotImplementedError("You must implement fallback() in your subclass.")
