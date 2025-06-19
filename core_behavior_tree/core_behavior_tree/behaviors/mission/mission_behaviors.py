from ..base_behavior import BaseBehavior
import py_trees
from py_trees.common import Status
from std_msgs.msg import UInt8
import rclpy.qos
from core.utils.config import BT, TopicFactory

class BaseMission(BaseBehavior):
    """Base class for mission behaviors that test conditions and report results"""
    
    def __init__(self, name: str):
        keys_to_register = {
            "mission_counter": BT.read,
            "px_heading": BT.read
        }
        super(BaseMission, self).__init__(name, keys_to_register)
    
    def setup(self, **kwargs) -> None:
        self.counter_pub = TopicFactory("/mission_counter", UInt8).createPublisher(self.node)
    
    def update(self) -> Status:
        # This is a simplified implementation that uses px_heading to determine status
        # In a real implementation, you would have actual mission logic here
        
        # # Map px_heading to a status value for demonstration
        stuff = [Status.SUCCESS, Status.FAILURE, Status.RUNNING]
        status = stuff[round(self.blackboard.get("px_heading"))]
        ############################################################# Only for PlaceHolders for real logic
        
        if status == Status.SUCCESS:
            self.node.get_logger().info(f"MISSION {self.blackboard.mission_counter} SUCCESS, CONTINUING :)")
            self.counter_pub.publish(UInt8(data=self.blackboard.mission_counter + 1)) #### increment mission counter

        elif status == Status.RUNNING:
            self.node.get_logger().info(f"Still doing mission {self.blackboard.mission_counter} ...")
        
        return status


class FallbackAction(BaseBehavior):
    """Action to take when a mission fails"""
    
    def __init__(self, name: str):
        keys_to_register = {"mission_counter": BT.read}
        super(FallbackAction, self).__init__(name, keys_to_register)
        
    def update(self) -> Status:
        self.node.get_logger().info(f"Mission {self.blackboard.mission_counter} Failure, falling back...")
        return Status.FAILURE