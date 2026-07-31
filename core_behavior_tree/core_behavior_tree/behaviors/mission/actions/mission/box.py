from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import String
from core.utils.config import Topic, MissionStatus

class Box_Execution(BaseExecution):
    """
    Main execution: Set inital heading and finding the buoy
    - Fallback: if pxmode is still on hold
    """
    def __init__(self, name, node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node
        self.mission_type = MissionStatus.BUOY
        self.mission = mission

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.mission_type_pub = Topic.mission_type.createPublisher(self.node)
        self.mission_pub = Topic.mission.createPublisher(self.node)
        self.mission_type_sub = Topic.mission_type.createSubscriber(
            self.node,
            self._mission_type_cb
        )
    
    def _mission_type_cb(self, msg: String):
        self.mission_type = str(msg.data)

    def execute(self) -> Status:
        if self.mission == 6:
            self.mission_type = MissionStatus.GREEN_BOX
        elif self.mission == 8:
            self.mission_type = MissionStatus.BLUE_BOX
        elif self.mission == 10:
            self.mission_type = MissionStatus.DOCKING
        else:
            if (self.mission_type == MissionStatus.BUOY):
                self.mission_type = MissionStatus.GREEN_BOX                   
            elif (self.mission_type == MissionStatus.GREEN_BOX):
                self.mission_type = MissionStatus.BLUE_BOX
            elif (self.mission_type == MissionStatus.BLUE_BOX):
                self.mission_type = MissionStatus.DOCKING

        self.mission_type_pub.publish(String(data=self.mission_type))
        if self.mission is not None:
            from std_msgs.msg import UInt8
            self.mission_pub.publish(UInt8(data=self.mission))
            
        self.node.get_logger().info(f"[{self.name}] Change Mission... to {self.mission_type}")
        return Status.SUCCESS


class Box_Fallback(BaseFallback):
    """
    Fallback: Remote still on hold and updating docking position
    - Execution: if remote change other than hold
    """
    def __init__(self, name, node=None):
        super().__init__(name, node=node)

    def fallback(self) -> Status:        
        return Status.FAILURE