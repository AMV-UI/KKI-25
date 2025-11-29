from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from core.mission.frame_counter import FrameCounter
from std_msgs.msg import String, UInt8
from core.utils.config import Topic, MissionStatus

class Photo_Execution(BaseExecution):
    """
    Main execution: Set inital heading and finding the buoy
    - Fallback: if pxmode is still on hold
    """
    def __init__(self, name, node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node
        self.arena = "B"
        self.time_threshold = 2.0
        self.mission = mission
        self.mission_type = MissionStatus.BUOY

        self.greenbox = None
        self.bluebox = None

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.greenbox_pub = Topic.image_green_box.createPublisher(self.node)
        self.bluebox_pub = Topic.image_blue_box.createPublisher(self.node)
        self.frame_counter = FrameCounter(self.time_threshold)
        self.mission_pub = Topic.mission.createPublisher(self.node)

        self.green_box_encoded_sub = Topic.green_box_encoded.createSubscriber(
            self.node,
            self._green_box_cb
        )
        self.blue_box_encoded_sub = Topic.blue_box_encoded.createSubscriber(
            self.node,
            self._blue_box_cb
        )
        self.mission_type_sub = Topic.mission_type.createSubscriber(
            self.node,
            self._mission_type_cb
        )
    
    def _mission_type_cb(self, msg: String):
        self.mission_type = str(msg.data)

    def _green_box_cb(self, msg: String):
        self.greenbox = str(msg.data)
    
    def _blue_box_cb(self, msg: String):
        self.bluebox = str(msg.data)

    def execute(self) -> Status:
        self.node.get_logger().info(f"[{self.name}] Taking Photo...", throttle_duration_sec=1.0)

        if(self.mission_type == MissionStatus.GREEN_BOX and self.greenbox != None):
            self.greenbox_pub.publish(self.greenbox)
        elif (self.mission_type == MissionStatus.BLUE_BOX and self.bluebox != None):
            self.bluebox_pub.publish(self.bluebox)
            
        self.frame_counter.is_started()            
        if self.frame_counter.is_enough():
            self.frame_counter.reset()
            self.node.get_logger().info(
                f"[{self.name}] Finding Complete"
            )
            self.mission_pub.publish(UInt8(data=self.mission))
            return Status.SUCCESS

        return Status.RUNNING


class Photo_Fallback(BaseFallback):
    """
    Fallback: Remote still on hold and updating docking position
    - Execution: if remote change other than hold
    """
    def __init__(self, name, node=None):
        super().__init__(name, node=node)

    def fallback(self) -> Status:        
        return Status.FAILURE