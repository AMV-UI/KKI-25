from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from core.mission.frame_counter import FrameCounter
from std_msgs.msg import String, UInt8, Float64
from core.utils.config import Topic, MissionStatus, MissionParams

class Photo_Execution(BaseExecution):
    """
    Main execution: Set inital heading and finding the buoy
    - Fallback: if pxmode is still on hold
    """
    def __init__(self, name, node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node
        self.arena = "B"
        self.time_threshold = MissionParams.photo_time_threshold
        self.mission = mission
        self.mission_type = MissionStatus.BUOY

        self.greenbox = None
        self.bluebox = None
        self.current = False

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.greenbox_pub = Topic.image_green_box.createPublisher(self.node)
        self.bluebox_pub = Topic.image_blue_box.createPublisher(self.node)
        self.frame_counter = FrameCounter(self.time_threshold)
        self.mission_pub = Topic.mission.createPublisher(self.node)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)

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

    def initialise(self) -> None:
        super().initialise()
        self.has_saved_photo = False

    def _save_photo_to_disk(self, b64_data, prefix):
        try:
            if not getattr(self, "has_saved_photo", False):
                import base64
                import os
                from datetime import datetime
                
                save_dir = os.path.expanduser("~/KKI-25/core_perception/photos")
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir, exist_ok=True)
                
                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                
                # Split the combined string to get both cameras
                parts = b64_data.split("|||")
                top_b64 = parts[0]
                bot_b64 = parts[1] if len(parts) > 1 else None
                
                # Save Top Camera
                filename_top = os.path.join(save_dir, f"{timestamp}_{prefix}_TopCamera_Auto.jpg")
                img_data_top = base64.b64decode(top_b64)
                with open(filename_top, "wb") as f:
                    f.write(img_data_top)
                self.node.get_logger().info(f"[{self.name}] Auto-saved Top Camera photo to {filename_top}")
                
                # Save Bottom Camera if available
                if bot_b64:
                    filename_bot = os.path.join(save_dir, f"{timestamp}_{prefix}_BottomCamera_Auto.jpg")
                    img_data_bot = base64.b64decode(bot_b64)
                    with open(filename_bot, "wb") as f:
                        f.write(img_data_bot)
                    self.node.get_logger().info(f"[{self.name}] Auto-saved Bottom Camera photo to {filename_bot}")
                
                self.has_saved_photo = True
        except Exception as e:
            self.node.get_logger().error(f"[{self.name}] Failed to save photo: {str(e)}")

    def execute(self) -> Status:
        self.node.get_logger().info(f"[{self.name}] Taking Photo...{self.mission_type}", throttle_duration_sec=1.0)

        if(self.mission_type == MissionStatus.GREEN_BOX and self.greenbox != None):
            self.node.get_logger().info(f"[{self.name}] Publishing Green Box Photo...", throttle_duration_sec=1.0)
            self.greenbox_pub.publish(String(data=self.greenbox))
            self._save_photo_to_disk(self.greenbox, "GreenBox")
        elif (self.mission_type == MissionStatus.BLUE_BOX and self.bluebox != None):
            self.node.get_logger().info(f"[{self.name}] Publishing Blue Box Photo...", throttle_duration_sec=1.0)
            self.bluebox_pub.publish(String(data=self.bluebox))
            self._save_photo_to_disk(self.bluebox, "BlueBox")
        
        self.yaw_effort_pub.publish(Float64(data=0.0))
        self.speed_effort_pub.publish(Float64(data=0.0))

        self.current = True            
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