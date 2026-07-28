#!/usr/bin/env python3

from core.utils.config import Topic
from core_perception.base_camera_controller import BaseCameraNode


class BottomCameraNode(BaseCameraNode):
    """
    Child class for the Bottom (Down) Camera.
    Handles specific downward-facing computer vision and publishers.
    """

    def __init__(self):
        super().__init__("bottom_camera", "CNFHH52R10643003DBB0_Integrated_Webcam_HD")

        # Publishers specific to the bottom camera
        self.camera_down_pub = Topic.bottom_camera_processed.createPublisher(self)

    def process_and_publish(self, frame):
        # Insert bottom-specific image processing here (e.g. line tracking, box detection)
        # ...

        # Encode frame and publish to down camera topic
        encoded_msg = self.encode_base64(frame)
        if encoded_msg:
            self.camera_down_pub.publish(encoded_msg)
