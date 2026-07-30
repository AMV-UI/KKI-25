#!/usr/bin/env python3

from core_perception.base_camera_controller import BaseCameraNode


class BottomCameraNode(BaseCameraNode):
    """
    Child class for the Bottom (Down) Camera.
    Handles specific downward-facing computer vision.
    """

    def __init__(self):
        super().__init__(
            node_name="bottom_camera",
            camera_identifier="Generic_HD_camera_20201212000000",
            stream_url="rtsp://localhost:8554/live/bottomcam",
        )

    def process_and_publish(self, frame):
        # Insert bottom-specific image processing here (e.g. line tracking, box detection)
        # Anything drawn onto `frame` here will automatically be streamed.
        pass
