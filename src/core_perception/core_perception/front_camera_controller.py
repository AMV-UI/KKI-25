#!/usr/bin/env python3

import cv2
from core.utils.config import Topic
from core_perception.base_camera_controller import BaseCameraNode
from std_msgs.msg import String


class FrontCameraNode(BaseCameraNode):
    """
    Child class for the Front (Up) Camera.
    Handles QR Code detection and specific front-facing publishers.
    """

    def __init__(self):
        super().__init__("front_camera", "CNFHH52R10643003DBB0_Integrated_Webcam_HD")

        self.camera_pub = Topic.front_camera_processed.createPublisher(self)
        self.qr_data_pub = Topic.qr_side.createPublisher(self)

        self.qr_detector = cv2.QRCodeDetector()

    def process_and_publish(self, frame):
        data, bbox, _ = self.qr_detector.detectAndDecode(frame)

        if bbox is not None and data:
            self.get_logger().info(
                f"QR Code Detected: {data}", throttle_duration_sec=2.0
            )

            qr_msg = String()
            qr_msg.data = data
            self.qr_data_pub.publish(qr_msg)

            bbox = bbox[0].astype(int)
            for i in range(len(bbox)):
                cv2.line(
                    frame,
                    tuple(bbox[i]),
                    tuple(bbox[(i + 1) % len(bbox)]),
                    (0, 255, 0),
                    3,
                )

        encoded_msg = self.encode_base64(frame)
        if encoded_msg:
            self.camera_pub.publish(encoded_msg)
