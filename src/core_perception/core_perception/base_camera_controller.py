#!/usr/bin/env python3

import traceback
import cv2
import rclpy
import base64
from core.utils.device_fetching import get_webcam_device_idx
from rclpy.node import Node
from std_msgs.msg import String


class BaseCameraNode(Node):
    """
    Parent class handling common camera operations:
    capture setup, frame reading, encoding, and visualization.
    """

    def __init__(self, node_name, camera_identifier, fps=30, width=640, height=480):
        super().__init__(node_name)

        self.camera_idx = get_webcam_device_idx(camera_identifier)

        self.cap = cv2.VideoCapture(self.camera_idx)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.fps = fps
        self.show_result = False
        self.vid_writer = None

        self.timer = self.create_timer(1.0 / self.fps, self._capture_loop)

        self.get_logger().info(f"Initialized {node_name} at {fps} FPS")

    def encode_base64(self, img):
        """Encodes an OpenCV image to a base64 string for ROS transmission"""
        result, encoded_image = cv2.imencode(
            ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 20]
        )
        if result:
            base64_image = base64.b64encode(encoded_image).decode("utf-8")
            img_msg = String()
            img_msg.data = base64_image
            return img_msg
        else:
            self.get_logger().error("Failed to encode frame to JPG")
            return None

    def visualize(self, img, window_name="Output", scale=0.6):
        """Display annotated frame"""

        display_img = cv2.resize(
            img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
        )
        cv2.imshow(window_name, display_img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Exit requested by user (q pressed).")
            return True
        return False

    def _capture_loop(self):
        """Internal loop called by timer. Reads frame and delegates to child."""
        try:
            success, frame = self.cap.read()
            if not success:
                self.get_logger().warn("Failed to get frame", throttle_duration_sec=5.0)
                return

            self.process_and_publish(frame)

            if self.show_result:
                if self.visualize(frame, window_name=self.get_name()):
                    rclpy.shutdown()

        except Exception:
            self.get_logger().error(f"Error in capture loop: {traceback.format_exc()}")

    def process_and_publish(self, frame):
        """To be overridden by child classes"""
        raise NotImplementedError("Child classes must implement this method")

    def cleanup(self):
        """Release hardware resources"""
        self.cap.release()
        if self.vid_writer:
            self.vid_writer.release()
