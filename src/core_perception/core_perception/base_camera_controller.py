#!/usr/bin/env python3

import traceback
import cv2
import rclpy
import subprocess
from core.utils.device_fetching import get_webcam_device_idx
from rclpy.node import Node


class BaseCameraNode(Node):
    """
    Parent class handling common camera operations:
    capture setup, frame reading, processing delegation, and direct FFmpeg streaming.
    """

    def __init__(
        self, node_name, camera_identifier, stream_url, fps=30, width=640, height=480
    ):
        super().__init__(node_name)

        self.camera_idx = get_webcam_device_idx(camera_identifier)
        self.stream_url = stream_url

        self.cap = cv2.VideoCapture(self.camera_idx)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.fps = fps
        self.show_result = False
        self.vid_writer = None

        # --- Initialize FFmpeg Pipeline ---
        ffmpeg_cmd = [
            "ffmpeg",
            "-y",
            "-thread_queue_size",
            "512",
            "-f",
            "rawvideo",
            "-vcodec",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            f"{width}x{height}",
            "-r",
            str(self.fps),
            "-i",
            "-",
            "-c:v",
            "libx264",
            "-preset",
            "superfast",
            "-tune",
            "zerolatency",
            "-crf",
            "18",
            "-b:v",
            "2.5M",
            "-pix_fmt",
            "yuv420p",
            "-r",
            str(self.fps),
            "-g",
            "15",
            "-forced-idr",
            "1",
            "-fflags",
            "+genpts+igndts",
            "-max_muxing_queue_size",
            "1024",
            "-rtsp_transport",
            "udp",
            "-f",
            "rtsp",
            self.stream_url,
        ]

        # Use bufsize=10**8 to prevent pipe blocking delays
        self.ffmpeg_process = subprocess.Popen(
            ffmpeg_cmd, stdin=subprocess.PIPE, bufsize=10**8
        )

        self.timer = self.create_timer(1.0 / self.fps, self._capture_loop)
        self.get_logger().info(
            f"Initialized {node_name} streaming to {stream_url} at {fps} FPS"
        )

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
        """Internal loop called by timer. Reads frame, delegates to child, and streams."""
        try:
            success, frame = self.cap.read()
            if not success:
                self.get_logger().warn("Failed to get frame", throttle_duration_sec=5.0)
                return

            # 1. Let the child node (Front/Bottom) process the frame in-place
            self.process_and_publish(frame)

            # 2. Stream the processed frame directly to FFmpeg
            if self.ffmpeg_process.poll() is None:
                self.ffmpeg_process.stdin.write(frame.tobytes())
                self.ffmpeg_process.stdin.flush()
            else:
                self.get_logger().error(
                    "FFmpeg process has died!", throttle_duration_sec=5.0
                )

            if self.show_result:
                if self.visualize(frame, window_name=self.get_name()):
                    rclpy.shutdown()

        except BrokenPipeError:
            self.get_logger().error(
                "FFmpeg pipe broken! Shutting down.", throttle_duration_sec=5.0
            )
        except Exception:
            self.get_logger().error(f"Error in capture loop: {traceback.format_exc()}")

    def process_and_publish(self, frame):
        """To be overridden by child classes"""
        raise NotImplementedError("Child classes must implement this method")

    def cleanup(self):
        """Release hardware resources and close FFmpeg"""
        self.cap.release()
        if self.vid_writer:
            self.vid_writer.release()
        if self.ffmpeg_process:
            self.ffmpeg_process.stdin.close()
            self.ffmpeg_process.wait()
