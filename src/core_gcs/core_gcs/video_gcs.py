import rclpy
from rclpy.node import Node
from core.utils.config import Topic
import cv2
import subprocess
import base64
import numpy as np


class CameraStreamerNode(Node):
    def __init__(self):
        super().__init__("camera_mediamtx_streamer")

        self.width = 640
        self.height = 480
        self.fps = 30

        self.stream_url = "rtsp://localhost:8554/live/usbcam"

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
            f"{self.width}x{self.height}",
            "-r",
            str(self.fps),  # input framerate
            "-i",
            "-",
            "-c:v",
            "libx264",
            "-preset",
            "ultrafast",
            "-tune",
            "zerolatency",
            "-r",
            str(self.fps),  # output framerate (matches input)
            "-g",
            str(self.fps),
            "-forced-idr",
            "1",
            "-fflags",
            "+genpts+igndts",  # regenerate PTS, ignore DTS
            "-max_muxing_queue_size",
            "1024",
            "-rtsp_transport",
            "tcp",
            "-f",
            "rtsp",
            self.stream_url,
        ]

        self.ffmpeg_process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)
        self.get_logger().info(
            f"Streaming {self.width}x{self.height} @ {self.fps}fps to {self.stream_url}"
        )

        # Subscribers
        self.front_cam_sub = Topic.front_camera_processed.createSubscriber(
            self, self.front_cam_cb
        )
        self.bottom_cam_sub = Topic.bottom_camera_processed.createSubscriber(
            self, self.bottom_cam_cb
        )

    def front_cam_cb(self, msg):
        if self.ffmpeg_process.poll() is not None:
            self.get_logger().error("FFmpeg process crashed! Stopping stream.")
            rclpy.shutdown()
            return

        try:
            jpg_bytes = base64.b64decode(msg.data)

            jpg_array = np.frombuffer(jpg_bytes, dtype=np.uint8)

            frame = cv2.imdecode(jpg_array, flags=cv2.IMREAD_COLOR)

            if frame is not None:
                self.ffmpeg_process.stdin.write(frame.tobytes())
            else:
                self.get_logger().warn("Failed to decode frame from topic data.")

        except BrokenPipeError:
            self.get_logger().error("FFmpeg pipe broken! Shutting down.")
            rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def bottom_cam_cb(self, msg):
        pass

    def destroy_node(self):
        self.get_logger().info("Shutting down camera streamer node...")

        if self.ffmpeg_process:
            self.ffmpeg_process.stdin.close()
            self.ffmpeg_process.wait()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
