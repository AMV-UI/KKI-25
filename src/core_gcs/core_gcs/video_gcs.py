import rclpy
from rclpy.node import Node
import cv2
import subprocess
import threading


class CameraStreamerNode(Node):
    def __init__(self):
        super().__init__("camera_mediamtx_streamer")

        # NOTE later this must subscribe to the image topic stream from core perception
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open USB Camera!")
            return

        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        if fps == 0:
            fps = 30

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
            f"{width}x{height}",
            "-r",
            str(fps),  # input framerate
            "-i",
            "-",
            "-c:v",
            "libx264",
            "-preset",
            "ultrafast",
            "-tune",
            "zerolatency",
            "-r",
            str(fps),  # output framerate (matches input)
            "-g",
            str(fps),
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
            f"Streaming {width}x{height} @ {fps}fps to {self.stream_url}"
        )

        self.is_running = True
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

    def capture_loop(self):
        while self.is_running and rclpy.ok():
            if self.ffmpeg_process.poll() is not None:
                self.get_logger().error(
                    "FFmpeg process crashed or exited! Stopping camera feed."
                )
                rclpy.shutdown()
                break

            ret, frame = self.cap.read()

            if ret:
                try:
                    self.ffmpeg_process.stdin.write(frame.tobytes())
                except Exception as e:
                    self.get_logger().error(f"FFmpeg pipe broken: {e}")
                    rclpy.shutdown()
                    break

    def destroy_node(self):
        self.get_logger().info("Shutting down camera and stream...")
        self.is_running = False

        if hasattr(self, "capture_thread"):
            self.capture_thread.join(timeout=1.0)

        self.cap.release()
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
