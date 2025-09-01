import os
import cv2
import torch
import numpy as np
import matplotlib.cm as cm
from datetime import datetime

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from core.perception.depth.depth_anything_v2.dpt import DepthAnythingV2
from core.utils.config import NodeConfig, Topic


class DepthController(Node):
    """
    DepthAnythingV2 ROS2 Node
    - Captures frames from webcam
    - Runs DepthAnythingV2 inference
    - Publishes / displays depth map
    """

    def __init__(self):
        super().__init__(NodeConfig.depth_controller)

        self.bridge = CvBridge()
        self.device = (
            "cuda" if torch.cuda.is_available()
            else "mps" if torch.backends.mps.is_available()
            else "cpu"
        )

        self.encoder = "vits"
        self.model_path = "/home/amv/core/core_perception/core_perception/models/depth_anything_v2_vits.pth"
        self.video_path = "/home/amv/Downloads/sim2.mp4"
        self.input_size = 518
        self.outdir = "./vis_webcam_depth"
        self.pred_only = False
        self.grayscale = False
        self.record = False
        self.skip = 10

        # Model configs
        self.model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
            'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }

        # Load everything
        self.__load_model__()
        # self.__load_camera__() webcam
        self.__load_camera__(self.video_path)
        self.__setup_writer__()

        self.cmap = cm.get_cmap("Spectral_r")
        self.margin_width = 50

        self.get_logger().info("DepthAnythingV2 Node initialized with encoder=vits.")

    def __load_model__(self):
        self.model = DepthAnythingV2(**self.model_configs[self.encoder])
        self.model.load_state_dict(torch.load(self.model_path, map_location="cuda"))
        self.model = self.model.to(self.device).eval()
        self.get_logger().info(f"Model loaded: {self.encoder} → {self.model_path}")

    def __load_camera__(self, video_path: str = None):
        """
        Load camera or video file.
        - If video_path is None → webcam (device 0).
        - If video_path is given → load from file.
        """
        if video_path:
            self.cap = cv2.VideoCapture(video_path)
            self.get_logger().info(f"Loading video file: {video_path}")
        else:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 500)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)
            self.get_logger().info("Loading webcam (device 0).")

        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open {'video' if video_path else 'webcam'}")

        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"Source resolution: {self.frame_width}x{self.frame_height}")


    def __setup_writer__(self):
        if self.record:
            os.makedirs(self.outdir, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            out_path = os.path.join(self.outdir, f"depth_webcam_{timestamp}.mp4")
            output_width = self.frame_width if self.pred_only else self.frame_width * 2 + self.margin_width

            self.writer = cv2.VideoWriter(
                out_path,
                cv2.VideoWriter_fourcc(*"mp4v"),
                60,
                (output_width, self.frame_height),
            )
            self.get_logger().info(f"Recording output → {out_path}")
        else:
            self.writer = None

    def process_frame(self):

        for _ in range(self.skip):
            self.cap.grab()

        ret, frame = self.cap.read()
        if not ret:
            return None

        with torch.no_grad():
            depth = self.model.infer_image(frame, self.input_size)

        depth = (depth - depth.min()) / (depth.max() - depth.min())
        depth_norm = depth.astype(np.float32)

        # object avoidance

        vis_frame = self.visualize_avoidance(frame, depth_norm)


        if self.grayscale:
            depth_uint8 = (depth_norm * 255).astype(np.uint8)
            depth_rgb = np.repeat(depth_uint8[..., np.newaxis], 3, axis=-1)
        else:
            depth_rgb = (self.cmap(depth_norm)[:, :, :3] * 255).astype(np.uint8)
            depth_rgb = depth_rgb[:, :, ::-1]  # RGB → BGR

        if self.pred_only:
            output = depth_rgb
        else:
            spacer = np.ones((self.frame_height, self.margin_width, 3), dtype=np.uint8) * 255
            output = cv2.hconcat([frame, spacer, depth_rgb, spacer, vis_frame])

        return output
    
    def visualize_avoidance(self, frame, depth_norm):
        """
        Pick the farthest X position (max depth) as avoidance direction,
        lock Y to the middle of the frame.
        """
        h, w = depth_norm.shape
        vis = frame.copy()

        # Step 1: Find column (x) with maximum depth (farthest point)
        col_depth = depth_norm.mean(axis=0)  # average depth per column
        best_x = int(np.argmax(col_depth))   # column with farthest depth
        ref_y = h // 2                       # always middle row
        ref_point = (best_x, ref_y)

        # Step 2: Draw the reference point and arrow
        cv2.circle(vis, ref_point, 8, (0, 255, 0), -1)              # green ref point
        cv2.arrowedLine(vis, (w // 2, h - 20), ref_point, (255, 0, 0), 2)  # arrow from bottom center

        return vis




    def run(self):
        self.get_logger().info("Starting webcam feed. Press 'q' to quit.")
        while rclpy.ok():
            output = self.process_frame()
            if output is None:
                break

            cv2.imshow("DepthAnythingV2 - Webcam", output)

            if self.writer:
                self.writer.write(output)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.cleanup()

    def cleanup(self):
        self.cap.release()
        if self.writer:
            self.writer.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Resources released, shutting down.")


def main():
    rclpy.init()
    node = DepthController()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
