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
    
    # def visualize_avoidance(self, frame, depth_norm):
    #     """
    #     Pick the farthest X position (max depth) as avoidance direction,
    #     lock Y to the middle of the frame.
    #     """
    #     h, w = depth_norm.shape
    #     vis = frame.copy()

    #     # Step 1: Find column (x) with maximum depth (farthest point)
    #     col_depth = depth_norm.mean(axis=0)  # average depth per column
    #     best_x = int(np.argmax(col_depth))   # column with farthest depth
    #     ref_y = h // 2                       # always middle row
    #     ref_point = (best_x, ref_y)

    #     # Step 2: Draw the reference point and arrow
    #     cv2.circle(vis, ref_point, 8, (0, 255, 0), -1)              # green ref point
    #     cv2.arrowedLine(vis, (w // 2, h - 20), ref_point, (255, 0, 0), 2)  # arrow from bottom center

    #     return vis
    
    def visualize_avoidance(self, frame, depth_norm):
        """
        Maritime obstacle avoidance for ASV (Autonomous Surface Vehicle):
        1. Focus on water surface level (ignore sky/horizon)
        2. Detect obstacles on water (boats, buoys, debris)
        3. Find safe navigation corridor on water surface
        """
        h, w = depth_norm.shape
        vis = frame.copy()
        
        # ASV Configuration - focus on water surface navigation
        water_level_start = int(h * 0.4)    # Start analyzing from 40% down (ignore sky)
        water_level_end = int(h * 0.85)     # Stop at 85% (ignore boat hull/deck)
        
        obstacle_threshold = 0.25           # Objects closer than this on water surface
        min_safe_corridor = 80              # Minimum safe passage width in pixels
        look_ahead_distance = 100           # How far ahead to check (pixels)
        
        # Step 1: Focus only on water surface area
        water_region = depth_norm[water_level_start:water_level_end, :]
        water_vis_region = vis[water_level_start:water_level_end, :]
        
        # Step 2: Detect obstacles on water surface
        # Find objects that are significantly closer than the average water depth
        water_median_depth = np.median(water_region)
        obstacle_mask = water_region < (water_median_depth - 0.15)  # Obstacles stick out from water
        
        # Step 3: Analyze horizontal navigation corridors
        safe_corridors = []
        
        # Check multiple rows in the water region for safe passages
        check_rows = range(0, water_region.shape[0], 15)  # Every 15 pixels
        
        for row_idx in check_rows:
            if row_idx >= water_region.shape[0]:
                continue
                
            row_obstacles = obstacle_mask[row_idx, :]
            
            # Find continuous safe segments
            safe_segments = self._find_water_safe_segments(row_obstacles, min_safe_corridor)
            
            for start_x, end_x in safe_segments:
                corridor_center = (start_x + end_x) // 2
                corridor_width = end_x - start_x
                actual_y = water_level_start + row_idx
                
                # Calculate safety score (prefer wider, more centered corridors)
                center_bias = 1.0 - abs(corridor_center - w//2) / (w//2)  # Prefer center
                width_score = min(corridor_width / 150.0, 1.0)  # Prefer wider corridors
                
                safe_corridors.append({
                    'center_x': corridor_center,
                    'y': actual_y,
                    'width': corridor_width,
                    'safety_score': (center_bias * 0.3 + width_score * 0.7),
                    'start_x': start_x,
                    'end_x': end_x,
                    'distance_ahead': actual_y - water_level_start
                })
        
        # Step 4: Visualize water surface area and obstacles
        # Draw water analysis region boundary
        cv2.rectangle(vis, (0, water_level_start), (w, water_level_end), (0, 255, 255), 2)
        cv2.putText(vis, "WATER SURFACE ANALYSIS", (10, water_level_start - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Highlight obstacles on water surface (red)
        obstacle_coords = np.where(obstacle_mask)
        for i in range(len(obstacle_coords[0])):
            y_abs = water_level_start + obstacle_coords[0][i]
            x_abs = obstacle_coords[1][i]
            cv2.circle(vis, (x_abs, y_abs), 3, (0, 0, 255), -1)
        
        # Step 5: Draw safe corridors
        for corridor in safe_corridors:
            x, y = corridor['center_x'], corridor['y']
            safety = corridor['safety_score']
            
            # Color based on safety score: Green (very safe) → Yellow (less safe)
            if safety > 0.7:
                color = (0, 255, 0)      # Green - excellent corridor
            elif safety > 0.4:
                color = (0, 255, 255)    # Yellow - good corridor
            else:
                color = (0, 165, 255)    # Orange - acceptable corridor
            
            # Draw corridor boundaries
            cv2.line(vis, (corridor['start_x'], y), (corridor['end_x'], y), color, 3)
            cv2.circle(vis, (x, y), 5, color, -1)
        
        # Step 6: Select best navigation target
        best_target = self._select_best_water_target(safe_corridors, w//2)
        
        if best_target:
            target_x, target_y = best_target['center_x'], best_target['y']
            
            # Draw the selected target
            cv2.circle(vis, (target_x, target_y), 15, (255, 255, 0), 3)
            cv2.circle(vis, (target_x, target_y), 8, (0, 255, 255), -1)
            
            # Draw navigation arrow from bottom center of water region
            start_arrow = (w // 2, water_level_end - 20)
            cv2.arrowedLine(vis, start_arrow, (target_x, target_y), (255, 255, 0), 6, tipLength=0.2)
            
            # Calculate steering direction
            center_x = w // 2
            steering_offset = target_x - center_x
            steering_angle = int(np.degrees(np.arctan2(steering_offset, water_level_end - target_y)))
            
            if abs(steering_offset) < 30:
                direction = "STRAIGHT"
                color = (0, 255, 0)
            elif steering_offset < 0:
                direction = "PORT (LEFT)"
                color = (0, 0, 255)
            else:
                direction = "STARBOARD (RIGHT)"
                color = (255, 0, 0)
            
            # Navigation display
            cv2.putText(vis, f"ASV Navigation: {direction}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(vis, f"Steering Angle: {steering_angle}°", (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(vis, f"Corridor Width: {best_target['width']}px", (10, 90), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            # Emergency situation - no safe corridor found
            cv2.putText(vis, "WARNING: NO SAFE CORRIDOR!", (w//4, h//2), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            cv2.putText(vis, "REDUCE SPEED / STOP", (w//4, h//2 + 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # Step 7: Add maritime navigation info
        self._draw_maritime_legend(vis, water_median_depth)
        
        return vis

    def _find_water_safe_segments(self, obstacle_row, min_width):
        """Find continuous safe segments on water surface."""
        segments = []
        in_safe_zone = True
        start_x = 0
        
        for x in range(len(obstacle_row)):
            if obstacle_row[x] and in_safe_zone:  # Hit obstacle, end safe zone
                if x - start_x >= min_width:
                    segments.append((start_x, x))
                in_safe_zone = False
            elif not obstacle_row[x] and not in_safe_zone:  # Clear water, start safe zone
                start_x = x
                in_safe_zone = True
        
        # Handle safe zone that goes to the end
        if in_safe_zone and len(obstacle_row) - start_x >= min_width:
            segments.append((start_x, len(obstacle_row)))
        
        return segments

    def _select_best_water_target(self, safe_corridors, center_x):
        """Select best navigation target on water surface."""
        if not safe_corridors:
            return None
        
        # Prefer corridors that are:
        # 1. Closer to the boat (higher y value in image)
        # 2. Have high safety score
        # 3. Are reasonably centered
        
        best_corridor = None
        best_score = -1
        
        for corridor in safe_corridors:
            # Distance factor (closer is better, but not too close)
            distance_factor = min(corridor['distance_ahead'] / 50.0, 1.0)
            if distance_factor < 0.2:  # Too close
                distance_factor = 0.1
            
            # Safety factor
            safety_factor = corridor['safety_score']
            
            # Total score
            total_score = distance_factor * 0.6 + safety_factor * 0.4
            
            if total_score > best_score:
                best_score = total_score
                best_corridor = corridor
        
        return best_corridor

    def _draw_maritime_legend(self, vis, water_depth):
        """Draw maritime-specific legend and info."""
        h, w = vis.shape[:2]
        
        # Legend background
        cv2.rectangle(vis, (w - 250, 10), (w - 10, 150), (0, 0, 0), -1)
        cv2.rectangle(vis, (w - 250, 10), (w - 10, 150), (255, 255, 255), 2)
        
        legend_items = [
            ("Red dots: Obstacles", (0, 0, 255)),
            ("Green line: Safe corridor", (0, 255, 0)),
            ("Yellow: Navigation target", (0, 255, 255)),
            ("Cyan box: Water analysis", (255, 255, 0))
        ]
        
        cv2.putText(vis, "ASV Navigation", (w - 240, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        for i, (text, color) in enumerate(legend_items):
            y_pos = 50 + i * 20
            cv2.circle(vis, (w - 235, y_pos), 5, color, -1)
            cv2.putText(vis, text, (w - 220, y_pos + 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Water depth info
        cv2.putText(vis, f"Water depth: {water_depth:.2f}", (w - 240, 140), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


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
