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
        Enhanced obstacle avoidance visualization with multiple strategies:
        1. Obstacle detection and safety zones
        2. Path planning with clearance analysis
        3. Multi-level threat assessment
        4. Dynamic safe corridor identification
        """
        h, w = depth_norm.shape
        vis = frame.copy()
        
        # Configuration parameters
        obstacle_threshold = 0.3  # Objects closer than this are obstacles
        safety_margin = 0.15      # Additional safety buffer
        corridor_width = 60       # Minimum safe corridor width in pixels
        look_ahead_rows = int(h * 0.7)  # How far ahead to analyze (70% of frame)
        
        # Step 1: Create obstacle mask
        obstacle_mask = depth_norm < obstacle_threshold
        danger_mask = depth_norm < (obstacle_threshold + safety_margin)
        
        # Step 2: Analyze horizontal corridors at different depths
        safe_corridors = []
        threat_levels = []
        
        # Analyze from middle to top of frame (looking ahead)
        analysis_rows = range(h//3, h//3 + look_ahead_rows//2, 10)
        
        for y in analysis_rows:
            if y >= h:
                continue
                
            row_obstacles = obstacle_mask[y, :]
            row_dangers = danger_mask[y, :]
            
            # Find continuous safe segments
            safe_segments = self._find_safe_segments(row_obstacles, corridor_width)
            danger_segments = self._find_safe_segments(~row_dangers, corridor_width//2)
            
            for start_x, end_x in safe_segments:
                corridor_center = (start_x + end_x) // 2
                corridor_width_actual = end_x - start_x
                
                # Calculate threat level based on surrounding dangers
                threat = self._calculate_threat_level(depth_norm, corridor_center, y, danger_mask)
                
                safe_corridors.append({
                    'center_x': corridor_center,
                    'y': y,
                    'width': corridor_width_actual,
                    'threat': threat,
                    'start_x': start_x,
                    'end_x': end_x
                })
        
        # Step 3: Visualize obstacles and danger zones
        # Red overlay for obstacles
        vis[obstacle_mask] = vis[obstacle_mask] * 0.3 + np.array([0, 0, 255]) * 0.7
        
        # Yellow overlay for danger zones
        danger_only = danger_mask & ~obstacle_mask
        vis[danger_only] = vis[danger_only] * 0.6 + np.array([0, 255, 255]) * 0.4
        
        # Step 4: Draw safe corridors with threat-based coloring
        for corridor in safe_corridors:
            x, y = corridor['center_x'], corridor['y']
            width = corridor['width']
            threat = corridor['threat']
            
            # Color based on threat level: Green (safe) → Yellow → Red (dangerous)
            if threat < 0.3:
                color = (0, 255, 0)      # Green - very safe
            elif threat < 0.6:
                color = (0, 255, 255)    # Yellow - moderate risk
            else:
                color = (0, 165, 255)    # Orange - higher risk
            
            # Draw corridor boundaries
            cv2.line(vis, (corridor['start_x'], y), (corridor['end_x'], y), color, 2)
            
            # Draw center point
            cv2.circle(vis, (x, y), 3, color, -1)
        
        # Step 5: Select optimal path
        best_path = self._select_optimal_path(safe_corridors, w//2, h//2)
        
        if best_path:
            # Draw the selected path
            path_points = [(corridor['center_x'], corridor['y']) for corridor in best_path]
            
            # Draw path line
            for i in range(len(path_points) - 1):
                cv2.line(vis, path_points[i], path_points[i+1], (255, 255, 0), 3)
            
            # Draw final target
            if path_points:
                target = path_points[0]  # Closest safe point
                cv2.circle(vis, target, 12, (255, 255, 0), 3)
                cv2.circle(vis, target, 6, (0, 255, 255), -1)
                
                # Draw navigation arrow from bottom center
                start_arrow = (w // 2, h - 30)
                cv2.arrowedLine(vis, start_arrow, target, (255, 255, 0), 4, tipLength=0.3)
                
                # Add text overlay with navigation info
                nav_text = f"Target: ({target[0]}, {target[1]})"
                cv2.putText(vis, nav_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Calculate and display turn direction
                center_x = w // 2
                turn_direction = "STRAIGHT" if abs(target[0] - center_x) < 20 else ("LEFT" if target[0] < center_x else "RIGHT")
                turn_angle = int(np.degrees(np.arctan2(target[0] - center_x, h - target[1])))
                
                cv2.putText(vis, f"Direction: {turn_direction} ({turn_angle}°)", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Step 6: Draw distance grid and safety zones
        self._draw_distance_grid(vis, depth_norm)
        
        # Step 7: Add legend
        self._draw_legend(vis)
        
        return vis

    def _find_safe_segments(self, binary_mask, min_width):
        """Find continuous safe segments in a binary mask."""
        segments = []
        in_segment = False
        start_x = 0
        
        for x in range(len(binary_mask)):
            if not binary_mask[x] and not in_segment:  # Start of safe segment
                start_x = x
                in_segment = True
            elif binary_mask[x] and in_segment:  # End of safe segment
                if x - start_x >= min_width:
                    segments.append((start_x, x))
                in_segment = False
        
        # Handle segment that goes to the end
        if in_segment and len(binary_mask) - start_x >= min_width:
            segments.append((start_x, len(binary_mask)))
        
        return segments

    def _calculate_threat_level(self, depth_norm, x, y, danger_mask):
        """Calculate threat level around a point."""
        h, w = depth_norm.shape
        
        # Sample area around the point
        sample_size = 20
        x1 = max(0, x - sample_size)
        x2 = min(w, x + sample_size)
        y1 = max(0, y - sample_size)
        y2 = min(h, y + sample_size)
        
        area_danger = danger_mask[y1:y2, x1:x2]
        area_depth = depth_norm[y1:y2, x1:x2]
        
        # Threat based on nearby dangers and average depth
        danger_ratio = np.sum(area_danger) / area_danger.size
        avg_depth = np.mean(area_depth)
        
        # Combine factors: more danger = higher threat, less depth = higher threat
        threat = danger_ratio + (1 - avg_depth) * 0.5
        
        return np.clip(threat, 0, 1)

    def _select_optimal_path(self, safe_corridors, center_x, center_y):
        """Select optimal path through safe corridors."""
        if not safe_corridors:
            return []
        
        # Group corridors by depth (y-coordinate)
        corridors_by_depth = {}
        for corridor in safe_corridors:
            y = corridor['y']
            if y not in corridors_by_depth:
                corridors_by_depth[y] = []
            corridors_by_depth[y].append(corridor)
        
        # Select best corridor at each depth level
        path = []
        last_x = center_x
        
        for y in sorted(corridors_by_depth.keys()):
            corridors_at_depth = corridors_by_depth[y]
            
            # Score corridors based on: width, low threat, proximity to last position
            best_corridor = None
            best_score = -float('inf')
            
            for corridor in corridors_at_depth:
                # Scoring factors
                width_score = corridor['width'] / 100.0  # Normalize width
                threat_score = 1 - corridor['threat']     # Lower threat = better
                proximity_score = 1 / (1 + abs(corridor['center_x'] - last_x) / 50.0)  # Closer = better
                
                total_score = width_score * 0.4 + threat_score * 0.4 + proximity_score * 0.2
                
                if total_score > best_score:
                    best_score = total_score
                    best_corridor = corridor
            
            if best_corridor:
                path.append(best_corridor)
                last_x = best_corridor['center_x']
        
        return path[:5]  # Limit path length

    def _draw_distance_grid(self, vis, depth_norm):
        """Draw distance grid overlay."""
        h, w = depth_norm.shape
        
        # Draw horizontal lines for distance zones
        for i, distance in enumerate([0.2, 0.4, 0.6, 0.8]):
            y_pos = int(h * (1 - distance))
            color = (100, 100, 100)
            cv2.line(vis, (0, y_pos), (w, y_pos), color, 1)
            cv2.putText(vis, f"{distance:.1f}", (w - 40, y_pos - 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    def _draw_legend(self, vis):
        """Draw legend for the visualization."""
        legend_items = [
            ("Red: Obstacles", (0, 0, 255)),
            ("Yellow: Danger Zone", (0, 255, 255)),
            ("Green: Safe Path", (0, 255, 0)),
            ("Cyan: Target", (255, 255, 0))
        ]
        
        y_offset = 90
        for i, (text, color) in enumerate(legend_items):
            y_pos = y_offset + i * 25
            cv2.rectangle(vis, (10, y_pos - 10), (30, y_pos + 5), color, -1)
            cv2.putText(vis, text, (35, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)




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
