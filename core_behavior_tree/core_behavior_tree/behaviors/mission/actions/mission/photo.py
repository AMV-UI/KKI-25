from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from core.mission.frame_counter import FrameCounter
from std_msgs.msg import String, UInt8, Float64
from core.utils.config import Topic, MissionStatus, MissionParams
import time

class Photo_Execution(BaseExecution):
    def __init__(self, name, node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node
        self.arena = "B"
        self.mission = mission
        
        self.greenbox = None
        self.bluebox = None
        
        self.dsc = 9999.0
        self.prev_dsc = 0.0
        self.integral = 0.0
        self.effort = MissionParams.finding_yaw_effort
        self.has_saved_photo = False
        self.time_threshold = getattr(MissionParams, 'photo_time_threshold', 2.0)
        self.frame_counter = FrameCounter(self.time_threshold)
        
    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.greenbox_pub = Topic.image_green_box.createPublisher(self.node)
        self.bluebox_pub = Topic.image_blue_box.createPublisher(self.node)
        self.mission_pub = Topic.mission.createPublisher(self.node)
        self.mission_type_pub = Topic.mission_type.createPublisher(self.node)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
        self.green_box_encoded_sub = Topic.green_box_encoded.createSubscriber(
            self.node, self._green_box_cb
        )
        self.blue_box_encoded_sub = Topic.blue_box_encoded.createSubscriber(
            self.node, self._blue_box_cb
        )
        self.dsc_sub = Topic.dsc.createSubscriber(
            self.node, self._dsc_cb
        )
        self.arena_sub = Topic.arena.createSubscriber(
            self.node, self._arena_cb
        )
        
    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)
        
    def _dsc_cb(self, msg: Float64):
        self.dsc = float(msg.data)
        
    def _green_box_cb(self, msg: String):
        self.greenbox = str(msg.data)
    
    def _blue_box_cb(self, msg: String):
        self.bluebox = str(msg.data)

    def initialise(self) -> None:
        super().initialise()
        self.phase = "setup_green"
        self.has_saved_photo = False
        self.target_mission = None
        self.integral = 0.0
        self.prev_dsc = 0.0
        if self.frame_counter:
            self.frame_counter.reset()
        self.sweep_start_time = time.time()
        self.sweep_direction = 1
        self.node.get_logger().info(f"[{self.name}] Memulai Misi Foto Ganda")

    def _save_photo_to_disk(self, b64_data, prefix):
        try:
            import base64
            import os
            from datetime import datetime
            
            save_dir = os.path.expanduser("~/KKI-25/core_perception/photos")
            if not os.path.exists(save_dir):
                os.makedirs(save_dir, exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            parts = b64_data.split("|||")
            top_b64 = parts[0]
            bot_b64 = parts[1] if len(parts) > 1 else None
            
            filename_top = os.path.join(save_dir, f"{timestamp}_{prefix}_TopCamera_Auto.jpg")
            img_data_top = base64.b64decode(top_b64)
            with open(filename_top, "wb") as f:
                f.write(img_data_top)
            self.node.get_logger().info(f"[{self.name}] Auto-saved Top Camera photo to {filename_top}")
            
            # Save Bottom Camera if available
            if bot_b64:
                filename_bot = os.path.join(save_dir, f"{timestamp}_{prefix}_BottomCamera_Auto.jpg")
                img_data_bot = base64.b64decode(bot_b64)
                with open(filename_bot, "wb") as f:
                    f.write(img_data_bot)
                self.node.get_logger().info(f"[{self.name}] Auto-saved Bottom Camera photo to {filename_bot}")
        except Exception as e:
            self.node.get_logger().error(f"[{self.name}] Failed to save photo: {str(e)}")

    def align_to_target(self, target_name):
        if self.dsc == 9999.0:
            if self.frame_counter: self.frame_counter.reset()
            # Default spinning direction (like finding box)
            # Arena A: Turn Right (Negative), Arena B: Turn Left (Positive)
            yaw_cmd = -float(self.effort) if self.arena == "A" else float(self.effort)
            
            self.yaw_effort_pub.publish(Float64(data=yaw_cmd))
            self.speed_effort_pub.publish(Float64(data=0.0))
            self.node.get_logger().info(f"[{self.name}] Memutar mencari Box {target_name} untuk difoto...", throttle_duration_sec=1.0)
            return False
            
        if self.dsc == 8888.0:
            if self.frame_counter: self.frame_counter.reset()
            # Only Green is seen.
            sign = getattr(MissionParams, 'turn_away_green_sign', 1.0)
            yaw_cmd = (sign * float(self.effort)) if self.arena == "A" else (-sign * float(self.effort))
            self.yaw_effort_pub.publish(Float64(data=yaw_cmd))
            self.speed_effort_pub.publish(Float64(data=0.0))
            self.node.get_logger().info(f"[{self.name}] Box Hijau terlihat! Memutar berlawanan mencari Box {target_name}...", throttle_duration_sec=1.0)
            return False
            
        if self.dsc == 7777.0:
            if self.frame_counter: self.frame_counter.reset()
            # Only Blue is seen.
            sign = getattr(MissionParams, 'turn_away_blue_sign', -1.0)
            yaw_cmd = (sign * float(self.effort)) if self.arena == "A" else (-sign * float(self.effort))
            self.yaw_effort_pub.publish(Float64(data=yaw_cmd))
            self.speed_effort_pub.publish(Float64(data=0.0))
            self.node.get_logger().info(f"[{self.name}] Box Biru terlihat! Memutar berlawanan mencari Box {target_name}...", throttle_duration_sec=1.0)
            return False
        # Target box is roughly in the center, start counting frames
        margin = getattr(MissionParams, 'photo_margin_error', 150.0)
        if abs(self.dsc) < margin:
            if self.frame_counter:
                self.frame_counter.is_started()
                if self.frame_counter.is_enough():
                    self.frame_counter.reset()
                    self.yaw_effort_pub.publish(Float64(data=0.0))
                    self.speed_effort_pub.publish(Float64(data=0.0))
                    return True
        else:
            if self.frame_counter:
                self.frame_counter.reset()
            
        kp = getattr(MissionParams, 'kp_cam', 0.2)
        ki = getattr(MissionParams, 'ki_cam', 0.01)
        kd = getattr(MissionParams, 'kd_cam', 0.4)
        
        self.integral += self.dsc
        max_int = 2000.0
        if self.integral > max_int: self.integral = max_int
        elif self.integral < -max_int: self.integral = -max_int
        
        derivative = self.dsc - self.prev_dsc
        self.prev_dsc = self.dsc
        
        raw_pid = (self.dsc * kp) + (self.integral * ki) + (derivative * kd)
        yaw_cmd = -raw_pid
        align_effort = float(self.effort) * 0.5
        if yaw_cmd > align_effort: yaw_cmd = align_effort
        elif yaw_cmd < -align_effort: yaw_cmd = -align_effort

        self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
        self.speed_effort_pub.publish(Float64(data=0.0))
        self.node.get_logger().info(f"[{self.name}] Menyelaraskan Box {target_name} (DSC: {self.dsc:.2f})", throttle_duration_sec=1.0)
        return False

    def execute(self) -> Status:
        if self.phase == "setup_green":
            self.mission_type_pub.publish(String(data=MissionStatus.GREEN_BOX))
            self.greenbox = None
            self.integral = 0.0
            self.phase = "align_green"
            return Status.RUNNING
            
        elif self.phase == "align_green":
            # Keep publishing to ensure it switches
            self.mission_type_pub.publish(String(data=MissionStatus.GREEN_BOX))
            if self.align_to_target("Green"):
                self.node.get_logger().info(f"[{self.name}] Box Hijau di tengah. Mengambil foto...")
                self.phase = "wait_green_photo"
            return Status.RUNNING
            
        elif self.phase == "wait_green_photo":
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.speed_effort_pub.publish(Float64(data=0.0))
            if self.greenbox != None:
                self.greenbox_pub.publish(String(data=self.greenbox))
                self._save_photo_to_disk(self.greenbox, "GreenBox")
                self.phase = "setup_blue"
            return Status.RUNNING
            
        elif self.phase == "setup_blue":
            self.mission_type_pub.publish(String(data=MissionStatus.BLUE_BOX))
            self.bluebox = None
            self.integral = 0.0
            self.dsc = 9999.0
            if self.frame_counter:
                self.frame_counter.reset()
            self.phase = "align_blue"
            return Status.RUNNING
            
        elif self.phase == "align_blue":
            self.mission_type_pub.publish(String(data=MissionStatus.BLUE_BOX))
            if self.align_to_target("Blue"):
                self.node.get_logger().info(f"[{self.name}] Box Biru di tengah. Mengambil foto...")
                self.phase = "wait_blue_photo"
            return Status.RUNNING
            
        elif self.phase == "wait_blue_photo":
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.speed_effort_pub.publish(Float64(data=0.0))
            if self.bluebox != None:
                self.bluebox_pub.publish(String(data=self.bluebox))
                self._save_photo_to_disk(self.bluebox, "BlueBox")
                self.phase = "done"
            return Status.RUNNING
            
        elif self.phase == "done":
            if self.mission is not None:
                self.mission_pub.publish(UInt8(data=self.mission))
            return Status.SUCCESS

        return Status.RUNNING


class Photo_Fallback(BaseFallback):
    def __init__(self, name, node=None):
        super().__init__(name, node=node)

    def fallback(self) -> Status:        
        return Status.FAILURE