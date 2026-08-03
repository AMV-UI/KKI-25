
from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64, UInt8, String
from core.mission.find_mode import FindMode
from core.mission.frame_counter import FrameCounter
from core_msgs.msg import Pixhawk
from core.utils.config import Topic, MissionParams

import time


class Finding_Execution(BaseExecution):
    """
    Combined behavior: FIND_STEP_TWO -> STEP_TWO
    - Phase "finding": search with find_mode range(1), wait until detected for N frames
    - Phase "approach": navigate to target, when lost for N frames -> SUCCESS (ready for docking)
    """
    def __init__(self, name, node=None, mission = None):
        super().__init__(name, node=node)
        self.node = node
        self.find_mode = None
        self.frame_counter = None
        self.detected = False
        self.arena = "B"
        self.effort = MissionParams.finding_yaw_effort
        self.dsc = 9999.0
        self.mission = mission
        self.time_threshold = MissionParams.finding_time_threshold

        self.px_heading = 0.0
        self.phase = "finding"        

        self.speed_effort = MissionParams.finding_speed_effort

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.find_mode = FindMode(self.node, self.arena)
        self.frame_counter = FrameCounter(self.time_threshold)

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
        self.mission_pub = Topic.mission.createPublisher(self.node)

        self.arena_sub = Topic.arena.createSubscriber(self.node, self._arena_cb)
        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.dsc_sub = Topic.dsc.createSubscriber(self.node, self._dsc_cb)
        self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)

    def _detected_cb(self, msg: Bool):
        self.detected = bool(msg.data)

    def _dsc_cb(self, msg: Float64):
        self.dsc = float(msg.data)

    def _heading_cb(self, msg: Float64):
        self.px_heading = float(msg.data)
    def initialise(self) -> None:
        self.phase = "searching" # "searching", "aligning", "approaching"
        self.approach_start_time = 0.0
        self.integral = 0.0
        self.prev_dsc = 0.0
        self.node.get_logger().info(f"[{self.name}] Initializing Finding Execution")

    def execute(self) -> Status:
        # State: SEARCHING
        if self.phase == "searching":
            if getattr(self, 'dsc', 9999.0) != 9999.0:
                self.node.get_logger().info(f"[{self.name}] Box terlihat! Beralih ke fase ALIGNMENT...")
                self.phase = "aligning"
                return Status.RUNNING

            self.node.get_logger().info(f"[{self.name}] Mencari box... (Spinning)", throttle_duration_sec=2.0)
            
            # Spin opposite to turn_next_buoy
            yaw_val = -float(self.effort) if self.arena == "A" else float(self.effort)
            
            self.yaw_effort_pub.publish(Float64(data=yaw_val))
            self.speed_effort_pub.publish(Float64(data=0.0))
            return Status.RUNNING

        # State: ALIGNING
        elif self.phase == "aligning":
            if getattr(self, 'dsc', 9999.0) == 9999.0:
                self.node.get_logger().info(f"[{self.name}] Box hilang saat alignment! Kembali ke SEARCHING...")
                self.phase = "searching"
                return Status.RUNNING

            if abs(self.dsc) < 20.0:
                self.node.get_logger().info(f"[{self.name}] Box berada di tengah! Beralih ke fase APPROACHING...")
                self.phase = "approaching"
                self.approach_start_time = time.time()
                self.integral = 0.0
                self.prev_dsc = 0.0
                return Status.RUNNING

            # Use PID to center the box, but DO NOT move forward
            kp = getattr(MissionParams, 'kp_cam', 0.5)
            ki = getattr(MissionParams, 'ki_cam', 0.02)
            kd = getattr(MissionParams, 'kd_cam', 0.2)
            
            self.integral += self.dsc
            max_int = 2000.0
            if self.integral > max_int: self.integral = max_int
            elif self.integral < -max_int: self.integral = -max_int
            
            derivative = self.dsc - self.prev_dsc
            self.prev_dsc = self.dsc
            
            yaw_cmd = (self.dsc * kp) + (self.integral * ki) + (derivative * kd)
            
            align_effort = float(self.effort) * 0.5
            if yaw_cmd > align_effort: yaw_cmd = align_effort
            elif yaw_cmd < -align_effort: yaw_cmd = -align_effort

            self.node.get_logger().info(f"[{self.name}] Menyelaraskan box (DSC: {self.dsc:.2f})", throttle_duration_sec=1.0)
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            self.speed_effort_pub.publish(Float64(data=0.0))
            return Status.RUNNING

        # State: APPROACHING
        elif self.phase == "approaching":
            elapsed = time.time() - self.approach_start_time
            if elapsed >= getattr(MissionParams, 'finding_approach_duration', 5.0):
                self.node.get_logger().info(f"[{self.name}] Selesai mendekati box selama {elapsed:.1f} detik. Mission COMPLETE.")
                if self.mission is not None:
                    from std_msgs.msg import UInt8
                    self.mission_pub.publish(UInt8(data=self.mission))
                return Status.SUCCESS

            if getattr(self, 'dsc', 9999.0) == 9999.0:
                self.node.get_logger().info(f"[{self.name}] Box hilang saat approach! Menunggu/melaju lurus...", throttle_duration_sec=1.0)
                # Keep moving forward but stop steering
                self.yaw_effort_pub.publish(Float64(data=0.0))
                self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))
                return Status.RUNNING

            # Track using PID while moving forward
            kp = getattr(MissionParams, 'kp_cam', 0.5)
            ki = getattr(MissionParams, 'ki_cam', 0.02)
            kd = getattr(MissionParams, 'kd_cam', 0.2)
            
            self.integral += self.dsc
            if abs(self.dsc) < 5.0:
                self.integral *= 0.9
                
            max_int = 2000.0
            if self.integral > max_int: self.integral = max_int
            elif self.integral < -max_int: self.integral = -max_int
            
            derivative = self.dsc - self.prev_dsc
            self.prev_dsc = self.dsc
            
            yaw_cmd = (self.dsc * kp) + (self.integral * ki) + (derivative * kd)
            
            if yaw_cmd > float(self.effort): yaw_cmd = float(self.effort)
            elif yaw_cmd < -float(self.effort): yaw_cmd = -float(self.effort)

            self.node.get_logger().info(f"[{self.name}] Mendekati box (Maju) - Waktu tersisa: {getattr(MissionParams, 'finding_approach_duration', 5.0) - elapsed:.1f}s", throttle_duration_sec=1.0)
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))
            return Status.RUNNING

class Finding_Fallback(BaseFallback):
    """
    """
    def __init__(self, name: str = "Mission2_Fallback", node=None):
        super().__init__(name, node=node)
    
    def fallback(self) -> Status:
        return Status.FAILURE
