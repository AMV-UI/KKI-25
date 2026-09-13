
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
        self.time_threshold = getattr(MissionParams, 'finding_lost_timeout', 3.0)

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
        self.direction = 1
        self.approach_start_time = 0.0
        self.integral = 0.0
        self.prev_dsc = 0.0
        self.has_seen_box = False
        if self.frame_counter:
            self.frame_counter.reset()
        self.node.get_logger().info(f"[{self.name}] Initializing Finding Execution (Searching)")

    def execute(self) -> Status:
        # If box is detected, mark it as seen
        if getattr(self, 'detected', False):
            if not getattr(self, 'has_seen_box', False):
                self.node.get_logger().info(f"[{self.name}] Box terlihat pertama kali! Mengingat status penemuan...")
            self.has_seen_box = True
            if self.frame_counter:
                self.frame_counter.reset()

            if self.phase == "searching":
                self.node.get_logger().info(f"[{self.name}] Beralih ke fase ALIGNMENT...")
                self.phase = "aligning"
                return Status.RUNNING

        # State: SEARCHING
        if self.phase == "searching":
            self.node.get_logger().info(f"[{self.name}] Mencari sembarang box... (Spinning)", throttle_duration_sec=2.0)
            # Turn opposite of Turn_Next_Buoy (Turn_Next_Buoy A sends Negative, so we send Positive)
            yaw_val = float(self.effort) if self.arena == "A" else -float(self.effort)
            self.yaw_effort_pub.publish(Float64(data=yaw_val))
            self.speed_effort_pub.publish(Float64(data=0.0))
            return Status.RUNNING

        # State: ALIGNING
        elif self.phase == "aligning":
            if not getattr(self, 'detected', False):
                # Box lost
                if getattr(self, 'has_seen_box', False):
                    self.frame_counter.is_started()
                    if self.frame_counter.is_enough():
                        self.frame_counter.reset()
                        self.node.get_logger().info(f"[{self.name}] Box hilang selama {self.time_threshold}s -> Beralih ke fase REVERSING!")
                        self.phase = "reversing"
                        return Status.RUNNING
                    
                    self.node.get_logger().info(f"[{self.name}] Box hilang sementara, menunggu...", throttle_duration_sec=1.0)
                    # Keep moving forward to try to find it again, similar to buoy
                    self.yaw_effort_pub.publish(Float64(data=0.0))
                    self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))
                    return Status.RUNNING
                else:
                    self.node.get_logger().info(f"[{self.name}] Box hilang! Kembali ke SEARCHING...", throttle_duration_sec=1.0)
                    self.phase = "searching"
                    return Status.RUNNING

            # Code 8888.0 = Only Green found.
            if self.dsc == 8888.0:
                self.node.get_logger().info(f"[{self.name}] Hanya Hijau terlihat. Maju sambil berputar mencari Biru...", throttle_duration_sec=1.0)
                sign = getattr(MissionParams, 'turn_away_green_sign', 1.0)
                yaw_cmd = (sign * float(self.effort)) if self.arena == "A" else (-sign * float(self.effort))
                self.yaw_effort_pub.publish(Float64(data=yaw_cmd))
                self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))
                return Status.RUNNING
            
            # Code 7777.0 = Only Blue found.
            elif self.dsc == 7777.0:
                self.node.get_logger().info(f"[{self.name}] Hanya Biru terlihat. Maju sambil berputar mencari Hijau...", throttle_duration_sec=1.0)
                sign = getattr(MissionParams, 'turn_away_blue_sign', -1.0)
                yaw_cmd = (sign * float(self.effort)) if self.arena == "A" else (-sign * float(self.effort))
                self.yaw_effort_pub.publish(Float64(data=yaw_cmd))
                self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))
                return Status.RUNNING

            # Use PID to center the midpoint while moving forward
            # INSTEAD: User requested to immediately switch to PHOTO mission when both boxes are detected!
            self.node.get_logger().info(f"[{self.name}] Kedua box (Hijau & Biru) terdeteksi (DSC: {self.dsc:.2f})! Beralih ke misi PHOTO!")
            if self.mission is not None:
                from std_msgs.msg import UInt8
                self.mission_pub.publish(UInt8(data=self.mission))
            return Status.SUCCESS

class Finding_Fallback(BaseFallback):
    """
    """
    def __init__(self, name: str = "Mission2_Fallback", node=None):
        super().__init__(name, node=node)
    
    def fallback(self) -> Status:
        return Status.FAILURE
