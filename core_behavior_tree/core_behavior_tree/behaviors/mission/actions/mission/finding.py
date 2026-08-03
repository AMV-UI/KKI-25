
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
        self.dsc = self.effort * (-1 if self.arena == "A" else 1)  #Reverse effort untuk mission 2
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
        self.has_seen_box = False
        self.lost_time = 0.0
        self.integral = 0.0
        self.prev_dsc = 0.0
        if self.frame_counter:
            self.frame_counter.reset()
        self.node.get_logger().info(f"[{self.name}] Initializing Finding Execution")

    def execute(self) -> Status:
        if self.dsc != 9999.0:
            self.has_seen_box = True
            self.lost_time = 0.0
            
            # Box is visible in camera
            self.node.get_logger().info(f"[{self.name}] Box terlihat! Bergerak mendekat... (DSC: {self.dsc:.2f})", throttle_duration_sec=1.0)
            
            # Full PID Control for tracking and fighting currents
            kp = MissionParams.kp_cam
            ki = MissionParams.ki_cam
            kd = MissionParams.kd_cam
            
            # Accumulate integral
            self.integral += self.dsc
            
            # Anti-windup
            max_integral = 2000.0
            if self.integral > max_integral: self.integral = max_integral
            elif self.integral < -max_integral: self.integral = -max_integral
            
            # Calculate derivative
            derivative = self.dsc - self.prev_dsc
            self.prev_dsc = self.dsc
            
            yaw_cmd = (self.dsc * kp) + (self.integral * ki) + (derivative * kd)
            
            # Remove hard deadband so the integral can fight steady currents
            # But limit small noise
            if abs(self.dsc) < 5.0:
                self.integral *= 0.9 # Bleed off integral slightly when centered
                
            # Limit maximum steering
            if yaw_cmd > float(self.effort): yaw_cmd = float(self.effort)
            elif yaw_cmd < -float(self.effort): yaw_cmd = -float(self.effort)
            
            # Move forward and center the box
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))
            
            if self.detected:
                # Box is visible AND large enough (area > 10000)
                self.frame_counter.is_started()
                if self.frame_counter.is_enough():
                    self.frame_counter.reset()
                    self.node.get_logger().info(
                        f"[{self.name}] Box cukup besar! Memulai pengambilan foto..."
                    )
                    self.mission_pub.publish(UInt8(data=self.mission))
                    return Status.SUCCESS
            else:
                self.frame_counter.reset()
        else:
            # Box not visible
            self.frame_counter.reset()
            
            if getattr(self, 'has_seen_box', False):
                if not hasattr(self, 'lost_time') or self.lost_time == 0.0:
                    self.lost_time = self.node.get_clock().now().nanoseconds / 1e9
                
                # If lost for more than 2 seconds, assume completely lost and spin again
                if (self.node.get_clock().now().nanoseconds / 1e9) - self.lost_time > 2.0:
                    self.has_seen_box = False
                    self.lost_time = 0.0
                else:
                    self.node.get_logger().info(f"[{self.name}] Box hilang sejenak, melaju lurus...", throttle_duration_sec=1.0)
                    self.yaw_effort_pub.publish(Float64(data=0.0))
                    self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))
                    return Status.RUNNING

            # Not seen yet or lost for too long, spin to find it
            self.node.get_logger().info(f"[{self.name}] Mencari box...", throttle_duration_sec=2.0)
            # Reverse yaw effort to counter Turn_Next_Buoy overshoot
            yaw_val = float(self.effort * (1 if self.arena == "A" else -1))
            self.yaw_effort_pub.publish(Float64(data=yaw_val))
            
            # Zero speed effort during finding phase, just yaw
            self.speed_effort_pub.publish(Float64(data=0.0))
            
        return Status.RUNNING

class Finding_Fallback(BaseFallback):
    """
    """
    def __init__(self, name: str = "Mission2_Fallback", node=None):
        super().__init__(name, node=node)
    
    def fallback(self) -> Status:
        return Status.FAILURE
