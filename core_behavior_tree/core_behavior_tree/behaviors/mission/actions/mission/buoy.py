from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64, UInt8, String
from core.utils.config import Topic
from core.mission.frame_counter import FrameCounter
from core_msgs.msg import Pixhawk
from core.utils.config import MissionParams


import time
class Buoy_Execution(BaseExecution):
    """
    Main execution: APPROACH to 2 Buoys (Different color, Green and Red) phase only
    - Navigate toward detected target using DSC from vision
    - When target is lost for N frames -> SUCCESS (mission complete)
    - If target not detected -> FAILURE (triggers fallback to search)
    - Store Pixhawk coordinate to docking station after to use in the last mission
    """
    def __init__(self, name: str = "Mission1_Execution", node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node
        self.frame_counter = None
        self.detected = True
        self.dsc = 0.0
        self.speed_effort = MissionParams.buoy_speed_effort
        self.arena = "B"
        self.time_threshold = MissionParams.buoy_time_threshold
        self.mission = mission
        self.has_seen_buoy = False
        
    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.frame_counter = FrameCounter(self.time_threshold)

        self.mission_pub = Topic.mission.createPublisher(self.node)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
        self.arena_sub = Topic.arena.createSubscriber(
            self.node, 
            self._arena_cb
        )
        self.detected_sub = Topic.detected.createSubscriber(
            self.node, 
            self._detected_cb
        )
        self.dsc_sub = Topic.dsc.createSubscriber(
            self.node, 
            self._dsc_cb
        )

    def initialise(self) -> None:
        self.has_seen_buoy = False
        self.integral = 0.0
        self.prev_dsc = 0.0
        if self.frame_counter:
            self.frame_counter.reset()
        self.node.get_logger().info(f"[{self.name}] Initializing Buoy Execution")

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)

    def _detected_cb(self, msg: Bool):
        self.detected = bool(msg.data)

    def _dsc_cb(self, msg: Float64):
        self.dsc = float(msg.data)

    def execute(self) -> Status:
        self.node.get_logger().info(f"[{self.name}] track {self.arena}", throttle_duration_sec=1.0)

        if not self.detected:
            if self.has_seen_buoy:
                self.frame_counter.is_started() 
                if self.frame_counter.is_enough():
                    self.frame_counter.reset()
                    self.node.get_logger().info(f"[{self.name}] Target lost for {self.time_threshold}s -> Mission COMPLETE")
                    self.mission_pub.publish(UInt8(data=self.mission))
                    return Status.SUCCESS

                self.node.get_logger().info(f"[{self.name}] Target lost briefly, waiting...", throttle_duration_sec=2.0)
                # Keep moving forward slowly while temporarily lost
                self.yaw_effort_pub.publish(Float64(data=0.0))
                self.speed_effort_pub.publish(Float64(data=self.speed_effort))
            else:
                self.node.get_logger().info(f"[{self.name}] Waiting for first buoy detection...", throttle_duration_sec=2.0)
                # Keep moving forward to find it
                self.yaw_effort_pub.publish(Float64(data=0.0))
                self.speed_effort_pub.publish(Float64(data=self.speed_effort))
            
            return Status.RUNNING

        # If detected
        self.has_seen_buoy = True
        self.frame_counter.reset()

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
        max_yaw = float(MissionParams.buoy_yaw_effort)
        if yaw_cmd > max_yaw: yaw_cmd = max_yaw
        elif yaw_cmd < -max_yaw: yaw_cmd = -max_yaw

        self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
        self.speed_effort_pub.publish(Float64(data=self.speed_effort))
            
        self.node.get_logger().info(
            f"[{self.name}] Approaching target - DSC: {self.dsc:.2f}, YawCmd: {yaw_cmd:.2f}",
            throttle_duration_sec=2.0
            )
        
        return Status.RUNNING


class Buoy_Fallback(BaseFallback):
    """
    """
    def __init__(self, name, node=None):
        super().__init__(name, node=node)

    def fallback(self) -> Status:
        return Status.FAILURE
