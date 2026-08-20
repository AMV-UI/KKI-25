from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Float64, String, Bool
from core.utils.config import Topic, MissionParams
import time

class Turn_Next_Buoy_Execution(BaseExecution):
    """
    Simultaneously goes forward and turns left/right for a fixed duration.
    """
    def __init__(self, name: str = "Turn_Next_Buoy_Execution", node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node
        self.arena = "B"
        self.duration = MissionParams.turn_next_buoy_duration  # Duration for the turn maneuver
        self.speed_effort = MissionParams.turn_next_buoy_speed_effort
        self.yaw_effort = MissionParams.turn_next_buoy_yaw_effort
        self.start_time = 0.0
        self.mission = mission
        self.detected = False
        
    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
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
        self.box_detected_sub = Topic.box_detected.createSubscriber(
            self.node,
            self._box_detected_cb
        )

    def _box_detected_cb(self, msg: Bool):
        self.box_detected = bool(msg.data)

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)
        
    def _detected_cb(self, msg):
        self.detected = bool(msg.data)

    def initialise(self) -> None:
        self.box_detected = False
        self.start_time = time.time()
        self.global_start_time = time.time()
        self.direction = 1
        self.node.get_logger().info(f"[{self.name}] Starting sweeping turn maneuver in arena {self.arena}")

    def execute(self) -> Status:
        total_elapsed = time.time() - self.global_start_time
        if total_elapsed >= MissionParams.turn_next_buoy_timeout:
            self.node.get_logger().info(f"[{self.name}] Global timeout ({MissionParams.turn_next_buoy_timeout}s) reached! Moving to next mission.")
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.speed_effort_pub.publish(Float64(data=0.0))
            return Status.FAILURE

        elapsed = time.time() - self.start_time
        
        if self.detected and total_elapsed > 3.0:
            self.node.get_logger().info(f"[{self.name}] Target detected! Sweeping complete.")
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.speed_effort_pub.publish(Float64(data=0.0))
            return Status.SUCCESS
            
        if self.box_detected and total_elapsed > 3.0:
            self.node.get_logger().info(f"[{self.name}] Box detected! Breaking loop to go to Box Mission.")
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.speed_effort_pub.publish(Float64(data=0.0))
            return Status.FAILURE
        
        if elapsed >= self.duration:
            self.node.get_logger().info(f"[{self.name}] Sweeping time {self.duration}s up. Reversing direction.")
            self.direction *= -1
            self.start_time = time.time()
            elapsed = 0.0

        # Simultaneous forward and turn
        speed = self.speed_effort
        
        # Turn Right for Track A (positive yaw effort), Turn Left for Track B (negative yaw effort)
        base_yaw = self.yaw_effort if self.arena == "A" else -self.yaw_effort
        yaw = base_yaw * self.direction
        
        self.speed_effort_pub.publish(Float64(data=speed))
        self.yaw_effort_pub.publish(Float64(data=float(yaw)))
        
        self.node.get_logger().info(f"[{self.name}] Sweeping (Dir: {self.direction})... elapsed: {elapsed:.1f}s / {self.duration}s", throttle_duration_sec=1.0)
        return Status.RUNNING

class Turn_Next_Buoy_Fallback(BaseFallback):
    def __init__(self, name, node=None):
        super().__init__(name, node=node)

    def fallback(self) -> Status:
        return Status.FAILURE
