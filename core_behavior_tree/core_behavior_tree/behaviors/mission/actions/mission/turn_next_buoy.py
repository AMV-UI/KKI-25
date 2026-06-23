from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Float64, String
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
        
        from std_msgs.msg import Bool
        self.detected_sub = Topic.detected.createSubscriber(
            self.node,
            self._detected_cb
        )

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)
        
    def _detected_cb(self, msg):
        self.detected = bool(msg.data)

    def initialise(self) -> None:
        self.start_time = time.time()
        self.node.get_logger().info(f"[{self.name}] Starting simultaneous turn and forward maneuver for {self.duration}s in arena {self.arena}")

    def execute(self) -> Status:
        elapsed = time.time() - self.start_time
        
        if elapsed >= self.duration:
            self.node.get_logger().info(f"[{self.name}] Maneuver complete. Detected: {self.detected}")
            # Stop the boat
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.speed_effort_pub.publish(Float64(data=0.0))
            
            if self.detected:
                self.node.get_logger().info(f"[{self.name}] Buoy STILL detected! Looping back to Buoy Mission.")
                return Status.FAILURE
            else:
                self.node.get_logger().info(f"[{self.name}] No buoy detected. Proceeding to Box Mission.")
                return Status.SUCCESS

        # Simultaneous forward and turn
        speed = self.speed_effort
        
        # Turn Left for Track A (negative yaw effort), Turn Right for Track B (positive yaw effort)
        yaw = self.yaw_effort if self.arena == "A" else -self.yaw_effort
        
        self.speed_effort_pub.publish(Float64(data=speed))
        self.yaw_effort_pub.publish(Float64(data=yaw))
        
        self.node.get_logger().info(f"[{self.name}] Turning... elapsed: {elapsed:.1f}s", throttle_duration_sec=1.0)
        return Status.RUNNING

class Turn_Next_Buoy_Fallback(BaseFallback):
    def __init__(self, name, node=None):
        super().__init__(name, node=node)

    def fallback(self) -> Status:
        return Status.FAILURE
