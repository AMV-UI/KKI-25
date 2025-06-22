from ..mission_behaviors import BaseMission, BaseFallback
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from py_trees.common import Status
from core.utils.factory import TopicFactory
import time

class Mission1(BaseMission):
    """
    Mission1: Commands the turtle to move forward until it hits a wall.
    Detects wall collision by checking boundary positions.
    """
    def __init__(self, name: str = "Mission1"):
        super().__init__(name)
        self.velocity_pub = None
        self.pose_sub = None
        self.twist = Twist()
        self.current_pose = None
        self.min_boundary = 0.0
        self.max_boundary = 11.0
        self.boundary_tolerance = 0.1 


    def setup(self, **kwargs) -> None:
        super().setup()
        self.velocity_pub = TopicFactory("/turtle1/cmd_vel", Twist).createPublisher(self.node)
        self.pose_sub = TopicFactory("/turtle1/pose", Pose).createSubscriber(
            self.node, self._pose_callback
        )
        self.node.get_logger().info(f"[{self.name}] Mission started - moving forward until wall hit")
    
    def _pose_callback(self, msg):
        """Callback to update current turtle position"""
        self.current_pose = msg
    
    def _is_at_boundary(self) -> bool:
        """
        Check if turtle is at any boundary by looking for clamped positions
        """
        if self.current_pose is None:
            return False
        
        x, y = self.current_pose.x, self.current_pose.y
        
        at_left = x <= (self.min_boundary + self.boundary_tolerance)
        at_right = x >= (self.max_boundary - self.boundary_tolerance)
        at_bottom = y <= (self.min_boundary + self.boundary_tolerance)
        at_top = y >= (self.max_boundary - self.boundary_tolerance)
        
        if at_left or at_right or at_bottom or at_top:
            self.node.get_logger().info(f"[{self.name}] At boundary: x={x:.2f}, y={y:.2f}")
            return True
        
        return False
    
    def evaluate(self) -> Status:
        if self._is_at_boundary():
            self.node.get_logger().warn(f"[{self.name}] Wall collision detected - mission failed")
            stop_twist = Twist()
            self.velocity_pub.publish(stop_twist)
            return Status.FAILURE  # Trigger fallback
        
        self.twist.linear.x = 2.0
        self.twist.angular.z = 0.0
        self.velocity_pub.publish(self.twist)
        
        return Status.RUNNING

class Mission1_Fallback(BaseFallback):
    """
    Fallback for Mission1: Recovery maneuver after wall collision.
    """
    def __init__(self, name: str = "Mission1Fallback"):
        super().__init__(name)
        self.start_time = None
        self.fallback_duration = 3.0  # Recovery duration
        self.velocity_pub = None
        
    def setup(self, **kwargs):
        super().setup()
        self.velocity_pub = TopicFactory("/turtle1/cmd_vel", Twist).createPublisher(self.node)
        self.node.get_logger().info(f"[{self.name}] Starting wall collision recovery")
    
    def execute_fallback(self):
        if self.start_time is None:
            self.start_time = time.time()
        
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time >= self.fallback_duration:
            # Complete fallback - stop turtle
            stop_twist = Twist()
            self.velocity_pub.publish(stop_twist)
            self.node.get_logger().info(f"[{self.name}] Recovery completed - proceeding to next mission")
            return Status.SUCCESS
        
        # Recovery maneuver: back up and turn
        twist = Twist()
        if elapsed_time < 1.0:
            # First second: back up
            twist.linear.x = -3.0
            twist.angular.z = 2.0
        else:
            # Remaining time: turn
            twist.linear.x = 0.0
            twist.angular.z = 2.0
        
        self.velocity_pub.publish(twist)
        return Status.RUNNING
