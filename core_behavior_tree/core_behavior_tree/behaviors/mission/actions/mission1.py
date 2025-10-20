from ..mission_behaviors import BaseExecution, BaseFallback
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from py_trees.common import Status
from core.utils.factory import TopicFactory
import time

class Mission1_Execution(BaseExecution):
    """
    Mission1: Hardcoded movement to move the asv on asvsim by publishing the topic on /cmd_vel, refer to the Scripts/Controllers/Core/CoreController.cs
    for the RosTCPConnector connection subscriber on /cmd_vel
    """
    def __init__(self, name: str = "Mission1_Execution"):
        super().__init__(name)
        self.velocity_pub = None
        self.pose_sub = None
        self.twist = Twist()
        self.current_pose = None


    def setup(self, **kwargs) -> None:
        super().setup()
        self.velocity_pub = TopicFactory("/cmd_vel", Twist).createPublisher(self.node)
        self.pose_sub = TopicFactory("/pose", Pose).createSubscriber(self.node, self._pose_callback) 

    def _pose_callback(self, msg):
        self.current_pose = msg
   
    def execute(self) -> Status:
        self.twist.linear.x = 2.0
        self.twist.angular.z = 0.0
        self.velocity_pub.publish(self.twist)
        
        return Status.RUNNING

class Mission1_Fallback(BaseFallback):
    """
    Fallback for Mission1
    """
    def __init__(self, name: str = "Mission1_Fallback"):
        super().__init__(name)
        self.start_time = None
        self.fallback_duration = 3.0  
        self.velocity_pub = None
        
    def setup(self, **kwargs):
        super().setup()
        self.velocity_pub = TopicFactory("/cmd_vel", Twist).createPublisher(self.node)
        self.node.get_logger().info(f"[{self.name}] Starting wall collision recovery")
    
    def fallback(self):
        if self.start_time is None:
            self.start_time = time.time()
        
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time >= self.fallback_duration:
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
