from ..mission_behaviors import BaseExecution, BaseFallback
from geometry_msgs.msg import Twist
from py_trees.common import Status
from core.utils.factory import TopicFactory
import time

class Mission2_Execution(BaseExecution):
    """
    Mission2: Moves the turtle in a triangle using simple state machine.
    """

    def __init__(self, name: str = "Mission2_Execution"):
        super().__init__(name)
        self.velocity_pub = None
        self.twist = Twist()
        self.state = 0
        self.last_time = time.time()

    def setup(self, **kwargs) -> None:
        super().setup()
        self.velocity_pub = TopicFactory("/turtle1/cmd_vel", Twist).createPublisher(self.node)

    def execute(self) -> Status:
        now = time.time()
        elapsed = now - self.last_time

        # Move forward for 2 seconds
        if self.state % 2 == 0 and elapsed < 2.0:
            self.twist.linear.x = 2.0
            self.twist.angular.z = 0.0
        # Rotate for 1.5 seconds (120 degrees roughly)
        elif self.state % 2 == 1 and elapsed < 1.5:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 2.0
        else:
            self.state += 1
            self.last_time = now
            if self.state >= 6:  # 3 sides (move + turn) * 3
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.velocity_pub.publish(self.twist)
                return Status.SUCCESS
            return Status.RUNNING

        self.velocity_pub.publish(self.twist)
        return Status.RUNNING


class Mission2_Fallback(BaseFallback):
    """
    Fallback for Mission2: Stops the turtle safely.
    """

    def __init__(self, name: str = "Mission2_Fallback"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.velocity_pub = TopicFactory("/turtle1/cmd_vel", Twist).createPublisher(self.node)

    def fallback(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.velocity_pub.publish(twist)
        self.node.get_logger().warn(f"[{self.name}] Fallback executed: stopping turtle")
