from ..mission_behaviors import BaseMission, BaseFallback
from geometry_msgs.msg import Twist
from py_trees.common import Status
from core.utils.factory import TopicFactory

class Mission1(BaseMission):
    """
    Mission1: Commands the turtle to move forward.
    """

    def __init__(self, name: str = "Mission1"):
        super().__init__(name)
        self.velocity_pub = None
        self.twist = Twist()

    def setup(self, **kwargs) -> None:
        super().setup()
        self.velocity_pub = TopicFactory("/turtle1/cmd_vel", Twist).createPublisher(self.node)

    def evaluate(self) -> Status:
        self.twist.linear.x = 2.0
        self.twist.angular.z = 0.0
        self.velocity_pub.publish(self.twist)

        # Placeholder: return SUCCESS immediately
        return Status.SUCCESS


class Mission1_Fallback(BaseFallback):
    """
    Fallback for Mission1: Stops the turtle as a safe action.
    """

    def __init__(self, name: str = "Mission1Fallback"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.velocity_pub = TopicFactory("/turtle1/cmd_vel", Twist).createPublisher(self.node)

    def execute_fallback(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.velocity_pub.publish(twist)
        self.node.get_logger().warn(f"[{self.name}] Fallback executed: stopping turtle")
