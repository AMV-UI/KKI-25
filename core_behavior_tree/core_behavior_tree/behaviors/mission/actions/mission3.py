from ..mission_behaviors import BaseExecution, BaseFallback
from geometry_msgs.msg import Twist
from py_trees.common import Status
from core.utils.factory import TopicFactory
from core.utils.frame_counter import FrameCounter
from core.utils.config import BT


class Mission3_Execution(BaseExecution):
    """
    Mission3: Moves the turtle in a triangle using frame counting.
    """

    def __init__(self, name: str = "Mission3_Execution"):
        extra_keys = {
            "frame_counter": BT.read  # assumes already initialized in blackboard
        }
        super().__init__(name, extra_keys)
        self.velocity_pub = None
        self.twist = Twist()
        self.phase = 0  # move = 0, turn = 1, repeat

    def setup(self, **kwargs) -> None:
        super().setup()
        self.velocity_pub = TopicFactory("/turtle1/cmd_vel", Twist).createPublisher(self.node)

    def execute(self) -> Status:
        counter = self.blackboard.frame_counter

        if self.phase % 2 == 0:
            self.twist.linear.x = 2.0
            self.twist.angular.z = 0.0
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 2.0

        self.velocity_pub.publish(self.twist)

        # Advance to next phase every N frames
        if counter.count():
            self.phase += 1
            if self.phase >= 6:  # 3 sides (move + turn) * 3
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.velocity_pub.publish(self.twist)
                return Status.SUCCESS

        return Status.RUNNING


class Mission3_Fallback(BaseFallback):
    """
    Fallback for Mission3: Stops the turtle safely.
    """

    def __init__(self, name: str = "Mission3_Fallback"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.velocity_pub = TopicFactory("/turtle1/cmd_vel", Twist).createPublisher(self.node)

    def fallback(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.velocity_pub.publish(twist)
        self.node.get_logger().warn(f"[{self.name}] Fallback executed: stopping turtle")
