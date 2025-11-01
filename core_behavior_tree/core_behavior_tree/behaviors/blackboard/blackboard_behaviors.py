from core.utils.config import BT
from ..base_behavior import BaseBehavior
import py_trees
from core.mission.frame_counter import FrameCounter
from core.utils.config import TopicFactory
from std_msgs.msg import UInt8, Bool, Float64
import rclpy.qos

class TopicToBlackboard(BaseBehavior):
    def __init__(self, topic_name: str, topic_factory: TopicFactory, node=None):
        # keep name for tree readability
        super(TopicToBlackboard, self).__init__(topic_name + "_subscriber", node=node)
        self.topic_name = topic_name
        self.topic_factory = topic_factory
        # store last received value here instead of blackboard
        self.latest = None
    
    def setup(self, **kwargs):
        # topic_factory.createSubscriber should accept (node, callback) in your codebase
        self.subscriber = self.topic_factory.createSubscriber(self.node, self.subscriber_callback)
        return True

    def subscriber_callback(self, msg):
        self.latest = getattr(msg, "data", msg)

    def update(self):
        return BT.success


class InitializeBlackboard(BaseBehavior):
    """Initialize values (no py_trees blackboard). Writes onto the node object."""
    
    def __init__(self, node):
        super(InitializeBlackboard, self).__init__(name="InitBlackboard", node=node)

    def update(self):
        # Set attributes on the node so other behaviors can access them via node.<attr>
        n = self.node
        n.px_heading = -1
        n.detected = False
        n.dsc = float(160)
        n.pxmode = "AUTO"
        n.mission_status = "RUNNING"
        n.frame_counter = FrameCounter(2)
        n.frame_counter_manuver = FrameCounter(3)
        n.manuver_detected = False
        n.isChange = False
        n.find_mode = self._create_find_mode()
        n.image = None
        n.camera_bottom = None
        n.current_mission = "-"
        n.mission_counter = 0
        
        n.get_logger().info("Blackboard (node attributes) initialized with default values")
        return BT.success
    
    def _create_find_mode(self):
        class FindMode:
            def __init__(self, node):
                self.initial_heading = -1
                self.node = node
                self.GO_LEFT = -200
                self.GO_RIGHT = 200
                
        return FindMode(self.node)


class PrintBlackboard(BaseBehavior):
    """Monitor and print node attributes (used instead of blackboard monitor)"""
    
    def __init__(self, node=None):
        super(PrintBlackboard, self).__init__(name="BlackboardMonitor", node=node)
        
    def update(self):
        # Example: log current mission stored on node
        n = self.node
        if n is not None:
            n.get_logger().debug(f"Current mission: {getattr(n, 'current_mission', '-')}")
        return BT.running