from core.utils.config import BT
from ..base_behavior import BaseBehavior
import py_trees
import py_trees.blackboard
from core.utils.frame_counter import FrameCounter
from core.utils.config import TopicFactory
from std_msgs.msg import UInt8, Bool, Float64
import rclpy.qos

class TopicToBlackboard(BaseBehavior):
    def __init__(self, topic_name: str, topic_factory: TopicFactory):
        super(TopicToBlackboard, self).__init__(topic_name + "_subscriber")
        self.topic_name = topic_name
        self.topic_factory = topic_factory
        self.blackboard.register_key(topic_name, access=BT.write)
    
    def setup(self, **kwargs):
        self.subscriber = self.topic_factory.createSubscriber(self.node, self.subscriber_callback)
        return True

    def subscriber_callback(self, msg):
        self.blackboard.set(name=self.topic_name, value=msg.data)
 
    def update(self):
        return BT.success


class InitializeBlackboard(BaseBehavior):
    """Initialize the blackboard with default values"""
    
    def __init__(self, node):
        # Register all keys from BT.ALL
        keys_to_register = {
            "mission_counter": BT.write,
            "ros_node": BT.write
            }
        for key, value in vars(BT.ALL).items():
            if not key.startswith("__") and isinstance(value, tuple):
                keys_to_register[value[0]] = value[1]
                
        super(InitializeBlackboard, self).__init__(name="InitBlackboard", keys_to_register=keys_to_register)
        self.blackboard.ros_node = node

    def update(self):
        # Initialize constants
        self.blackboard.px_heading = -1
        self.blackboard.detected = False
        self.blackboard.dsc = float(160)
        self.blackboard.pxmode = "HOLD"  # Using string instead of enum for clarity
        self.blackboard.mission_status = "RUNNING"
        self.blackboard.frame_counter = FrameCounter(2)
        self.blackboard.frame_counter_manuver = FrameCounter(3)
        self.blackboard.manuver_detected = False
        self.blackboard.isChange = False
        self.blackboard.find_mode = self._create_find_mode()
        self.blackboard.image = None
        self.blackboard.camera_bottom = None
        self.blackboard.current_mission = "-"
        self.blackboard.mission_counter = 0
        
        self.node.get_logger().info("Blackboard initialized with default values")
        return BT.success
    
    def _create_find_mode(self):
        # This is a simplified placeholder for FindMode
        class FindMode:
            def __init__(self, node):
                self.initial_heading = -1
                self.node = node
                self.GO_LEFT = -200
                self.GO_RIGHT = 200
                
        return FindMode(self.node)


class PrintBlackboard(BaseBehavior):
    """Monitor and print blackboard values for debugging"""
    
    def __init__(self):
        keys_to_register = {}
        # Register all keys from BT.ALL for reading
        for key, value in vars(BT.ALL).items():
            if not key.startswith("__") and isinstance(value, tuple):
                keys_to_register[value[0]] = BT.read
                
        super(PrintBlackboard, self).__init__(name="BlackboardMonitor", keys_to_register=keys_to_register)
        
    def update(self):
        # Mainly for logging/printing logic here
        # For example:
        # self.node.get_logger().debug("Current mission: " + str(self.blackboard.get("current_mission")))
        return BT.running