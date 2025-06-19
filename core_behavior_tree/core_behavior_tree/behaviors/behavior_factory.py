from .blackboard.blackboard_behaviors import TopicToBlackboard, InitializeBlackboard, PrintBlackboard
from .mission.mission_behaviors import BaseTestMission, FallbackAction
from .actions.action_behaviors import ManuallyMoveBoat
from core.utils.factory import TopicFactory
from core.utils.config import Topic

class BehaviorFactory:
    """Factory for creating behavior tree nodes"""
    
    @staticmethod
    def create_topic_to_blackboard(topic_name: str, topic_factory: TopicFactory):
        """Create a TopicToBlackboard behavior"""
        return TopicToBlackboard(topic_name, topic_factory)
    
    @staticmethod
    def create_initialize_blackboard(node):
        """Create an InitializeBlackboard behavior"""
        return InitializeBlackboard(node)
    
    @staticmethod
    def create_print_blackboard():
        """Create a PrintBlackboard behavior"""
        return PrintBlackboard()
    
    @staticmethod
    def create_mission(name: str):
        """Create a mission behavior"""
        return BaseTestMission(name)
    
    @staticmethod
    def create_fallback(name: str):
        """Create a fallback behavior"""
        return FallbackAction(name)
    
    @staticmethod
    def create_manual_movement(name: str = "ManualMovement"):
        """Create a manual movement behavior"""
        return ManuallyMoveBoat(name)
    
    @staticmethod
    def create_px_heading_subscriber():
        """Create a subscriber for pixhawk heading"""
        return TopicToBlackboard("px_heading", Topic.heading_deg)