import py_trees
from core.utils.config import BT

class BlackboardConfig:
    """Centralized configuration for blackboard keys and access patterns"""
    
    @staticmethod
    def register_all_bt_keys(blackboard_client):
        """Register all BT.ALL keys to the provided blackboard client"""
        for key, value in vars(BT.ALL).items():
            if not key.startswith("__") and isinstance(value, tuple):
                blackboard_client.register_key(value[0], access=value[1])
                
    @staticmethod
    def get_mission_keys():
        """Return dict of keys needed for mission behaviors"""
        return {
            "mission_counter": BT.read,
            "px_heading": BT.read
        }