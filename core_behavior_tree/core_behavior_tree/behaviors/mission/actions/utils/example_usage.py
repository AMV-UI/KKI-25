"""
Example: How to refactor tuning scripts using movement utilities.

This file demonstrates how to use the movement utilities in your behavior tree nodes.
"""

from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Float64, UInt8
from core.utils.config import Topic
from ..utils.movement import MovementController


# Example 1: Refactored Straight Execution using MovementController class
class StraightExecutionRefactored(BaseExecution):
    """
    Example of using MovementController for straight movement.
    """
    def __init__(self, name: str = "Straight_Execution", node=None):
        super().__init__(name, node=node)
        self.node = node
        self.success = False
        self.effort = 100.0
        self.movement_controller = None

    def initialise(self):
        self.effort = 100.0
        self.success = False

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        
        # Setup subscribers
        self.mission_sub = Topic.tuning_mission.createSubscriber(
            self.node,
            self._mission_cb
        )        
        self.effort_sub = Topic.tuning_effort_st.createSubscriber(
            self.node,
            self._effort_cb
        )        

        # Setup publishers
        yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
        # Initialize movement controller
        self.movement_controller = MovementController(
            yaw_effort_pub,
            speed_effort_pub,
            logger=self.node.get_logger()
        )

    def _mission_cb(self, msg: UInt8):
        if msg.data == 1:
            self.success = True

    def _effort_cb(self, msg: Float64):
        self.effort = float(msg.data)

    def execute(self) -> Status:
        if self.success:
            return Status.SUCCESS

        # Use movement controller instead of manual publishing
        self.movement_controller.go_straight(self.effort)
        
        return Status.RUNNING


# Example 2: Refactored Turn Execution
class TurnExecutionRefactored(BaseExecution):
    """
    Example of using MovementController for turning.
    """
    def __init__(self, name: str = "Turn_Execution", node=None):
        super().__init__(name, node=node)
        self.node = node
        self.success = False
        self.effort = 100.0
        self.movement_controller = None

    def initialise(self):
        self.effort = 100.0
        self.success = False

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        
        self.mission_sub = Topic.tuning_mission.createSubscriber(
            self.node,
            self._mission_cb
        )        
        self.effort_sub = Topic.tuning_effort_tn.createSubscriber(
            self.node,
            self._effort_cb
        )        

        yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
        self.movement_controller = MovementController(
            yaw_effort_pub,
            speed_effort_pub,
            logger=self.node.get_logger()
        )

    def _mission_cb(self, msg: UInt8):
        if msg.data == 1:
            self.success = True

    def _effort_cb(self, msg: Float64):
        self.effort = float(msg.data)

    def execute(self) -> Status:
        if self.success:
            return Status.SUCCESS

        # Use movement controller for turning
        # Positive effort turns left, negative turns right
        self.movement_controller.turn(self.effort)
        
        return Status.RUNNING


# Example 3: Refactored Go Execution (combined movement)
class GoExecutionRefactored(BaseExecution):
    """
    Example of using MovementController for combined yaw and speed.
    """
    def __init__(self, name: str = "Go_Execution", node=None):
        super().__init__(name, node=node)
        self.node = node
        self.success = False
        self.effort_tn = 0.0
        self.effort_st = 0.0
        self.movement_controller = None

    def initialise(self):
        self.effort_st = 0.0
        self.effort_tn = 0.0
        self.success = False

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        
        self.mission_sub = Topic.tuning_mission.createSubscriber(
            self.node,
            self._mission_cb
        )        
        self.effort_tn_sub = Topic.tuning_effort_tn.createSubscriber(
            self.node,
            self._effort_tn_cb
        )        
        self.effort_st_sub = Topic.tuning_effort_st.createSubscriber(
            self.node,
            self._effort_st_cb
        )        

        yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
        self.movement_controller = MovementController(
            yaw_effort_pub,
            speed_effort_pub,
            logger=self.node.get_logger()
        )

    def _mission_cb(self, msg: UInt8):
        if msg.data == 1:
            self.success = True

    def _effort_st_cb(self, msg: Float64):
        self.effort_st = float(msg.data)

    def _effort_tn_cb(self, msg: Float64):
        self.effort_tn = float(msg.data)

    def execute(self) -> Status:
        if self.success:
            return Status.SUCCESS

        # Use movement controller for combined movement
        self.movement_controller.move(self.effort_tn, self.effort_st)
        
        return Status.RUNNING


# Example 4: Refactored Done Execution (stop)
class DoneExecutionRefactored(BaseExecution):
    """
    Example of using MovementController to stop.
    """
    def __init__(self, name: str = "Done_Execution", node=None):
        super().__init__(name, node=node)
        self.node = node
        self.success = False
        self.movement_controller = None

    def initialise(self):
        self.success = False

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        
        self.mission_sub = Topic.tuning_mission.createSubscriber(
            self.node,
            self._mission_cb
        )        

        yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
        self.movement_controller = MovementController(
            yaw_effort_pub,
            speed_effort_pub,
            logger=self.node.get_logger()
        )

    def _mission_cb(self, msg: UInt8):
        if msg.data == 10:
            self.success = True

    def execute(self) -> Status:
        if self.success:
            return Status.SUCCESS

        # Use movement controller to stop
        self.movement_controller.stop()
        
        return Status.RUNNING


# Example 5: Using standalone functions (alternative approach)
from ..utils.movement import publish_stop, publish_straight, publish_turn, publish_move

class AlternativeApproach(BaseExecution):
    """
    Example using standalone functions instead of the class.
    """
    def __init__(self, name: str = "Alternative", node=None):
        super().__init__(name, node=node)
        self.node = node
        self.effort = 100.0

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)

    def execute(self) -> Status:
        # Option 1: Go straight
        publish_straight(self.yaw_effort_pub, self.speed_effort_pub, self.effort)
        
        # Option 2: Turn
        # publish_turn(self.yaw_effort_pub, self.speed_effort_pub, self.effort)
        
        # Option 3: Stop
        # publish_stop(self.yaw_effort_pub, self.speed_effort_pub)
        
        # Option 4: Combined movement
        # publish_move(self.yaw_effort_pub, self.speed_effort_pub, yaw=50.0, speed=100.0)
        
        return Status.RUNNING
