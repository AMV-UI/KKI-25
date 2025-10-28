import py_trees
from typing import List, Tuple, Type, Callable
from ..behaviors.mission.actions.sim import (
    Mission1_Execution, Mission1_Fallback,
    Mission2_Execution, Mission2_Fallback, 
    Mission3_Execution, Mission3_Fallback
)
from ..behaviors.blackboard.blackboard_behaviors import (
    InitializeBlackboard, 
    PrintBlackboard, 
    TopicToBlackboard
)
from core.utils.config import Topic, BT


class MissionTreeBuilder:
    """Builder class responsible for constructing the behavior tree"""
    
    MISSIONS_CONFIG = [
        (Mission1_Execution, Mission1_Fallback),
        (Mission2_Execution, Mission2_Fallback),
        (Mission3_Execution, Mission3_Fallback)
    ]
    
    def __init__(self, ros_node):
        self.ros_node = ros_node
        
    def build(self) -> py_trees.trees.BehaviourTree:
        """Build and return the complete behavior tree"""
        root = self._create_root()
        return py_trees.trees.BehaviourTree(root)
    
    def _create_root(self) -> py_trees.composites.Parallel:
        root = py_trees.composites.Parallel(
        """Create the root of the behavior tree with all major branches"""
            name="Root",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
        )
        
        init_branch = self._create_init_branch()
        tasks_branch = self._create_tasks_branch()
        watcher = PrintBlackboard()
        
        root.add_children([init_branch, tasks_branch, watcher])
        return root
    
    def _create_init_branch(self) -> py_trees.composites.Sequence:
        """Create the initialization branch of the tree"""
        init_sequence = py_trees.composites.Sequence(
            name="Init",
            memory=True
        )
        
        init_blackboard = InitializeBlackboard(self.ros_node)
        heading_subscriber = TopicToBlackboard("px_heading", Topic.heading_deg)
        
        init_sequence.add_children([init_blackboard, heading_subscriber])
        return init_sequence
        
    def _create_tasks_branch(self) -> py_trees.composites.Selector:
        """Create the tasks branch of the tree"""
        tasks = py_trees.composites.Selector(
            name="Tasks",
            memory=False
        )
        
        manual_guard = self._create_manual_movement_guard()
        mission_sequence = self._create_mission_sequence()
        idle_behavior = py_trees.behaviours.Running("Idle")
        
        tasks.add_children([manual_guard, mission_sequence, idle_behavior])
        return tasks
    
    def _create_manual_movement_guard(self) -> py_trees.decorators.EternalGuard:
        """Create the manual movement guard with proper condition checking"""
        # Note: MoveTurtle import seems to be missing from original code
        # You'll need to import this or replace with the correct class
        try:
            from ..behaviors.movement import MoveTurtle  # Adjust import as needed
            manual_movement = MoveTurtle("ManualMovement")
        except ImportError:
            # Fallback behavior if MoveTurtle is not available
            manual_movement = py_trees.behaviours.Success("ManualMovement")
        
        def is_manual_mode(blackboard) -> bool:
            """Check if the system is in manual mode"""
            return getattr(blackboard, 'pxmode', None) == "manual"
        
        return py_trees.decorators.EternalGuard(
            name="Manual Mode Guard",
            child=manual_movement,
            condition=is_manual_mode,
            blackboard_keys={"pxmode"}
        )
    
    def _create_mission_sequence(self) -> py_trees.composites.Sequence:
        """Create the mission sequence branch with improved structure"""
        mission_sequence = py_trees.composites.Sequence(
            name="Mission Sequence", 
            memory=False
        )
        
        for i, (mission_class, fallback_class) in enumerate(self.MISSIONS_CONFIG, start=1):
            mission_selector = self._create_mission_selector(
                mission_class, 
                fallback_class, 
                i
            )
            mission_sequence.add_child(mission_selector)
        
        return mission_sequence
    
    def _create_mission_selector(
        self, 
        mission_class: Type, 
        fallback_class: Type, 
        mission_number: int) -> py_trees.composites.Selector:

        """Create a selector for a specific mission with its fallback"""


        selector = py_trees.composites.Selector(
            name=f"Mission{mission_number} Success Check",
            memory=True
        )
        
        mission = mission_class(f"Mission{mission_number}")
        fallback = fallback_class(f"Mission{mission_number} Fallback")
        
        selector.add_children([mission, fallback])
        return selector
    
    @classmethod
    def add_mission_config(
        cls, 
        mission_class: Type, 
        fallback_class: Type) -> None:

        """Add a new mission configuration (useful for extending missions)"""
        cls.MISSIONS_CONFIG.append((mission_class, fallback_class))
    
    def get_tree_structure(self) -> str:
        """Return a string representation of the tree structure for debugging"""
        root = self._create_root()
        return py_trees.console.ascii_tree(root)

    def get_tree_ascii(self) -> str:
        return py_trees.display.render_dot_tree(root)
