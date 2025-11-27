import py_trees
from typing import List, Tuple, Type, Callable

from ..behaviors.mission.actions.tuning.straight import Straight_Execution, Straight_Fallback
from ..behaviors.mission.actions.tuning.turn import Turn_Execution, Turn_Fallback
from ..behaviors.mission.actions.tuning.go import Go_Execution, Go_Fallback
from ..behaviors.mission.actions.tuning.done import Done_Execution, Done_Fallback
from ..behaviors.mission.actions.tuning.hardcode import *

from ..behaviors.mission.actions.mission.initial import Initial_Execution, Initial_Fallback
from ..behaviors.mission.actions.mission.buoy  import Buoy_Execution, Buoy_Fallback
from ..behaviors.mission.actions.mission.finding import Finding_Execution, Finding_Fallback
from ..behaviors.mission.actions.mission.dock import Docking_Execution, Docking_Fallback

from core.utils.config import Topic, BT

from ..behaviors.base_behavior import BaseBehavior

class MissionTreeBuilder:
    """Builder class responsible for constructing the behavior tree"""


    # Hard Code Config
    # MISSIONS_CONFIG = [
    #     (RecordGPSExecution, RecordGPSFallback, "Hardcode"),
    #     (Done_Execution, Done_Fallback, "Stall"),   
    # ]

    # Record Straight Config
    # MISSIONS_CONFIG = [
    #     (RecordStraightExecution, Straight_Fallback, "Straight"),
    # ]

    #MISSIONS_CONFIG = [
        
       #(Mission1_Execution, Mission1_Fallback, "Mission 1"),
       #(Mission2_Execution, Mission2_Fallback, "Mission 2"),
       #(Mission3_Execution, Mission3_Fallback, "Mission 3"),
    #]

    # Tuning Config
    # MISSIONS_CONFIG = [
    #      (Straight_Execution, Done_Execution, "Straight"),
    #      (Done_Execution, Done_Fallback, "Stall"),
    #      (Turn_Execution, Done_Execution, "Turn"),
    #      (Done_Execution, Done_Fallback, "Stall"),
    #      (Go_Execution, Go_Fallback, "Go"),
    #      (Done_Execution, Done_Fallback, "Done"),
    # ]

    # Perception
    MISSIONS_CONFIG = [
        # (Initial_Execution, Initial_Fallback, "Initial"),
        # (Buoy_Execution, Buoy_Fallback, "Buoy"),
        (Finding_Execution, Finding_Fallback, "Finding"), 
        # (Buoy_Execution, Buoy_Fallback, "Buoy"),
        #(Docking_Execution, Docking_Fallback, "Docking"),
        # (Done_Execution, Done_Fallback, "Done"),
    ]

    def __init__(self, ros_node):
        self.ros_node = ros_node
        BaseBehavior.set_ros_node(ros_node)
        
    def build(self) -> py_trees.trees.BehaviourTree:
        """Build and return the complete behavior tree"""
        root = self._create_root()
        return py_trees.trees.BehaviourTree(root)
    
    def _create_root(self) -> py_trees.composites.Parallel:
        """Create the root of the behavior tree with all major branches"""
        root = py_trees.composites.Parallel(
            name="Root",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
        )
        
        tasks_branch = self._create_tasks_branch()
        
        root.add_children([tasks_branch])
        return root
    
        
    def _create_tasks_branch(self) -> py_trees.composites.Selector:
        """Create the tasks branch of the tree"""
        tasks = py_trees.composites.Selector(
            name="Tasks",
            memory=False
        )
        
        mission_sequence = self._create_mission_sequence()
        
        tasks.add_children([mission_sequence])
        return tasks
    
    def _create_mission_sequence(self) -> py_trees.composites.Sequence:
        """Create the mission sequence branch with improved structure"""
        mission_sequence = py_trees.composites.Sequence(
            name="Mission Sequence", 
            memory=True
        )
        
        for i, (mission_class, fallback_class, name) in enumerate(self.MISSIONS_CONFIG, start=1):
            mission_selector = self._create_mission_selector(
                mission_class, 
                fallback_class,
                name, 
                i
            )
            mission_sequence.add_child(mission_selector)
        
        return mission_sequence
    
    def _create_mission_selector(
        self, 
        mission_class: Type, 
        fallback_class: Type, 
        name: str,
        mission_number: int) -> py_trees.composites.Selector:

        """Create a selector for a specific mission with its fallback"""


        selector = py_trees.composites.Selector(
            name=f"Mission{mission_number}",
            memory=True
        )
        
        mission = mission_class(f"{name} Execution", node=self.ros_node, mission=mission_number)
        fallback = fallback_class(f"{name} Fallback", node=self.ros_node)

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
        """Return a DOT representation of the tree for visualization"""
        root = self._create_root()
        return py_trees.display.render_dot_tree(root)
