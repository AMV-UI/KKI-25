import py_trees
from typing import List, Tuple, Type, Callable

from ..behaviors.mission.actions.tuning.straight import Straight_Execution, Straight_Fallback
from ..behaviors.mission.actions.tuning.turn import Turn_Execution, Turn_Fallback
from ..behaviors.mission.actions.tuning.go import Go_Execution, Go_Fallback
from ..behaviors.mission.actions.tuning.done import Done_Execution, Done_Fallback
from ..behaviors.mission.actions.tuning.hardcode import *

from ..behaviors.mission.actions.mission.initial import *
from ..behaviors.mission.actions.mission.initial_dock import *
from ..behaviors.mission.actions.mission.buoy  import *
from ..behaviors.mission.actions.mission.finding import *
from ..behaviors.mission.actions.mission.dock import *
from ..behaviors.mission.actions.mission.dockv2 import *
from ..behaviors.mission.actions.mission.reverse import *
from ..behaviors.mission.actions.mission.box import *
from ..behaviors.mission.actions.mission.passthrough import *
from ..behaviors.mission.actions.mission.unfinding import *
from ..behaviors.mission.actions.mission.photo import *
from ..behaviors.mission.actions.mission.idle import *

from ..behaviors.base_behavior import BaseBehavior

from ..behaviors.mission.actions.mission.turn_next_buoy import *

from core.utils.config import Topic
from std_msgs.msg import Bool

class DynamicBuoySequence(py_trees.composites.Sequence):
    def __init__(self, name, memory=True, node=None):
        super().__init__(name=name, memory=memory)
        self.node = node

    def tick(self):
        """Override the tick generator to implement the custom loop logic."""
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        
        # If we are starting fresh, initialise
        if self.status != py_trees.common.Status.RUNNING:
            self.current_child = self.children[0]
            for child in self.children:
                child.status = py_trees.common.Status.INVALID

        for child in self.children:
            # Only tick the current child or the ones after it
            if child == self.current_child or self.current_child is None:
                yield from child.tick()
                
                if child.status == py_trees.common.Status.RUNNING:
                    self.status = py_trees.common.Status.RUNNING
                    self.current_child = child
                    yield self
                    return
                elif child.status == py_trees.common.Status.FAILURE:
                    # If any child fails (e.g. Turn_Next_Buoy times out), break the loop and return SUCCESS!
                    self.status = py_trees.common.Status.SUCCESS
                    self.current_child = None
                    yield self
                    return
                # If SUCCESS, we continue to the next child in the for-loop
                # We MUST set current_child to None so the next iteration will tick the next child!
                self.current_child = None
                
        # If all children returned SUCCESS (both Buoy and Turn_Next_Buoy succeeded),
        # it means we found a buoy and finished the sweep. We want to LOOP BACK!
        self.status = py_trees.common.Status.RUNNING
        self.current_child = self.children[0]
        # Invalidate children so they can run again
        for child in self.children:
            child.stop(py_trees.common.Status.INVALID)
            
        yield self

class MissionTreeBuilder:
    """Builder class responsible for constructing the behavior tree"""
    # Hard Code Config
    # MISSIONS_CONFIG = [
    #     (RecordGPSExecution, RecordGPSFallback, "Hardcode"),
    #     (Done_Execution, Done_Fallback, "Stall"),   
    # ]

    MISSIONS_CONFIG = [
        (InitialDock_Execution, InitialDock_Fallback, "InitialDock"),
        (Idle_Execution, Idle_Fallback, "Idle"),
        (Docking_Execution, Docking_Fallback, "Docking"),
        (Done_Execution, Done_Fallback, "Done"),
    ]

    # Perception
    MISSIONS_CONFIG = [
        (Initial_Execution, Initial_Fallback, "Initial", 0),
        ("BUOY_LOOP", 1), # Dynamic Loop for Buoys
        (Finding_Execution, Finding_Fallback, "Finding Both Boxes", 6), 
        (Photo_Execution, Photo_Fallback, "Photo Both Boxes", 7), 
        (Box_Execution, Box_Fallback, "Change Mission to Docking", 10),
        (Docking_Execution, Docking_Fallback, "Docking", 10),
        (Done_Execution, Done_Fallback, "Done", 10),
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
        
        for config in self.MISSIONS_CONFIG:
            if config[0] == "BUOY_LOOP":
                buoy_loop = DynamicBuoySequence("Dynamic Buoy Loop", memory=True, node=self.ros_node)
                
                buoy_sel = self._create_mission_selector(Buoy_Execution, Buoy_Fallback, "Buoy", 1)
                turn_sel = self._create_mission_selector(Turn_Next_Buoy_Execution, Turn_Next_Buoy_Fallback, "Turn Next Buoy", 1)
                
                buoy_loop.add_children([buoy_sel, turn_sel])
                mission_sequence.add_child(buoy_loop)
                
                # After the loop breaks, we must change mission to BOTH_BOXES
                box_sel = self._create_mission_selector(Box_Execution, Box_Fallback, "Change Mission to Both Boxes", 6)
                mission_sequence.add_child(box_sel)
            else:
                mission_class, fallback_class, name, mission_id = config
                mission_selector = self._create_mission_selector(
                    mission_class, 
                    fallback_class,
                    name, 
                    mission_id
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
