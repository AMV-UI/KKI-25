import py_trees
from ..behaviors.mission.mission_behaviors import BaseTestMission, FallbackAction
from ..behaviors.blackboard.blackboard_behaviors import InitializeBlackboard, PrintBlackboard, TopicToBlackBoard
from ..behaviors.actions.action_behaviors import ManuallyMoveBoat
from core.utils.config import Topic, BT

class MissionTreeBuilder:
    """Builder class responsible for constructing the behavior tree"""
    
    def __init__(self, ros_node):
        self.ros_node = ros_node
        
    def build(self):
        """Build and return the complete behavior tree"""
        root = self._create_root()
        behavior_tree = py_trees.trees.BehaviourTree(root)
        return behavior_tree
    
    def _create_root(self):
        """Create the root of the behavior tree with all major branches"""
        root = py_trees.composites.Parallel(
            name="Root",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
        )
        
        init = self._create_init_branch()
        tasks = self._create_tasks_branch()
        watcher = PrintBlackboard()
        
        root.add_children([init, tasks, watcher])
        return root
    
    def _create_init_branch(self):
        """Create the initialization branch of the tree"""
        init = py_trees.composites.Sequence(
            name="Init",
            memory=True
        )
        
        init_blackboard = InitializeBlackboard(self.ros_node)
        init_heading_deg_subscriber = TopicToBlackBoard("px_heading", Topic.heading_deg)
        init.add_children([init_blackboard, init_heading_deg_subscriber])
        
        return init
        
    def _create_tasks_branch(self):
        """Create the tasks branch of the tree"""
        tasks = py_trees.composites.Selector(
            name="Tasks",
            memory=False
        )
        
        # Create mission sequence
        mission_sequence = self._create_mission_sequence()
        
        # Create manual movement guard
        pxmode_manual = ManuallyMoveBoat("ManualMovement")
        
        def check_pxmode(blackboard):
            return True if blackboard.pxmode == "manual" else False
        
        isManualMoving = py_trees.decorators.EternalGuard(
            "is manual moving?",
            pxmode_manual,
            check_pxmode,
            blackboard_keys={"pxmode"}
        )
        
        # Add children to tasks
        tasks.add_children([
            isManualMoving, 
            mission_sequence, 
            py_trees.behaviours.Running("Idle")
        ])
        
        return tasks
    
    def _create_mission_sequence(self):
        """Create the mission sequence branch"""
        mission_sequence = py_trees.composites.Sequence(
            name="Mission Sequence", 
            memory=False
        )
        
        # Create mission selectors
        for i in range(1, 4):  # Create missions 1-3
            mission_selector = py_trees.composites.Selector(
                name=f"mission{i}Succeed?", 
                memory=True
            )
            
            mission = BaseTestMission(f"mission{i}")
            fallback = FallbackAction(f"mission{i} Fallback")
            
            mission_selector.add_children([mission, fallback])
            mission_sequence.add_child(mission_selector)
        
        return mission_sequence