import py_trees
from ..behaviors.mission.childrens.mission1 import Mission1, Mission1_Fallback
from ..behaviors.mission.childrens.mission2 import Mission2, Mission2_Fallback
from ..behaviors.mission.childrens.mission3 import Mission3, Mission3_Fallback
from ..behaviors.blackboard.blackboard_behaviors import InitializeBlackboard, PrintBlackboard, TopicToBlackboard
from ..behaviors.actions.action_behaviors import MoveTurtle
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
        init_heading_deg_subscriber = TopicToBlackboard("px_heading", Topic.heading_deg)
        init.add_children([init_blackboard, init_heading_deg_subscriber])
        
        return init
        
    def _create_tasks_branch(self):
        """Create the tasks branch of the tree"""
        tasks = py_trees.composites.Selector(
            name="Tasks",
            memory=False
        )
        
        mission_sequence = self._create_mission_sequence()
        pxmode_manual = MoveTurtle("ManualMovement")
        
        def check_pxmode(blackboard):
            return True if blackboard.pxmode == "manual" else False
        
        isManualMoving = py_trees.decorators.EternalGuard(
            "is manual moving?",
            pxmode_manual,
            check_pxmode,
            blackboard_keys={"pxmode"}
        )
        
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
        
        missions = [
            (Mission1, Mission1_Fallback),
            (Mission2, Mission2_Fallback),
            (Mission3, Mission3_Fallback)
        ]

        for i, (MissionClass, FallbackClass) in enumerate(missions, start=1):
            mission_selector = py_trees.composites.Selector(
                name=f"Mission{i} Success Check",
                memory=True
            )
            
            mission = MissionClass(f"Mission{i}")
            fallback = FallbackClass(f"Mission{i} Fallback")

            mission_selector.add_children([mission, fallback])
            mission_sequence.add_child(mission_selector)
        
        return mission_sequence