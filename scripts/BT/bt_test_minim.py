import py_trees
import py_trees.behaviours as behaviours
import py_trees.display
import time

class Patrol(py_trees.behaviour.Behaviour):
    def __init__(self, name="Patrol"):
        super(Patrol, self).__init__(name)

    def update(self):
        self.logger.info("Patrolling the area...")
        return py_trees.common.Status.SUCCESS

class Docking(py_trees.behaviour.Behaviour):
    def __init__(self, name="Docking"):
        super(Docking, self).__init__(name)

    def update(self):
        self.logger.info("Docking to base...")
        return py_trees.common.Status.SUCCESS

def create_behavior_tree():
    root = py_trees.composites.Selector("Mission Selector", memory=True)
    
    patrol_seq = py_trees.composites.Sequence("Patrol Mission", memory=True)
    patrol_action = Patrol()
    patrol_seq.add_child(patrol_action)

    docking_seq = py_trees.composites.Sequence("Docking Mission", memory=True)
    docking_action = Docking()
    docking_seq.add_child(docking_action)

    root.add_children([patrol_seq, docking_seq])
    
    return root

if __name__ == "__main__":
    tree = create_behavior_tree()
    py_trees.display.render_dot_tree(tree)  # Visualize the tree structure

    tree.tick_once()
    time.sleep(1)
    tree.tick_once()
