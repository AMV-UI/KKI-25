import py_trees


def build(behaviors):
    mission_sequence = py_trees.composites.Sequence(
        name="Mission Sequence", memory=True
    )

    for behavior in behaviors:
        mission_sequence.add_child(behavior)

    return mission_sequence
