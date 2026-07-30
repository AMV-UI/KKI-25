from py_trees.common import Status
from py_trees.behaviour import Behaviour


class Buoy_Execution(Behaviour):
    def __init__(self, name: str = "Buoy Behavior"):
        super().__init__(name)

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError("Didn't find 'node' in tree builder setup kwargs") from e

    def update(self) -> Status:
        self.node.get_logger().info(
            f"[{self.name}] track {self.arena}", throttle_duration_sec=1.0
        )

        return Status.RUNNING
