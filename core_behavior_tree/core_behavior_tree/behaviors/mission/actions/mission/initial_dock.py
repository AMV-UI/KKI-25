from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64, UInt8, String
from core.utils.config import Topic
from core.mission.frame_counter import FrameCounter
from core_msgs.msg import Pixhawk
from core.utils.config import PxMode


import time
class InitialDock_Execution(BaseExecution):
    """
    Main execution: Set inital heading and finding the buoy
    - Fallback: if pxmode is still on hold
    """
    def __init__(self, name, node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node
        self.hold = True

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.pxmode_sub = Topic.pxmode.createSubscriber(self.node, self._pxmode_cb)
    
    def _pxmode_cb(self, msg: String):
        if msg.data != PxMode.AUTO:
            self.hold = True
        else:
            self.hold = False

    def execute(self) -> Status:
        self.node.get_logger().info(f"[{self.name}] Initial EXECUTION mode active...", throttle_duration_sec=1.0)

        if self.hold:
            return Status.FAILURE

        self.node.get_logger().info(f"[{self.name}] Done execution...", throttle_duration_sec=1.0)
        return Status.SUCCESS


class InitialDock_Fallback(BaseFallback):
    """
    Fallback: Remote still on hold and updating docking position
    - Execution: if remote change other than hold
    """
    def __init__(self, name, node=None):
        super().__init__(name, node=node)
        self.node = node
        self.pixhawk = Pixhawk()
        self.gps_ready = False
        self.hold = True

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.docking_lat_pub = Topic.dock_lat.createPublisher(self.node)
        self.docking_lon_pub = Topic.dock_lon.createPublisher(self.node)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)

        self.pxmode_sub = Topic.pxmode.createSubscriber(self.node, self._pxmode_cb)
        self.pixhawk_sub = Topic.pixhawk.createSubscriber(self.node, self._pixhawk_cb)
    
    def _pxmode_cb(self, msg: String):
        if msg.data != PxMode.AUTO:
            self.hold = True
        else:
            self.hold = False

    def _pixhawk_cb(self, msg: Pixhawk):
        self.pixhawk = msg
        if msg.lat != 0.0 and msg.lon != 0.0:
            self.gps_ready = True

    def set_docking_coordinates(self):
        """Save Pixhawk coordinates for docking mission"""
        self.docking_lat_pub.publish(Float64(data=self.pixhawk.lat))
        self.docking_lon_pub.publish(Float64(data=self.pixhawk.lon))
        self.node.get_logger().info(f"[{self.name}] Saving Docking Location on {self.pixhawk.lat}, {self.pixhawk.lon} {self.pixhawk.lat}")

    def fallback(self) -> Status:        
        self.node.get_logger().info(f"[{self.name}] Initial Fallback mode active..", throttle_duration_sec=1.0)
        self.yaw_effort_pub.publish(Float64(data=0.0))        
        if self.gps_ready:
            self.set_docking_coordinates()

        if not self.hold:
            return Status.FAILURE

        return Status.RUNNING
