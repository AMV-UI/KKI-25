from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64, UInt8, String
from core.utils.config import Topic
from core.mission.frame_counter import FrameCounter
from core_msgs.msg import Pixhawk
from core.utils.config import PxMode


import time
class Initial_Execution(BaseExecution):
    """
    Main execution: Set inital heading and finding the buoy
    - Fallback: if pxmode is still on hold
    """
    def __init__(self, name, node=None):
        super().__init__(name, node=node)
        self.node = node

        self.arena = "B"
        self.detected = False
        self.time_threshold = 1
        self.target = 180
        self.hold = False

        self.frame_counter = FrameCounter(self.time_threshold)
        self.effort = 200.0 
        self.heading = 0

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.pixhawk = Pixhawk()
        
        self.initial_heading_pub = Topic.initial_heading.createPublisher(self.node)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)

        self.arena_sub = Topic.arena.createSubscriber(self.node, self._arena_cb)
        self.pxmode_sub = Topic.pxmode.createSubscriber(self.node, self._pxmode_cb)
        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)

    def calc_dsc(self):
        kanan = (abs(self.target - self.heading) + 360) % 360
        kiri = (abs(self.heading - self.target) + 360) % 360

        if kanan <= kiri:
            return 1
        else:
            return -1


    def _pxmode_cb(self, msg: String):
        if msg.data != PxMode.AUTO:
            self.hold = True
        else:
            self.hold = False

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)

    def _heading_cb(self, msg: Float64):
        self.px_heading = float(msg.data)

    def _detected_cb(self, msg: Bool):
        self.detected = bool(msg.data)

    def execute(self) -> Status:
        self.node.get_logger().info(f"[{self.name}] Initial EXECUTION mode active...", throttle_duration_sec=1.0)

        if self.hold:
            return Status.FAILURE
        
        if self.detected:
            self.initial_heading_pub.publish(UInt8(data=self.heading))
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                self.node.get_logger().info(f"[{self.name}] Target found -> switching to mission")
                return Status.SUCCESS
            self.yaw_effort_pub.publish(Float64(data=0.0))        
            return Status.RUNNING
        else:
            self.frame_counter.reset()

        self.yaw_effort_pub.publish(Float64(data=self.effort * self.calc_dsc()))        
        return Status.RUNNING


class Initial_Fallback(BaseFallback):
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
        self.record = False

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.docking_lat_pub = Topic.dock_lat.createPublisher(self.node)
        self.docking_lon_pub = Topic.dock_lon.createPublisher(self.node)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)


        self.rc5_sub = Topic.rc5.createSubscriber(self.node, self._rc5_cb)
        self.pxmode_sub = Topic.pxmode.createSubscriber(self.node, self._pxmode_cb)
        self.pixhawk_sub = Topic.pixhawk.createSubscriber(self.node, self._pixhawk_cb)
    
    
    def _rc5_cb(self, msg: UInt8):
        if 1301 <= msg.data:
            self.record = True
        else:
            self.record = False

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
        if self.gps_ready and self.record:
            self.set_docking_coordinates()

        if not self.hold:
            return Status.FAILURE

        return Status.RUNNING
