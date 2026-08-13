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
    def __init__(self, name, node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node

        self.arena = "B"
        self.detected = False
        self.time_threshold = 0.2
        self.target = 126
        self.hold = False

        self.mission = mission
        self.frame_counter = FrameCounter(self.time_threshold)
        self.effort = 120.0 
        self.heading = 0

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.pixhawk = Pixhawk()
        
        self.initial_heading_pub = Topic.initial_heading.createPublisher(self.node)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)

        self.mission_pub = Topic.mission.createPublisher(self.node)

        self.arena_sub = Topic.arena.createSubscriber(self.node, self._arena_cb)
        self.pxmode_sub = Topic.pxmode.createSubscriber(self.node, self._pxmode_cb)
        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)
    
    def calc_dsc(self):
        dsc = self.target - self.heading
        dsc = dsc if abs(dsc) <= 180 else (360 - abs(dsc)) * (-1 if dsc > 0 else 1)
        self.node.get_logger().info(f"[{self.name}] DSC Calculation: Target {self.target} - Heading {self.heading} = DSC {dsc}", throttle_duration_sec=1.0)    
        return 1 if dsc > 0 else -1

    def _pxmode_cb(self, msg: String):
        if msg.data != PxMode.AUTO:
            self.hold = True
        else:
            self.hold = False

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)

    def _heading_cb(self, msg: Float64):
        self.heading = float(msg.data)

    def _detected_cb(self, msg: Bool):
        self.detected = bool(msg.data)

    def execute(self) -> Status:
        self.node.get_logger().info(f"[{self.name}] Initial EXECUTION mode active...", throttle_duration_sec=1.0)

        if self.hold:
            return Status.FAILURE
        
        if self.detected:
            self.initial_heading_pub.publish(Float64(data=float(self.heading)))
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                self.node.get_logger().info(f"[{self.name}] Target found -> switching to mission")
                self.mission_pub.publish(UInt8(data=1))
                return Status.SUCCESS
            self.yaw_effort_pub.publish(Float64(data=0.0))        
            return Status.RUNNING
        else:
            self.frame_counter.reset()

        self.node.get_logger().info(f"[{self.name}] value {self.calc_dsc()}...", throttle_duration_sec=1.0)
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

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.docking_lat_pub = Topic.dock_lat.createPublisher(self.node)
        self.docking_lon_pub = Topic.dock_lon.createPublisher(self.node)
        self.initial_heading_pub = Topic.initial_heading.createPublisher(self.node)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)

        self.pxmode_sub = Topic.pxmode.createSubscriber(self.node, self._pxmode_cb)
        self.pixhawk_sub = Topic.pixhawk.createSubscriber(self.node, self._pixhawk_cb)
        self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)
        
        self.heading = 0.0

    def _heading_cb(self, msg: Float64):
        self.heading = float(msg.data)

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
        self.initial_heading_pub.publish(Float64(data=self.heading))
        self.node.get_logger().info(f"[{self.name}] Saving Docking Location on {self.pixhawk.lat}, {self.pixhawk.lon} with Heading {self.heading}")

    def fallback(self) -> Status:        
        self.node.get_logger().info(f"[{self.name}] Initial Fallback mode active..", throttle_duration_sec=1.0)
        self.yaw_effort_pub.publish(Float64(data=0.0))        
        if self.gps_ready:
            self.set_docking_coordinates()

        if not self.hold:
            return Status.FAILURE

        return Status.RUNNING
