from ...mission_behaviors import BaseExecution, BaseFallback
from pyproj import Geod
from py_trees.common import Status
from std_msgs.msg import Float64, UInt8
from core.utils.config import Topic
from core.mission.frame_counter import FrameCounter
from core_msgs.msg import Pixhawk
from core.mission.gps_stuff import haversine, find_deg

import time
class Docking_Execution(BaseExecution):
    """
    Main execution: Set inital heading and finding the buoy
    - Fallback: if pxmode is still on hold
    """
    def __init__(self, name, node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node

        self.mission = mission
        self.arena = "B"
        self.detected = False
        self.time_threshold = 0.2
        self.target = 180
        self.hold = False

        self.frame_counter = FrameCounter(self.time_threshold)
        self.effort = 200
        self.heading = 0
        self.docking_lat = 0.0
        self.docking_lon = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.speed_effort = 100.0


    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.pixhawk = Pixhawk()
        self.geodesic = Geod(ellps='WGS84')

        self.docking_lat_sub = Topic.dock_lat.createSubscriber(self.node, self._docking_lat_cb)
        self.docking_lon_sub = Topic.dock_lon.createSubscriber(self.node, self._docking_lon_cb)
        self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)
        self.pixhawk = Topic.pixhawk.createSubscriber(self.node, self._pixhawk_cb) 

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)

        self.mission_pub = Topic.mission.createPublisher(self.node)

    def _heading_cb(self, msg: Float64):
        self.heading = float(msg.data)

    def _pixhawk_cb(self, msg: Pixhawk):
        self.lat = msg.lat
        self.lon = msg.lon

    def _docking_lat_cb(self, msg: Float64):
        self.docking_lat = float(msg.data)
    
    def _docking_lon_cb(self, msg: Float64):
        self.docking_lon = float(msg.data)

    def execute(self) -> Status:
        self.node.get_logger().info(f"[{self.name}] Initial Dock mode Lat: {self.docking_lat} Lon: {self.docking_lon}", throttle_duration_sec=1.0)

        if haversine(self.lon, self.lat, self.docking_lon, self.docking_lat) < 1.0:
            self.node.get_logger().info(f"[{self.name}] Arrived at docking station", throttle_duration_sec=5.0)
            self.speed_effort_pub.publish(Float64(data=0.0))
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.mission_pub.publish(UInt8(data=self.mission))
            return Status.SUCCESS
        
        theta = find_deg(self.lat, self.lon, self.docking_lat, self.docking_lon, self.heading, self.geodesic)         
        self.node.get_logger().info(f"[{self.name}] test: {theta}", throttle_duration_sec=1.0)
        self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))

        if theta > 90:
            self.effort = 250
        elif theta > 45:
            self.effort = 200
        elif theta > 10:
            self.effort = 150
        else:
            self.effort = 120

        self.yaw_effort_pub.publish(Float64(data=float((self.effort) * (1 if theta > 0 else -1))))

        return Status.RUNNING

class Docking_Fallback(BaseFallback):
    """
    """
    def __init__(self, name, node=None):
        super().__init__(name, node=node)
       
    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        

    def fallback(self) -> Status:        
        return Status.FAILURE
