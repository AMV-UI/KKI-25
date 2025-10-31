from ..mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64
from core.utils.config import Topic
from core.mission.find_mode import FindMode
from core.mission.frame_counter import FrameCounter
from core.mission.docking import Docking
from core_msgs.msg import Pixhawk
from core.utils.config import Param

class Mission1_Execution(BaseExecution):
    """
    Main execution: APPROACH to 2 Buoys (Different color, Green and Red) phase only
    - Navigate toward detected target using DSC from vision
    - When target is lost for N frames -> SUCCESS (mission complete)
    - If target not detected -> FAILURE (triggers fallback to search)
    - Store Pixhawk coordinate to docking station after to use in the last mission
    """
    def __init__(self, name: str = "Mission1_Execution"):
        super().__init__(name)
        self.frame_counter = None
        self.detected = False
        self.dsc = 0.0
        self.speed_effort = 300.0
        self.pixhawk = None
        Param.DOCKING_LAT.createParam(self.node, default_value=0.0)
        Param.DOCKING_LON.createParam(self.node, default_value=0.0)
        self.coordinate_saved_state = False
        self.gps_ready = False
        self.time_threshold = 2 # in Seconds
        

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.frame_counter = FrameCounter(self.time_threshold)
        self.pixhawk = Pixhawk()

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
        self.pixhawk_sub = Topic.pixhawk.createSubscriber(self.node, self._pixhawk_cb)
        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.dsc_sub = Topic.dsc.createSubscriber(self.node, self._dsc_cb)

    def set_docking_coordinates(self):
        """Save Pixhawk coordinates for docking mission"""
        Param.DOCKING_LAT.setParam(self.node, self.pixhawk.latitude)
        Param.DOCKING_LON.setParam(self.node, self.pixhawk.longitude)

        
    def _pixhawk_cb(self, msg: Pixhawk):
        self.pixhawk = msg
        if msg.latitude != 0.0 and msg.longitude != 0.0:
            self.gps_ready = True

    def _detected_cb(self, msg: Bool):
        self.detected = bool(msg.data)

    def _dsc_cb(self, msg: Float64):
        self.dsc = float(msg.data)

    def execute(self) -> Status:
        lat_ok = abs(self.pixhawk.latitude) > 0.1
        lon_ok = abs(self.pixhawk.longitude) > 0.1
        
        if self.gps_ready and lat_ok and lon_ok:
            if not self.coordinate_saved_state:
                self.set_docking_coordinates()
                self.coordinate_saved_state = True
                self.node.get_logger().info(f"[{self.name}] Docking coordinates saved: LAT {self.pixhawk.latitude}, LON {self.pixhawk.longitude}")

            if not self.detected:
                self.frame_counter.is_started()
                if self.frame_counter.is_enough():
                    self.frame_counter.reset()
                    self.node.get_logger().info(
                        f"[{self.name}] Lost target consistently -> Mission complete",
                        throttle_duration_sec=1.0
                    )
                    return Status.SUCCESS
                
                return Status.FAILURE

            self.frame_counter.reset()

            self.yaw_effort_pub.publish(Float64(data=self.dsc))
            self.speed_effort_pub.publish(Float64(data=self.speed_effort))
            
            self.node.get_logger().info(
                f"[{self.name}] Approaching target - DSC: {self.dsc}",
                throttle_duration_sec=2.0
            )
        
        return Status.RUNNING


class Mission1_Fallback(BaseFallback):
    """
    Fallback: FINDING phase
    - Search for target using constant yaw + find_mode
    - When target found consistently -> returns SUCCESS (lets execution take over)
    """
    def __init__(self, name: str = "Mission1_Fallback"):
        super().__init__(name)
        self.find_mode = None
        self.frame_counter = None
        self.detected = False
        self.px_heading = 0.0
        self.arena = "B"  # or "A"
        self.dsc = 160.0 if self.arena == "A" else -160.0
        self.speed_effort = 300.0
        self.time_threshold = 2 # in Seconds

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.find_mode = FindMode(self.node)
        self.frame_counter = FrameCounter(self.time_threshold)

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)

        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)

    def _detected_cb(self, msg: Bool):
        self.detected = bool(msg.data)

    def _heading_cb(self, msg: Float64):
        self.px_heading = float(msg.data)

    def fallback(self) -> Status:
        self.yaw_effort_pub.publish(Float64(data=self.dsc))
        self.speed_effort_pub.publish(Float64(data=self.speed_effort))
        
        if self.detected:
            self.frame_counter.is_started()
            if self.frame_counter.is_enough():
                self.frame_counter.reset()
                self.node.get_logger().info(f"[{self.name}] Target found -> switching to execution")
                return Status.SUCCESS
        else:
            self.frame_counter.reset()
            self.find_mode.set_initial_heading(self.px_heading)
        
        self.node.get_logger().info(
            f"[{self.name}] Searching for target (detected: {self.detected})",
            throttle_duration_sec=5.0
        )
        
        return Status.RUNNING
