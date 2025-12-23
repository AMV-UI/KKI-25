from core.mission.gps_stuff import haversine
from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Bool, Float64, UInt8, String, UInt32
from core.utils.config import Topic
from core.mission.find_mode import FindMode
from core.mission.frame_counter import FrameCounter
from core_msgs.msg import Pixhawk
from core.utils.config import Param, PxMode


import time
class RecordStraightExecution(BaseExecution):
    """
    Main execution: Go to specific coordinates in Lat/Lon 
    """
    def __init__(self, name: str = "Straight_Execution", node=None):
        super().__init__(name, node=node)
        self.node = node
        self.success = False
        self.effort = 0.0
        self.duration = 3.0
        self.recording = False
        self.effort_yaw = 0.0
        self.start_lat = 0.0
        self.start_lon = 0.0
        self.start_heading = 0.0

    def initialise(self):
        self.effort = 0.0
        self.success = False
        self.effort_yaw = 0.0

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        
        self.mission_sub = Topic.tuning_mission.createSubscriber(
            self.node,
            self._mission_cb
        )        
        self.effort_sub = Topic.tuning_effort_st.createSubscriber(
            self.node,
            self._effort_cb
        )        

        self.effort_yaw_sub = Topic.tuning_effort_tn.createSubscriber(
            self.node,
            self._effort_tn_cb
        )        

        self.px_sub = Topic.pixhawk.createSubscriber(self.node, self._px_cb)
        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)

    def _px_cb(self, msg: Pixhawk):
        self.lat = msg.lat
        self.lon = msg.lon
        self.heading = msg.msg_heading

    def _mission_cb(self, msg: UInt8):
        if(msg.data == 1):
            self.success = True

    def _effort_cb(self, msg: Float64):
        if self.recording:
            self.node.get_logger().info("[{}] Already recording...".format(self.name))
        else:
            self.recording = True
            self.start_lat = self.lat
            self.start_lon = self.lon
            self.start_time = time.time()
            self.effort = float(msg.data)
    
    def _effort_tn_cb(self, msg: Float64):
        if self.recording:
            self.node.get_logger().info("[{}] Already recording...".format(self.name))
        else:
            self.recording = True
            self.start_heading = self.heading
            self.start_time = time.time()
            self.effort_yaw = float(msg.data)

    def execute(self) -> Status:
        self.node.get_logger().info(f"[{self.name}] Tuning")
        if(self.success):
            return Status.SUCCESS

        if self.recording:
            if (time.time() - self.start_time) >= self.duration:
                self.recording = False
                distance = haversine(self.start_lon, self.start_lat, self.lon, self.lat)
                speed = distance / self.duration  # meters per second
                angular_speed = (self.heading - self.start_heading) / self.duration # degrees per second
                if (self.effort_yaw) > 0.0:
                    log = f"[{time.time()}] Start at heading: {self.start_heading}, Effort: {self.effort_yaw}, end at heading: {self.heading}, Heading change: {self.heading - self.start_heading} degrees, Angular Speed: {angular_speed} deg/s"
                    with open("core_behavior_tree/core_behavior_tree/behaviors/mission/actions/tuning/log/rec.txt", "a") as file:
                        file.write(f"{log}\n")
                    self.node.get_logger().info(log)
                if (self.effort) > 0.0:
                    log = f"[{time.time()}] Start at Lat: {self.start_lat} Lon: {self.start_lon}, Effort: {self.effort}, end at Lat: {self.lat} Lon: {self.lon}, distance: {distance} meters, Speed: {speed} m/s"
                    with open("core_behavior_tree/core_behavior_tree/behaviors/mission/actions/tuning/log/rec.txt", "a") as file:
                        file.write(f"{log}\n")
                    self.node.get_logger().info(log)
            else:
                self.yaw_effort_pub.publish(Float64(data=self.effort_yaw))
                self.speed_effort_pub.publish(Float64(data=self.effort))
        else:
            self.node.get_logger().info(f"[{self.name}] Not recording")
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.speed_effort_pub.publish(Float64(data=0.0))

        return Status.RUNNING

