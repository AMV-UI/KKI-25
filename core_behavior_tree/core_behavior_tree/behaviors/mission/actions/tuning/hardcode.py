from re import split
from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
import py_trees

from std_msgs.msg import Bool, Float64, UInt8, String, UInt32
from core.utils.config import Topic
from core.mission.gps_stuff import turner
from core.mission.find_mode import FindMode
from core.mission.frame_counter import FrameCounter
from core.mission.gps_stuff import haversine
from core_msgs.msg import Pixhawk
from core.utils.config import Param, PxMode
import time



"""
FALLBACK
1. If channel 5 on, reset coordinate.txt then record current GPS coordinate every time step (2 sec)

EXECUTE
2. If channel 5 off, stop recording and return success
3. coordinate.txt and navigate each coordinate
4. Make margin error 1 meter

"""


class RecordGPSExecution(BaseExecution):
    """
    If mode is auto, return success, else keep running and storing lat / lot data each time step
    """
    def __init__(self, name: str = "RecordExecution", node=None):
        super().__init__(name, node=node)
        self.node = node

    def initialise(self):
        self.px_mode = PxMode.HOLD
        self.effort_st = 100.0
        self.effort_tn = 100.0
        self.position_idx = 0
        self.margin_error = 1.0
        self.turn_error = 10.0
        self.playback = False
        self.blackboard_client = py_trees.blackboard.Client(name="RecordGPSExecutionBBClient")
        self.blackboard_client.register_key(key="tuning_coordinate_file_position", access=py_trees.common.Access.READ)
        return super().initialise()

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        
        self.effort_st_sub = Topic.tuning_effort_st.createSubscriber(
            self.node,
            self._effort_st_cb
        )

        self.effort_tn_sub = Topic.tuning_effort_tn.createSubscriber(
            self.node,
            self._effort_tn_cb
        )   

        self.px_sub =  Topic.pixhawk.createSubscriber(
            self.node,
            self._px_cb
        )

        self.px_mode_sub = Topic.pxmode.createSubscriber(
            self.node,
            self._px_mode_cb
        )

        self.rc5_sub = Topic.rc5.createSubscriber(
            self.node,
            self._rc5_cb
        )

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)

    def _effort_st_cb(self, msg: Float64):
        self.effort_st = float(msg.data)       

    def _effort_tn_cb(self, msg: Float64):
        self.effort_tn = float(msg.data)       

    def _px_mode_cb(self, msg: UInt8):
        self.px_mode = msg.data

    def _rc5_cb(self, msg: UInt8):
        if 1301 <= msg.data <= 1700:
            self.playback = True
        else:
            self.playback = False

    def _px_cb(self, msg: Pixhawk):
        self.lat = msg.lat
        self.lon = msg.lon
        self.heading = msg.msg_heading

    # Assume line never empty for simplicity
    def continue_file_line(self):
        line: str = ""
        with open("core_behavior_tree/core_behavior_tree/behaviors/mission/actions/tuning/log/coordinate.txt", "r") as coor_file:
            coor_file.seek(self.file_position)
            line = coor_file.readline().strip().split(",")
            if len(line) != 2:
                self.node.get_logger().info("[{}] CEEEEEEEEEEk")
                self.next_coor = float(0.0), float(0.0)
            else:
                self.node.get_logger().info("[{}] CEEEEEEEEEEk {} {} {}".format(self.name, line[0], line[1], self.file_position))
                self.next_coor = float(line[0]), float(line[1])
                self.file_position = coor_file.tell()

        

    def execute(self) -> Status:
        self.node.get_logger().info("[{}] Execution... {}".format(self.name, self.playback), throttle_duration_sec=1.0)

        if (not self.playback):
            return Status.FAILURE

        if self.px_mode != PxMode.AUTO:
            return Status.RUNNING
        
        if (self.next_coor == ""):
            return Status.SUCCESS
        
        self.node.get_logger().info("[{}] Playing back GPS coordinates... Next Coordinate: {}".format(self.name, self.next_coor), throttle_duration_sec=1.0)
        if self.next_coor != (0.0, 0.0):
            theta = turner((self.lon, self.lat), self.heading, self.next_coor)       
            if theta > self.turn_error:
                self.yaw_effort_pub.publish(Float64(data=self.effort_tn))
            elif theta < -self.turn_error:
                self.yaw_effort_pub.publish(Float64(data=-self.effort_tn))
            else:
                self.yaw_effort_pub.publish(Float64(data=0.0))

            if (haversine(self.lon, self.lat, self.next_coor[0], self.next_coor[1]) < self.margin_error):
                self.continue_file_line()
        
            self.speed_effort_pub.publish(Float64(data=self.effort_st))
        else:
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.speed_effort_pub.publish(Float64(data=0.0))
            self.continue_file_line()
            
        return Status.RUNNING

class RecordGPSFallback(BaseFallback):
    """
    If mode is auto, return success, else keep running and storing lat / lot data each time step
    """
    def __init__(self, name: str = "RecordFallback", node=None):
        super().__init__(name, node=node)
        self.node = node

    def initialise(self):
        self.time_step = 2.0  # seconds
        self.last_time = time.time()
        self.playback = False
        self.lat = 0
        self.lon = 0
        self.recording = False

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        
        self.px_sub =  Topic.pixhawk.createSubscriber(
            self.node,
            self._px_cb
        )

        self.rc5_sub = Topic.rc5.createSubscriber(
            self.node,
            self._rc5_cb
        )

        self.rc6_sub = Topic.rc6.createSubscriber(
            self.node,
            self._rc6_cb
        )

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)

    def _rc5_cb(self, msg: UInt8):
        if 1301 <= msg.data <= 1700:
            self.playback = True
        else:
            self.playback = False

    def _rc6_cb(self, msg: UInt8):
        if 1301 <= msg.data <= 1700 and not self.recording:
            open("core_behavior_tree/core_behavior_tree/behaviors/mission/actions/tuning/log/coordinate.txt", "w").close()  # reset file
            self.recording = True
        else:
            self.recording = False

    def _px_cb(self, msg: Pixhawk):
        self.lat = msg.lat
        self.lon = msg.lon

        if (self.recording):
            # self.last_time = time.time()
            with open("core_behavior_tree/core_behavior_tree/behaviors/mission/actions/tuning/log/coordinate.txt", "a") as coor_file:
                coor_file.write("{},{}\n".format(self.lon, self.lat))

    def fallback(self):        
        self.node.get_logger().info("[{}] Failure...".format(self.name), throttle_duration_sec=1.0)

        if (self.playback):
            return Status.FAILURE
        

        return Status.RUNNING
