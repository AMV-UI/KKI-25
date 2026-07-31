from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Float64, UInt8, Bool
from core.utils.config import Topic, MissionParams
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
        self.time_threshold = MissionParams.dock_time_threshold
        self.target = 180
        self.hold = False
        self.dock_state = 0 # 0=WAITING_BUOYS, 1=TURNING, 2=NAVIGATING
        self.target_yaw = 0.0
        self.locked_heading = 0.0

        self.frame_counter = FrameCounter(self.time_threshold)
        self.effort = MissionParams.dock_yaw_effort
        self.heading = 0
        self.dsc = 0.0
        self.docking_lat = 0.0
        self.docking_lon = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.speed_effort = MissionParams.dock_speed_effort


    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.pixhawk = Pixhawk()

        self.docking_lat_sub = Topic.dock_lat.createSubscriber(self.node, self._docking_lat_cb)
        self.docking_lon_sub = Topic.dock_lon.createSubscriber(self.node, self._docking_lon_cb)
        self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)
        self.pixhawk = Topic.pixhawk.createSubscriber(self.node, self._pixhawk_cb) 
        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.dsc_sub = Topic.dsc.createSubscriber(self.node, self._dsc_cb)

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        self.bow_effort_pub = Topic.bow_effort.createPublisher(self.node)

        self.mission_pub = Topic.mission.createPublisher(self.node)
        self.arena_sub = Topic.arena.createSubscriber(self.node, self._arena_cb)

    def _arena_cb(self, msg):
        self.arena = str(msg.data)

    def _heading_cb(self, msg: Float64):
        self.heading = float(msg.data)

    def _dsc_cb(self, msg: Float64):
        self.dsc = float(msg.data)

    def _detected_cb(self, msg: Bool):
        self.detected = msg.data

    def _pixhawk_cb(self, msg: Pixhawk):
        self.lat = msg.lat
        self.lon = msg.lon

    def _docking_lat_cb(self, msg: Float64):
        self.docking_lat = float(msg.data)
    
    def _docking_lon_cb(self, msg: Float64):
        self.docking_lon = float(msg.data)

    def execute(self) -> Status:
        # Override for testing GPS accuracy based on initial position
        self.docking_lat = 44.51239
        self.docking_lon = 26.10777

        theta = find_deg(self.lat, self.lon, self.docking_lat, self.docking_lon, self.heading)
        distance = haversine(self.lon, self.lat, self.docking_lon, self.docking_lat)

        if self.dock_state == 0:
            # ALIGN TO GPS FIRST
            self.node.get_logger().info(f"[{self.name}] ALIGN_TO_GPS: Berputar menyamakan arah ke target GPS (theta: {theta:.2f})...", throttle_duration_sec=1.0)
            
            if abs(theta) < 10.0:
                self.node.get_logger().info(f"[{self.name}] Arah GPS sesuai! Mulai berjalan maju menuju target...")
                self.dock_state = 1
                self.speed_effort_pub.publish(Float64(data=0.0))
                self.yaw_effort_pub.publish(Float64(data=0.0))
                self.bow_effort_pub.publish(Float64(data=0.0))
                return Status.RUNNING

            if theta > 0:
                yaw_cmd = -float(self.effort)
            else:
                yaw_cmd = float(self.effort)
                
            self.speed_effort_pub.publish(Float64(data=0.0))
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            return Status.RUNNING

        elif self.dock_state == 1:
            # APPROACH
            if distance <= MissionParams.dock_margin_error:
                self.node.get_logger().info(f"[{self.name}] Mencapai batas margin GPS ({distance:.2f}m)! Memulai PUTARAN 90 DERAJAT...")
                self.dock_state = 2
                self.speed_effort_pub.publish(Float64(data=0.0))
                self.yaw_effort_pub.publish(Float64(data=0.0))
                self.bow_effort_pub.publish(Float64(data=0.0))
                
                # Set target yaw based on arena to align parallel to dock
                if self.arena == "A":
                    # Slide Left -> Dock is on Left -> Turn Right 90 degrees
                    self.target_yaw = (self.heading + 90) % 360
                else:
                    # Slide Right -> Dock is on Right -> Turn Left 90 degrees
                    self.target_yaw = (self.heading - 90 + 360) % 360
                return Status.RUNNING
                
            self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))
            
            # Combine GPS and Vision for perfect docking approach
            if self.dsc != 9999.0:
                self.node.get_logger().info(f"[{self.name}] CAMERA LOCK: Menyelaraskan kapal ke buoy docking (DSC: {self.dsc:.2f})...", throttle_duration_sec=1.0)
                # DSC > 0 means buoys are to the right. Turn right (negative yaw) to center them.
                yaw_cmd = -self.dsc * 0.3 
                max_yaw = float(self.effort)
            else:
                self.node.get_logger().info(f"[{self.name}] GPS NAVIGATE: Jarak: {distance:.2f}m, theta: {theta:.2f}. Menuju lat lon...", throttle_duration_sec=1.0)
                if abs(theta) < 2.0:
                    yaw_cmd = 0.0
                else:
                    yaw_cmd = -theta * 2.0
                max_yaw = float(self.effort * 0.4) # Limit to 40% effort for smooth GPS corrections
                    
            if yaw_cmd > max_yaw: yaw_cmd = max_yaw
            elif yaw_cmd < -max_yaw: yaw_cmd = -max_yaw
                
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            return Status.RUNNING
            
        elif self.dock_state == 2:
            # ALIGN 1 (TURN 90 DEGREES OUT)
            from core.mission.gps_stuff import calc_turn
            yaw_diff = calc_turn(self.target_yaw, self.heading)
            
            self.node.get_logger().info(f"[{self.name}] ALIGN 1: Berputar ke {self.target_yaw:.1f} deg (diff: {yaw_diff:.1f} deg)...", throttle_duration_sec=1.0)
            
            if abs(yaw_diff) < 5.0:
                self.node.get_logger().info(f"[{self.name}] Selesai putaran 1! Memulai maju 1...")
                self.dock_state = 2
                self.start_time = time.time()
                self.yaw_effort_pub.publish(Float64(data=0.0))
                return Status.RUNNING
                
            yaw_cmd = float(self.effort) if yaw_diff > 0 else -float(self.effort)
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            self.speed_effort_pub.publish(Float64(data=0.0))
            self.bow_effort_pub.publish(Float64(data=0.0))
            return Status.RUNNING

        elif self.dock_state == 3:
            # FORWARD 1
            if time.time() - self.start_time > MissionParams.dock_forward_time_1:
                self.node.get_logger().info(f"[{self.name}] Selesai maju 1! Memulai putaran 2...")
                self.dock_state = 3
                if self.arena == "A":
                    self.target_yaw = (self.heading - 90 + 360) % 360 # Turn CCW
                else:
                    self.target_yaw = (self.heading + 90) % 360 # Turn CW
                self.speed_effort_pub.publish(Float64(data=0.0))
                return Status.RUNNING
                
            self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.bow_effort_pub.publish(Float64(data=0.0))
            return Status.RUNNING

        elif self.dock_state == 4:
            # ALIGN 2 (TURN 90 DEGREES IN)
            from core.mission.gps_stuff import calc_turn
            yaw_diff = calc_turn(self.target_yaw, self.heading)
            
            self.node.get_logger().info(f"[{self.name}] ALIGN 2: Berputar ke {self.target_yaw:.1f} deg (diff: {yaw_diff:.1f} deg)...", throttle_duration_sec=1.0)
            
            if abs(yaw_diff) < 5.0:
                self.node.get_logger().info(f"[{self.name}] Selesai putaran 2! Memulai maju 2...")
                self.dock_state = 4
                self.start_time = time.time()
                self.yaw_effort_pub.publish(Float64(data=0.0))
                return Status.RUNNING
                
            yaw_cmd = float(self.effort) if yaw_diff > 0 else -float(self.effort)
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            self.speed_effort_pub.publish(Float64(data=0.0))
            self.bow_effort_pub.publish(Float64(data=0.0))
            return Status.RUNNING

        elif self.dock_state == 5:
            # FORWARD 2
            if time.time() - self.start_time > MissionParams.dock_forward_time_2:
                self.node.get_logger().info(f"[{self.name}] Selesai maju 2! Memulai putaran 3...")
                self.dock_state = 5
                if self.arena == "A":
                    self.target_yaw = (self.heading - 90 + 360) % 360 # Turn CCW
                else:
                    self.target_yaw = (self.heading + 90) % 360 # Turn CW
                self.speed_effort_pub.publish(Float64(data=0.0))
                return Status.RUNNING
                
            self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.bow_effort_pub.publish(Float64(data=0.0))
            return Status.RUNNING

        elif self.dock_state == 6:
            # ALIGN 3 (TURN 90 DEGREES FINAL)
            from core.mission.gps_stuff import calc_turn
            yaw_diff = calc_turn(self.target_yaw, self.heading)
            
            self.node.get_logger().info(f"[{self.name}] ALIGN 3: Berputar ke {self.target_yaw:.1f} deg (diff: {yaw_diff:.1f} deg)...", throttle_duration_sec=1.0)
            
            if abs(yaw_diff) < 5.0:
                self.node.get_logger().info(f"[{self.name}] Selesai putaran 3! Memulai SLIDING...")
                self.dock_state = 7
                self.yaw_effort_pub.publish(Float64(data=0.0))
                return Status.RUNNING
                
            yaw_cmd = float(self.effort) if yaw_diff > 0 else -float(self.effort)
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            self.speed_effort_pub.publish(Float64(data=0.0))
            self.bow_effort_pub.publish(Float64(data=0.0))
            return Status.RUNNING

        elif self.dock_state == 7:
            # SLIDING
            if self.arena == "A":
                # Slide Left (Surge Port CCW, Surge Starboard CW)
                speed_cmd = -float(self.effort * 2.0)
                bow_cmd = float(self.effort * 2.0)
            else:
                # Slide Right (Surge Port CW, Surge Starboard CCW)
                speed_cmd = float(self.effort * 2.0)
                bow_cmd = -float(self.effort * 2.0)
                
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.speed_effort_pub.publish(Float64(data=speed_cmd))
            self.bow_effort_pub.publish(Float64(data=bow_cmd))
            
            self.node.get_logger().info(f"[{self.name}] SLIDING: Sliding to {'Left' if self.arena == 'A' else 'Right'} into docking bay...", throttle_duration_sec=1.0)
            return Status.RUNNING
        
        self.node.get_logger().info(f"[{self.name}] distance: {distance:.2f}m, theta: {theta:.2f} deg", throttle_duration_sec=1.0)
        
        if abs(theta) < 15:
            self.node.get_logger().info(f"[{self.name}] Arah sesuai, kapal berjalan maju...", throttle_duration_sec=1.0)
            speed = float(self.speed_effort)
            
            # Smooth proportional steering while moving forward
            if abs(theta) < 2.0:
                yaw_cmd = 0.0
            else:
                yaw_cmd = -theta * 2.0
                max_yaw = float(self.effort * 0.4)
                if yaw_cmd > max_yaw: yaw_cmd = max_yaw
                elif yaw_cmd < -max_yaw: yaw_cmd = -max_yaw
        else:
            self.node.get_logger().info(f"[{self.name}] Berputar menyamakan arah ke target...", throttle_duration_sec=1.0)
            speed = 0.0
            
            # Pivot constantly if off-angle
            if theta > 0:
                yaw_cmd = -float(self.effort)
            else:
                yaw_cmd = float(self.effort)

        self.speed_effort_pub.publish(Float64(data=speed))
        self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))

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
