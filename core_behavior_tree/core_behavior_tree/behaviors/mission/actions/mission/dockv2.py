from ...mission_behaviors import BaseExecution, BaseFallback
from py_trees.common import Status
from std_msgs.msg import Float64, UInt8, Bool, String
from core.utils.config import Topic, MissionParams, MissionStatus
from core.mission.frame_counter import FrameCounter
from core_msgs.msg import Pixhawk
from core.mission.gps_stuff import calc_turn

import time

class DockingV2_Execution(BaseExecution):
    """
    Docking V2 without GPS:
    - Phase 0: Pass the boxes (BOTH_BOXES)
    - Phase 1: Sweeping for blue buoy (DOCKING_V2)
    - Phase 2: Vision centering blue buoy
    - Phase 3: Turn 90 degrees
    - Phase 4: Sliding mode
    """
    def __init__(self, name, node=None, mission=None):
        super().__init__(name, node=node)
        self.node = node

        self.mission = mission
        self.arena = "B"
        self.detected = False
        self.dock_state = 0 # 0=EVADE, 1=SWEEP, 2=CENTER, 3=TURN, 4=SLIDE
        self.target_heading = 0.0

        self.effort = MissionParams.dock_yaw_effort
        self.heading = 0.0
        self.dsc = 0.0 
        self.blue_area = 0.0
        self.speed_effort = MissionParams.dock_speed_effort
        
        self.time_threshold = 2.0
        self.frame_counter = None
        self.start_time = 0.0
        self.sweep_direction = 1

        self.dock_lat = 0.0
        self.dock_lon = 0.0
        self.pixhawk = Pixhawk()
        
    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)
        self.frame_counter = FrameCounter(self.time_threshold)

        self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)
        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.dsc_sub = Topic.dsc.createSubscriber(self.node, self._dsc_cb)
        self.blue_area_sub = Topic.blue_area.createSubscriber(self.node, self._blue_area_cb)
        
        self.box_detected_sub = Topic.box_detected.createSubscriber(self.node, self._box_detected_cb)
        
        self.dock_lat_sub = Topic.dock_lat.createSubscriber(self.node, self._dock_lat_cb)
        self.dock_lon_sub = Topic.dock_lon.createSubscriber(self.node, self._dock_lon_cb)
        self.pixhawk_sub = Topic.pixhawk.createSubscriber(self.node, self._pixhawk_cb)

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        self.bow_effort_pub = Topic.bow_effort.createPublisher(self.node)
        self.mission_type_pub = Topic.mission_type.createPublisher(self.node)

        self.arena_sub = Topic.arena.createSubscriber(self.node, self._arena_cb)

    def _arena_cb(self, msg):
        self.arena = str(msg.data)

    def _heading_cb(self, msg: Float64):
        self.heading = float(msg.data)

    def _dsc_cb(self, msg: Float64):
        self.dsc = float(msg.data)
        
    def _blue_area_cb(self, msg: Float64):
        self.blue_area = float(msg.data)

    def _detected_cb(self, msg: Bool):
        self.detected = msg.data
        
    def _box_detected_cb(self, msg: Bool):
        self.box_detected = bool(msg.data)
        
    def _dock_lat_cb(self, msg: Float64):
        self.dock_lat = float(msg.data)
        
    def _dock_lon_cb(self, msg: Float64):
        self.dock_lon = float(msg.data)
        
    def _pixhawk_cb(self, msg: Pixhawk):
        self.pixhawk = msg

    def initialise(self) -> None:
        self.dock_state = 0
        if self.frame_counter:
            self.frame_counter.reset()
        self.node.get_logger().info(f"[{self.name}] Initializing Docking V2 (GPS Guided)")

    def execute(self) -> Status:
        
        if self.dock_state == 0:
            # PHASE 0: NAVIGATE TO INITIAL GPS WHILE LOOKING FOR BLUE BUOY
            self.mission_type_pub.publish(String(data=MissionStatus.DOCKING_V2))
            
            # 1. Check if blue buoy detected (from vision)
            if self.detected and self.blue_area > 0:
                if self.frame_counter:
                    self.frame_counter.is_started()
                    if self.frame_counter.is_enough():
                        self.node.get_logger().info(f"[{self.name}] Blue Buoy terdeteksi saat menuju GPS! Beralih ke CENTERING...")
                        self.dock_state = 2
                        self.frame_counter.reset()
                        return Status.RUNNING
            else:
                if self.frame_counter:
                    self.frame_counter.reset()

            # 2. Navigate to GPS
            # If gps is not valid, fallback to sweeping immediately
            if self.dock_lat == 0.0 or self.dock_lon == 0.0 or self.pixhawk.lat == 0.0 or self.pixhawk.lon == 0.0:
                self.node.get_logger().info(f"[{self.name}] Data GPS awal tidak valid, langsung beralih ke SWEEPING...")
                self.dock_state = 1
                self.start_time = time.time()
                self.sweep_direction = 1
                return Status.RUNNING

            from core.mission.gps_stuff import haversine, find_deg
            dist = haversine(self.pixhawk.lon, self.pixhawk.lat, self.dock_lon, self.dock_lat)
            yaw_diff = find_deg(self.pixhawk.lat, self.pixhawk.lon, self.dock_lat, self.dock_lon, self.heading)

            self.node.get_logger().info(f"[{self.name}] Menuju GPS Awal: Jarak {dist:.1f}m, Yaw Diff {yaw_diff:.1f}", throttle_duration_sec=1.0)

            if dist < 4.0: # Reached within 4 meters
                self.node.get_logger().info(f"[{self.name}] Telah sampai di sekitar titik GPS awal! Beralih ke SWEEPING mencari Dock...")
                self.dock_state = 1
                self.start_time = time.time()
                self.sweep_direction = 1
                return Status.RUNNING

            # Proportional steering to waypoint
            kp_gps = getattr(MissionParams, 'kp_gps', 2.0)
            yaw_cmd = yaw_diff * kp_gps
            
            max_yaw = float(self.effort)
            if yaw_cmd > max_yaw: yaw_cmd = max_yaw
            elif yaw_cmd < -max_yaw: yaw_cmd = -max_yaw

            self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            return Status.RUNNING

        elif self.dock_state == 1:
            # PHASE 1: SWEEP BLUE BUOY (ZIGZAG)
            self.mission_type_pub.publish(String(data=MissionStatus.DOCKING_V2))
            
            if self.detected and self.blue_area > 0:
                if self.frame_counter:
                    self.frame_counter.is_started()
                    if self.frame_counter.is_enough():
                        self.node.get_logger().info(f"[{self.name}] Blue Buoy stabil terdeteksi! Beralih ke CENTERING...")
                        self.dock_state = 2
                        self.frame_counter.reset()
                        return Status.RUNNING
            else:
                if self.frame_counter:
                    self.frame_counter.reset()
            
            duration = getattr(MissionParams, 'turn_next_buoy_duration', 4.0)
            elapsed = time.time() - self.start_time
            if elapsed >= duration:
                self.node.get_logger().info(f"[{self.name}] Sweeping time up. Reversing direction.")
                self.sweep_direction *= -1
                self.start_time = time.time()
                elapsed = 0.0
                
            speed = float(self.speed_effort)
            base_yaw = float(self.effort) if self.arena == "A" else -float(self.effort)
            yaw_cmd = base_yaw * self.sweep_direction
            
            self.speed_effort_pub.publish(Float64(data=speed))
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            self.node.get_logger().info(f"[{self.name}] SWEEPING (Dir: {self.sweep_direction})... elapsed: {elapsed:.1f}s", throttle_duration_sec=1.0)
            return Status.RUNNING
            
        elif self.dock_state == 2:
            # PHASE 2: VISION CENTERING BLUE BUOY
            self.mission_type_pub.publish(String(data=MissionStatus.DOCKING_V2))
            
            self.node.get_logger().info(f"[{self.name}] CENTERING: Mengikuti blue buoy (menunggu jarak < 1.5m)...", throttle_duration_sec=1.0)
            
            if self.detected:
                self.node.get_logger().info(f"[{self.name}] Jarak Blue Buoy tercapai (<1.5m)! Memulai putaran 90 derajat...")
                self.dock_state = 3
                
                # Kalkulasi target heading (Arena A = Kiri / -90, Arena B = Kanan / +90)
                if self.arena == "A":
                    self.target_heading = (self.heading - 90.0) % 360.0
                else:
                    self.target_heading = (self.heading + 90.0) % 360.0
                    
                self.speed_effort_pub.publish(Float64(data=0.0))
                self.yaw_effort_pub.publish(Float64(data=0.0))
                self.bow_effort_pub.publish(Float64(data=0.0))
                return Status.RUNNING

            speed = float(self.speed_effort)
            yaw_cmd = 0.0
            
            if self.detected and self.blue_area > 0:
                kp = getattr(MissionParams, 'kp_cam', 0.2)
                yaw_cmd = -(self.dsc * kp) # self.dsc adalah (mid_x - width). Positif = kanan = belok kanan (+). Koreksi: butuh minus agar belok ke arah target.
                
                max_yaw = float(self.effort)
                if yaw_cmd > max_yaw: 
                    yaw_cmd = max_yaw
                elif yaw_cmd < -max_yaw: 
                    yaw_cmd = -max_yaw
            else:
                self.node.get_logger().info(f"[{self.name}] Blue buoy hilang sejenak, maju perlahan...", throttle_duration_sec=1.0)

            self.speed_effort_pub.publish(Float64(data=speed))
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            return Status.RUNNING

        elif self.dock_state == 3:
            # PHASE 3: TURNING 90 DEGREES
            yaw_diff = calc_turn(self.target_heading, self.heading)
            self.node.get_logger().info(f"[{self.name}] TURNING 90: Heading sekarang {self.heading:.1f}, Target {self.target_heading:.1f} (diff: {yaw_diff:.1f})...", throttle_duration_sec=1.0)
            
            if abs(yaw_diff) < 5.0:
                self.node.get_logger().info(f"[{self.name}] Putaran selesai! Memulai SLIDING...")
                self.dock_state = 4
                self.speed_effort_pub.publish(Float64(data=0.0))
                self.yaw_effort_pub.publish(Float64(data=0.0))
                self.bow_effort_pub.publish(Float64(data=0.0))
                return Status.RUNNING
                
            yaw_cmd = float(self.effort) if yaw_diff > 0 else -float(self.effort)
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            self.speed_effort_pub.publish(Float64(data=0.0))
            self.bow_effort_pub.publish(Float64(data=0.0))
            return Status.RUNNING

        elif self.dock_state == 4:
            # PHASE 4: SLIDING
            if self.arena == "A":
                speed_cmd = float(self.effort * 2.0)
                bow_cmd = -float(self.effort * 2.0)
            else:
                speed_cmd = -float(self.effort * 2.0)
                bow_cmd = float(self.effort * 2.0)
                
            self.yaw_effort_pub.publish(Float64(data=0.0))
            self.speed_effort_pub.publish(Float64(data=speed_cmd))
            self.bow_effort_pub.publish(Float64(data=bow_cmd))
            
            self.node.get_logger().info(f"[{self.name}] SLIDING: Sliding to {'Right' if self.arena == 'A' else 'Left'} into docking bay...", throttle_duration_sec=1.0)
            return Status.RUNNING

        return Status.RUNNING


class DockingV2_Fallback(BaseFallback):
    def __init__(self, name, node=None):
        super().__init__(name, node=node)
       
    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)

    def fallback(self) -> Status:        
        return Status.FAILURE
