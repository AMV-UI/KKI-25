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
        self.dock_state = -1 # -1=REVERSE FROM BOX, 0=ALIGN GPS, 1=VISION, 3=ALIGN 90, 4=SLIDE
        self.target_yaw = 0.0
        self.locked_heading = 0.0
        self.reverse_start_time = 0.0

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
        self.initial_heading_sub = Topic.initial_heading.createSubscriber(self.node, self._initial_heading_cb)
        self.pixhawk = Topic.pixhawk.createSubscriber(self.node, self._pixhawk_cb) 
        self.detected_sub = Topic.detected.createSubscriber(self.node, self._detected_cb)
        self.dsc_sub = Topic.dsc.createSubscriber(self.node, self._dsc_cb)

        self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        self.bow_effort_pub = Topic.bow_effort.createPublisher(self.node)

        self.mission_pub = Topic.mission.createPublisher(self.node)
        self.arena_sub = Topic.arena.createSubscriber(self.node, self._arena_cb)

    def _initial_heading_cb(self, msg: Float64):
        self.locked_heading = float(msg.data)

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

        if self.dock_state == -1:
            # PHASE -1: MELEWATI BOX SEBELUM ALIGN GPS
            # Gunakan topic mission_type untuk memberi tahu kamera mencari Box
            from std_msgs.msg import String
            self.mission_pub = getattr(self, 'mission_type_pub', Topic.mission_type.createPublisher(self.node))
            self.mission_pub.publish(String(data=MissionStatus.BOTH_BOXES))
            
            speed = float(self.speed_effort)
            yaw_cmd = 0.0
            
            # Asumsi: `self.detected` akan True jika melihat Box (karena mission_type = BOTH_BOXES)
            if self.detected:
                if self.frame_counter:
                    self.frame_counter.reset()
                
                if self.dsc == 8888.0:
                    yaw_cmd = -float(self.effort) if self.arena == "A" else float(self.effort)
                elif self.dsc == 7777.0:
                    yaw_cmd = float(self.effort) if self.arena == "A" else -float(self.effort)
                else:
                    kp = getattr(MissionParams, 'kp_cam', 0.2)
                    yaw_cmd = self.dsc * kp
                    max_yaw = float(self.effort * 0.5)
                    if yaw_cmd > max_yaw: yaw_cmd = max_yaw
                    elif yaw_cmd < -max_yaw: yaw_cmd = -max_yaw
                    
                self.node.get_logger().info(f"[{self.name}] MELEWATI BOKS: dsc={self.dsc:.1f}, yaw_cmd={yaw_cmd:.1f}", throttle_duration_sec=1.0)
            else:
                self.node.get_logger().info(f"[{self.name}] BOKS HILANG! Menunggu {self.time_threshold}s...", throttle_duration_sec=1.0)
                if self.frame_counter:
                    self.frame_counter.is_started()
                    if self.frame_counter.is_enough():
                        self.node.get_logger().info(f"[{self.name}] Boks berhasil dilewati! Memulai manuver GPS...")
                        self.dock_state = 0
                        self.frame_counter.reset()
                        self.speed_effort_pub.publish(Float64(data=0.0))
                        self.yaw_effort_pub.publish(Float64(data=0.0))
                        return Status.RUNNING
                        
            self.speed_effort_pub.publish(Float64(data=speed))
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            return Status.RUNNING

        elif self.dock_state == 0:
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
            # VISION APPROACH (Forward through gates)
            if self.detected:
                # Target distance reached (detected = True from camera). Transition to Align 90 deg.
                self.node.get_logger().info(f"[{self.name}] Jarak Docking tercapai. Beralih ke perputaran 90 derajat...")
                self.dock_state = 3
                self.speed_effort_pub.publish(Float64(data=0.0))
                self.yaw_effort_pub.publish(Float64(data=0.0))
                self.bow_effort_pub.publish(Float64(data=0.0))
                return Status.RUNNING

            self.speed_effort_pub.publish(Float64(data=float(self.speed_effort)))

            # Handle hard-coded turns for "Only Blue" priority
            if self.dsc == 7777.0:
                self.node.get_logger().info(f"[{self.name}] VISION: Hanya Biru terlihat. Banting setir ke Kiri...", throttle_duration_sec=1.0)
                yaw_cmd = float(self.effort)
            elif self.dsc == 8888.0:
                self.node.get_logger().info(f"[{self.name}] VISION: Hanya Biru terlihat. Banting setir ke Kanan...", throttle_duration_sec=1.0)
                yaw_cmd = -float(self.effort)
            else:
                self.node.get_logger().info(f"[{self.name}] VISION: Berjalan menuju dock (DSC: {self.dsc:.2f})...", throttle_duration_sec=1.0)
                # Proportional steering to center of gate
                kp = getattr(MissionParams, 'kp_cam', 0.2)
                yaw_cmd = -(self.dsc * kp) # Negative because Target Left (Negative DSC) -> Needs Left Turn -> Positive Yaw
                
                align_effort = float(self.effort) * 0.8
                if yaw_cmd > align_effort: yaw_cmd = align_effort
                elif yaw_cmd < -align_effort: yaw_cmd = -align_effort

            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            return Status.RUNNING
            
        elif self.dock_state == 2:
            # REVERSE TO GPS
            if distance <= MissionParams.dock_margin_error:
                self.node.get_logger().info(f"[{self.name}] Mencapai batas margin GPS ({distance:.2f}m)! Memulai penyelarasan arah akhir...")
                self.dock_state = 3
                self.speed_effort_pub.publish(Float64(data=0.0))
                self.yaw_effort_pub.publish(Float64(data=0.0))
                self.bow_effort_pub.publish(Float64(data=0.0))
                return Status.RUNNING

            self.node.get_logger().info(f"[{self.name}] REVERSE: Mundur (Jarak: {distance:.2f}m)...", throttle_duration_sec=1.0)
            self.speed_effort_pub.publish(Float64(data=-float(self.speed_effort)))
            
            # Steering while reversing (User wants it to steer to stay in middle of Green & Red buoys if visible, else use GPS)
            if self.dsc != 9999.0 and self.dsc != 7777.0 and self.dsc != 8888.0:
                self.node.get_logger().info(f"[{self.name}] REVERSE VISION: Menyelaraskan ke tengah buoy (DSC: {self.dsc:.2f})...", throttle_duration_sec=1.0)
                kp = getattr(MissionParams, 'kp_cam', 0.2)
                # When reversing, steering logic is INVERTED!
                yaw_cmd = (self.dsc * kp)
                align_effort = float(self.effort) * 0.8
                if yaw_cmd > align_effort: yaw_cmd = align_effort
                elif yaw_cmd < -align_effort: yaw_cmd = -align_effort
            else:
                self.node.get_logger().info(f"[{self.name}] REVERSE GPS: Menyelaraskan arah via GPS (theta: {theta:.2f})...", throttle_duration_sec=1.0)
                if abs(theta) < 2.0:
                    yaw_cmd = 0.0
                else:
                    yaw_cmd = theta * 2.0 # Reversing proportional
                max_yaw = float(self.effort * 0.4) 
                if yaw_cmd > max_yaw: yaw_cmd = max_yaw
                elif yaw_cmd < -max_yaw: yaw_cmd = -max_yaw
                
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            return Status.RUNNING

        elif self.dock_state == 3:
            # FINAL ALIGNMENT TO 90 DEGREES FROM LOCKED HEADING
            from core.mission.gps_stuff import calc_turn
            target_heading = (self.locked_heading - 90.0) if self.arena == "A" else (self.locked_heading + 90.0)
            target_heading = target_heading % 360.0
            
            yaw_diff = calc_turn(target_heading, self.heading)
            
            self.node.get_logger().info(f"[{self.name}] ALIGN 90 DEG: Mengarahkan ke {target_heading:.1f} deg (diff: {yaw_diff:.1f} deg)...", throttle_duration_sec=1.0)
            
            if abs(yaw_diff) < 5.0:
                self.node.get_logger().info(f"[{self.name}] Arah sudah disesuaikan 90 derajat! Memulai SLIDING...")
                self.dock_state = 4
                self.yaw_effort_pub.publish(Float64(data=0.0))
                return Status.RUNNING
                
            yaw_cmd = float(self.effort) if yaw_diff > 0 else -float(self.effort)
            self.yaw_effort_pub.publish(Float64(data=float(yaw_cmd)))
            self.speed_effort_pub.publish(Float64(data=0.0))
            self.bow_effort_pub.publish(Float64(data=0.0))
            return Status.RUNNING

        elif self.dock_state == 4:
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
