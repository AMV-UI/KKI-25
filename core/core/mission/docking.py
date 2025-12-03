import math
from ..utils.config import Topic
from rclpy.node import Node

class DockingController:

    def __init__(self, target_lat=None, target_lon=None):
        """
        Modular docking controller using PID control for autonomous navigation.
        Works with effort values in range ±300 (matching ROS system).
        
        Parameters:
        target_lat (float, optional): Target latitude for docking
        target_lon (float, optional): Target longitude for docking
        """
        self.target_lat = target_lat
        self.target_lon = target_lon
        
        #initialize PID gains
        self.Kp_yaw = 90.0
        self.Ki_yaw = 0.0
        self.Kd_yaw = 0.0
        
        self.Kp_speed = 100.0  # Proportional gain for speed (distance-based)
        self.min_speed = 50.0  # Minimum speed effort when moving
        self.max_speed = 300.0 # Maximum speed effort
        
        self.yaw_error = 0.0
        self.yaw_integral = 0.0
        self.prev_yaw_error = 0.0
        self.max_integral = 6.0
        
        self.MAX_EFFORT = 300.0
        self.MIN_EFFORT = -300.0
        
        # CHANGED: Reduced threshold from 30.0 to 1.0 meter to prevent premature docking completion
        self.docking_distance_threshold = 1.0  # meters - consider docked when closer
        self.alignment_threshold = 0.3         # radians (~17 degrees) - move forward when aligned
        
        self.is_docked = False
        
        self.recorded_movements = []  # List of (yaw_effort, speed_effort, dt) tuples
        self.recorded_lat_lon = []  # List of (lat, lon, dt) tuples
        self.is_recording = False # For movements
        self.is_recording_lat_lon = False # For lat/lon
        self.initial_position = None  # (lat, lon, heading) when recording starts
        self.recording_start_time = 0.0

        self.waypoint_threshold = 0.2  # meters - minimum distance between recorded points
        self.accumulated_dt = 0.0

        self.rc6 = 0.0
        self.dsc = 0.0
        
        self.rc6_pub = Topic.rc6.createPublisher(Node("docking_controller_node"))
        self.rc6_sub = Topic.rc6.createSubscriber(Node("docking_controller_node"), self._rc6_callback)
        self.dsc_sub = Topic.dsc.createSubscriber(Node("docking_controller_node"), self._dsc_callback)

    def _rc6_callback(self, msg):
        """Callback to update RC6 value from incoming messages."""
        self.rc6 = float(msg.data)

    def _dsc_callback(self, msg):
        """Callback to update DSC value from incoming messages."""
        self.dsc = float(msg.data)

    def update_pid_gains(self, Kp_yaw, Ki_yaw, Kd_yaw):
        """
        Update PID gains for yaw control.
        
        Parameters:
        Kp_yaw (float): Proportional gain
        Ki_yaw (float): Integral gain
        Kd_yaw (float): Derivative gain
        """
        self.Kp_yaw = Kp_yaw
        self.Ki_yaw = Ki_yaw
        self.Kd_yaw = Kd_yaw
    
    def get_current_error(self):
        """
        Get the current yaw error in radians.
        
        Returns:
        float: Current yaw error
        """
        return self.yaw_error
        
    def calculate_bearing_rad(self, lat1, lon1, lat2, lon2):
        """
        Calculate the bearing from current position to target position.
        Uses PIXHAWK convention: 0 = North, clockwise (0-360°).
        
        Parameters:
        lat1, lon1 (float): Starting position in decimal degrees
        lat2, lon2 (float): Target position in decimal degrees
        
        Returns:
        float: Bearing in radians using Pixhawk convention (0 = North, clockwise), range [0, 2π]
        """

        delta_lat = lat2 - lat1
        delta_lon = lon2 - lon1
        
        lat_avg = math.radians((lat1 + lat2) / 2)
        
        dx_east = delta_lon * math.cos(lat_avg)   # East-West component
        dy_north = delta_lat                      # North-South component
        
        # Calculate bearing using atan2(East, North) for Pixhawk convention
        # atan2(x, y) gives: North=0, East=90°, South=180°, West=270° (clockwise)
        bearing_rad = math.atan2(dx_east, dy_north)
        
        # Normalize to [0, 2π] for Pixhawk convention
        if bearing_rad < 0:
            bearing_rad += 2 * math.pi
        
        return bearing_rad
    
    def calculate_bearing_deg(self, lat1, lon1, lat2, lon2):
        """
        Calculate the bearing from current position to target position.
        Returns bearing in degrees (0-360).
        """
        bearing_rad = self.calculate_bearing_rad(lat1, lon1, lat2, lon2)
        bearing_deg = math.degrees(bearing_rad)
        bearing_deg = (bearing_deg + 360) % 360  # Normalize to 0-360
        return bearing_deg
    
    def normalize_angle_rad(self, angle):
        """
        Normalize angle to range [-π, π].
        
        Parameters:
        angle (float): Angle in radians
        
        Returns:
        float: Normalized angle in radians
        """
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def normalize_angle_deg(self, angle):
        """
        Normalize angle to range [-180, 180] degrees.
        
        Parameters:
        angle (float): Angle in degrees
        
        Returns:
        float: Normalized angle in degrees
        """
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def set_target(self, target_lat, target_lon):
        """
        Set or update the docking target position.
        
        Parameters:
        target_lat (float): Target latitude
        target_lon (float): Target longitude
        """
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.reset_controller()
    
    def reset_controller(self):
        """Reset PID controller state variables."""
        self.yaw_error = 0.0
        self.yaw_integral = 0.0
        self.prev_yaw_error = 0.0
        self.is_docked = False
    
    

    def start_docking(self):
        """
        Start autonomous docking mode.
        Resets the controller state for a fresh docking approach.
        
        Returns:
        bool: True if docking started (target is set)
        """
        if self.target_lat is None or self.target_lon is None:
            return False
        
        self.reset_controller()
        return True

    def stop_recording(self):
        """
        Stop recording movements and lat/lon logging.
        
        Returns:
        bool: True if recording stopped successfully, write the waypoints to txt file
        """
        self.is_recording_lat_lon = False 
        
        with open("waypoints.txt", "w") as f:
            for waypoint in self.recorded_movements:
                f.write(f"{waypoint}\n")
        return True
    
    def stop_and_save_playback(self):
        """
        Stop docking and save the current state.
        Can be used to pause docking or prepare for playback.
        
        Returns:
        dict: Current state information
        """
        state = {
            'is_docked': self.is_docked,
            'recorded_movements': len(self.recorded_movements),
            'recorded_lat_lon': len(self.recorded_lat_lon),
            'target': (self.target_lat, self.target_lon)
        }
        return state

    def execute_playback_latlon(self, current_lat, current_lon, current_heading_deg, dt):

        if not hasattr(self, "playback_latlon_index"):
            self.playback_latlon_index = 0

        if self.playback_latlon_index >= len(self.recorded_lat_lon):
            return 0.0, 0.0, True
        
        
        while self.playback_latlon_index < len(self.recorded_lat_lon):
            self.target_lat, self.target_lon, recorded_dt, rc6 = self.recorded_lat_lon[self.playback_latlon_index]
            distance = self.get_distance_to_target(current_lat, current_lon)

            yaw_effort, speed_effort, is_at_target = self.calculate_control_efforts(
                current_lat,
                current_lon,
                current_heading_deg,
                recorded_dt
            )

            if self.dsc != 0.0:
                return self.dsc, speed_effort, False
            
            self.rc6_pub.publish(rc6)

            if distance < self.waypoint_threshold and is_at_target:
                self.playback_latlon_index += 1
                self.reset_controller()
            else:
                break

        return yaw_effort, speed_effort, False

    
    def get_adaptive_pid_gains(self, error):
        """
        Adapt PID gains based on current yaw error magnitude.
        You can tune the scaling factors as needed.
        """
        base_kp = self.Kp_yaw
        base_ki = self.Ki_yaw
        base_kd = self.Kd_yaw

        # Example: Increase Kp and Kd with error, keep Ki constant
        kp = base_kp + 40.0 * abs(error)    # scale as needed
        ki = base_ki
        kd = base_kd + 5.0 * abs(error)     # scale as needed

        return kp, ki, kd

    def calculate_control_efforts(self, current_lat, current_lon, current_heading_deg, dt):
        """
        Compute yaw + speed efforts for autonomous navigation/docking.
        - Yaw effort responds only to heading error
        - Speed effort depends on distance AND heading alignment
        """

        # --- No target → do nothing ---
        if self.target_lat is None or self.target_lon is None:
            return 0.0, 0.0, False

        # --- Compute distance to target ---
        distance = self.get_distance_to_target(current_lat, current_lon)

        # --- Docking threshold check ---
        if distance < self.docking_distance_threshold:
            self.is_docked = True
            self.yaw_integral = 0.0
            return 0.0, 0.0, True

        # --- Convert heading to radians ---
        current_heading_rad = math.radians(current_heading_deg)

        # --- Desired bearing to waypoint (Pixhawk convention) ---
        desired_heading = self.calculate_bearing_rad(
            current_lat, current_lon,
            self.target_lat, self.target_lon
        )

        # --- Compute shortest angular error (-π to +π) ---
        raw_error = desired_heading - current_heading_rad
        self.yaw_error = math.atan2(math.sin(raw_error), math.cos(raw_error))

        # --- If <2°, treat as aligned ---
        if abs(self.yaw_error) < math.radians(2):
            self.yaw_error = 0.0

        # --- Adaptive PID YAW ---
        kp, ki, kd = self.get_adaptive_pid_gains(self.yaw_error)

        # integral
        self.yaw_integral += self.yaw_error * dt
        self.yaw_integral = max(-self.max_integral, min(self.max_integral, self.yaw_integral))

        # derivative
        derivative = (self.yaw_error - self.prev_yaw_error) / dt if dt > 0 else 0.0

        # PID output
        yaw_pid = (kp * self.yaw_error +
                   ki * self.yaw_integral +
                   kd * derivative)

        yaw_effort = max(self.MIN_EFFORT, min(self.MAX_EFFORT, yaw_pid))
        self.prev_yaw_error = self.yaw_error

        yaw_align = max(0.0, 1.0 - abs(self.yaw_error) / math.pi)
        raw_speed = distance * self.Kp_speed
        raw_speed = max(self.min_speed, min(self.max_speed, raw_speed))
        speed_effort = raw_speed * yaw_align
        if speed_effort > 0 and speed_effort < self.min_speed:
            speed_effort = self.min_speed

        if self.is_recording_lat_lon:
            self.record_lat_lon(current_lat, current_lon, dt, self.rc6)

        return yaw_effort, speed_effort, False

    
    def get_distance_to_target(self, current_lat, current_lon):
        """
        Calculate distance to target in meters using Haversine formula.
        
        Parameters:
        current_lat (float): Current latitude in decimal degrees
        current_lon (float): Current longitude in decimal degrees
        
        Returns:
        float: Distance to target in meters
        """
        if self.target_lat is None or self.target_lon is None:
            return float('inf')
        
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(current_lat)
        lat2_rad = math.radians(self.target_lat)
        delta_lat = math.radians(self.target_lat - current_lat)
        delta_lon = math.radians(self.target_lon - current_lon)
        
        a = math.sin(delta_lat/2)**2 + \
            math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        distance = R * c
        return distance
    
    def get_lat_lon_duration(self):
        """
        Get the duration of latitude and longitude recording.
        
        Returns:
        float: Duration of recording in seconds
        """
        return self.accumulated_dt
    
    def get_heading_error(self):
        """
        Get the current heading error in radians.
        Useful for monitoring alignment during docking.
        
        Returns:
        float: Current heading error in radians
        """
        return self.yaw_error
    
    def get_heading_error_deg(self):
        """
        Get the current heading error in degrees.
        
        Returns:
        float: Current heading error in degrees
        """
        return math.degrees(self.yaw_error)
    
    def start_lat_lon_recording(self, current_lat, current_lon, current_heading):
        """
        Start recording latitude and longitude.
        
        Parameters:
        current_lat (float): Starting latitude
        current_lon (float): Starting longitude
        current_heading (float): Starting heading in degrees (Pixhawk: 0=North, clockwise)
        """
        self.is_recording_lat_lon = True
        self.recorded_lat_lon = []
        self.initial_position = (current_lat, current_lon, current_heading)
        self.recording_start_time = 0.0
        self.accumulated_dt = 0.0
        return True
    
    def record_lat_lon(self, lat, lon, dt, rc6):
        """
        Record a single latitude and longitude during recording.
        Ignores points closer than error_coordinate_threshold meters to the last recorded point.
        
        Parameters:
        lat (float): Current latitude
        lon (float): Current longitude
        dt (float): Time delta since last frame
        rc6 (float): Current RC6 value
        
        Returns:
        bool: True if recorded, False if not recording or ignored
        """
        if not self.is_recording_lat_lon:
            return False
        
        if self.target_lat is None or self.target_lon is None:
            return False
            
        self.accumulated_dt += dt
            
        if self.recorded_lat_lon:
            last_lat, last_lon, *_ = self.recorded_lat_lon[-1]
            distance = self.calculate_distance(lat, lon, last_lat, last_lon)
            if distance < self.waypoint_threshold:
                return False
        
        self.recorded_lat_lon.append((lat, lon, self.accumulated_dt, rc6))
        self.recording_start_time += self.accumulated_dt
        self.accumulated_dt = 0.0
        return True

    def get_recorded_lat_lon(self):
        """
        Get the list of recorded latitude and longitude.
        
        Returns:
        list: List of (lat, lon, dt, rc6) tuples
        """
        return self.recorded_lat_lon.copy()

    def stop_lat_lon_recording(self):
        """
        Stop recording latitude and longitude.
        
        Returns:
        int: Number of recorded latitude and longitude frames
        """
        self.is_recording_lat_lon = False
        with open("waypoints.txt", "w") as f:
            for waypoint in self.recorded_lat_lon:
                f.write(f"{waypoint}\n")
        return len(self.recorded_lat_lon)
    
    def record_movement(self, yaw_effort, speed_effort, dt):
        """
        Record a single movement frame during recording.
        
        Parameters:
        yaw_effort (float): Current yaw effort
        speed_effort (float): Current speed effort
        dt (float): Time delta since last frame
        
        Returns:
        bool: True if recorded, False if not recording
        """
        if not self.is_recording:
            return False
        
        # Fixed logic: Record all movements, don't skip based on threshold
        self.recorded_movements.append((yaw_effort, speed_effort, dt))
        self.recording_start_time += dt
        return True

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """
        Calculate distance between two points in meters using Haversine formula.
        
        Parameters:
        lat1, lon1 (float): First point
        lat2, lon2 (float): Second point
        
        Returns:
        float: Distance in meters
        """
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = math.sin(delta_lat/2)**2 + \
            math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c

    def get_initial_position(self):
        """
        Get the initial position when recording started.
        
        Returns:
        tuple: (lat, lon, heading_deg) or None if no recording has been made
               heading_deg is in Pixhawk convention (0=North, clockwise)
        """
        return self.initial_position
    
    def get_recorded_movements(self):
        """
        Get the list of recorded movements.
        
        Returns:
        list: List of (yaw_effort, speed_effort, dt) tuples
        """
        return self.recorded_movements.copy()

    def get_recorded_lat_lon(self):
        """
        Get the list of recorded latitude and longitude.
        
        Returns:
        list: List of (lat, lon, dt) tuples
        """
        return self.recorded_lat_lon.copy()
    
    def has_recording(self):
        """
        Check if there is a recorded movement sequence.
        
        Returns:
        bool: True if movements have been recorded
        """
        return len(self.recorded_movements) > 0
    
    def has_lat_lon(self):
        """
        Check if there is a recorded latitude and longitude sequence.
        
        Returns:
        bool: True if latitude and longitude have been recorded
        """
        return len(self.recorded_lat_lon) > 0
    
    def clear_recording(self):
        """Clear the recorded movements."""
        self.recorded_movements = []
        self.recorded_lat_lon = []
        self.initial_position = None
        self.recording_start_time = 0.0
        self.is_recording = False
        self.is_recording_lat_lon = False
    
    def get_recording_duration(self):
        """
        Get the total duration of the recorded movement.
        
        Returns:
        float: Total duration in seconds
        """
        return sum(frame[2] for frame in self.recorded_movements)

    def get_lat_lon_duration(self):
        """
        Get the total duration of the recorded latitude and longitude.
        
        Returns:
        float: Total duration in seconds
        """
        return sum(frame[2] for frame in self.recorded_lat_lon)
