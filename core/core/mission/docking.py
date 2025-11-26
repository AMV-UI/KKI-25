import math

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
        
        self.Kp_yaw = 200.0    # Proportional gain for yaw (increased for ±300 range)
        self.Ki_yaw = 1.0      # Integral gain for yaw
        self.Kd_yaw = 50.0     # Derivative gain for yaw
        
        self.Kp_speed = 100.0  # Proportional gain for speed (distance-based)
        self.min_speed = 50.0  # Minimum speed effort when moving
        self.max_speed = 200.0 # Maximum speed effort
        
        self.yaw_error = 0.0
        self.yaw_integral = 0.0
        self.prev_yaw_error = 0.0
        
        self.MAX_EFFORT = 300.0
        self.MIN_EFFORT = -300.0
        
        self.docking_distance_threshold = 1.5  # meters - consider docked when closer
        self.alignment_threshold = 0.3         # radians (~17 degrees) - move forward when aligned
        
        self.is_docked = False
        
        self.recorded_movements = []  # List of (yaw_effort, speed_effort, dt) tuples
        self.recorded_lat_lon = []  # List of (lat, lon, dt) tuples
        self.is_recording = False # For movements
        self.is_recording_lat_lon = False # For lat/lon
        self.initial_position = None  # (lat, lon, heading) when recording starts
        self.recording_start_time = 0.0


        self.error_coordinate_threshold = 5.0  # meters - minimum distance between recorded points
        self.accumulated_dt = 0.0
        
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
    
    def calculate_control_efforts(self, current_lat, current_lon, current_heading_deg, dt):
        """
        Calculate yaw and speed efforts for autonomous docking.
        
        Convention (PIXHAWK SYSTEM):
        - Yaw: POSITIVE = turn RIGHT (CW), NEGATIVE = turn LEFT (CCW)
        - Speed: POSITIVE = forward, NEGATIVE = backward
        - Heading: 0 = North, 90 = East, 180 = South, 270 = West (CLOCKWISE)
        
        Parameters:
        current_lat (float): Current latitude in decimal degrees
        current_lon (float): Current longitude in decimal degrees
        current_heading_deg (float): Current heading in DEGREES (Pixhawk: 0=North, clockwise)
        dt (float): Time step since last calculation in seconds
        
        Returns:
        tuple: (yaw_effort, speed_effort, is_docked)
            - yaw_effort (float): Yaw control effort in range ±300 (+ = right, - = left)
            - speed_effort (float): Speed control effort in range ±300 (+ = forward, - = backward)
            - is_docked (bool): True if within docking threshold
        """
        if self.target_lat is None or self.target_lon is None:
            return 0.0, 0.0, False
        
        distance = self.get_distance_to_target(current_lat, current_lon)

        if distance < self.docking_distance_threshold:
            self.is_docked = True
            return 0.0, 0.0, True
        
        # Convert current heading to radians (Pixhawk convention: 0=N, clockwise)
        current_heading_rad = math.radians(current_heading_deg)
        
        # Calculate desired bearing (in radians, Pixhawk convention, range [0, 2π])
        desired_heading = self.calculate_bearing_rad(
            current_lat, current_lon,
            self.target_lat, self.target_lon
        )
        
        # Calculate heading error (normalized to -π to π)
        # For Pixhawk clockwise convention:
        # Positive error = need to turn clockwise (right)
        # Negative error = need to turn counter-clockwise (left)
        error = desired_heading - current_heading_rad
        # Normalize to shortest rotation (-π to π)
        self.yaw_error = math.atan2(math.sin(error), math.cos(error))
        
        # PID calculations for yaw
        self.yaw_integral += self.yaw_error * dt
        
        # Anti-windup: limit integral term
        max_integral = 30.0
        self.yaw_integral = max(-max_integral, min(max_integral, self.yaw_integral))
        
        derivative = (self.yaw_error - self.prev_yaw_error) / dt if dt > 0 else 0.0
        
        yaw_pid_output = (self.Kp_yaw * self.yaw_error + 
                          self.Ki_yaw * self.yaw_integral + 
                          self.Kd_yaw * derivative)
        
        # Convert to yaw effort (clamp to ±300)
        # In Pixhawk convention with clockwise positive:
        # Positive error = need to turn RIGHT (clockwise) = positive yaw effort
        # Negative error = need to turn LEFT (counter-clockwise) = negative yaw effort
        # So NO inversion needed - signs match!
        yaw_effort = max(self.MIN_EFFORT, min(self.MAX_EFFORT, yaw_pid_output))
        
        # Calculate speed effort based on distance and alignment
        if abs(self.yaw_error) < self.alignment_threshold:
            # Well aligned - use distance-based speed
            speed_effort = min(self.max_speed, max(self.min_speed, distance * self.Kp_speed))
        else:
            # Not aligned - slow down or stop
            speed_effort = 0.0
        
        self.prev_yaw_error = self.yaw_error
        
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
    
    def start_recording(self, current_lat, current_lon, current_heading):
        """
        Start recording manual movements.
        
        Parameters:
        current_lat (float): Starting latitude
        current_lon (float): Starting longitude
        current_heading (float): Starting heading in degrees (Pixhawk: 0=North, clockwise)
        """
        self.is_recording = True
        self.recorded_movements = []
        self.initial_position = (current_lat, current_lon, current_heading)
        self.recording_start_time = 0.0
        return True

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
    
    def stop_recording(self):
        """
        Stop recording movements.
        
        Returns:
        int: Number of recorded movement frames
        """
        self.is_recording = False
        return len(self.recorded_movements)

    def stop_lat_lon_recording(self):
        """
        Stop recording latitude and longitude.
        
        Returns:
        int: Number of recorded latitude and longitude frames
        """
        self.is_recording_lat_lon = False
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
        
        if yaw_effort or speed_effort < self.error_coordinate_threshold:
            pass
        
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

    def record_lat_lon(self, lat, lon, dt):
        """
        Record a single latitude and longitude during recording.
        Ignores points closer than 5 meters to the last recorded point.
        
        Parameters:
        lat (float): Current latitude
        lon (float): Current longitude
        dt (float): Time delta since last frame
        
        Returns:
        bool: True if recorded, False if not recording or ignored
        """
        if not self.is_recording_lat_lon:
            return False
        
        if self.target_lat is None or self.target_lon is None:
            return False
            
        self.accumulated_dt += dt
            
        if self.recorded_lat_lon:
            last_lat, last_lon, _ = self.recorded_lat_lon[-1]
            distance = self.calculate_distance(lat, lon, last_lat, last_lon)
            if distance < self.error_coordinate_threshold:
                return False
        
        self.recorded_lat_lon.append((lat, lon, self.accumulated_dt))
        self.recording_start_time += self.accumulated_dt
        self.accumulated_dt = 0.0
        return True
    
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