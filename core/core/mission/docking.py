import math

class Docking:
    def __init__(self, current_lat, current_lon, target_lat, target_lon):
        """
        PID docking station using coordinate from Pixhawk
        Parameters:
        current_lat (float): Current latitude, subscribed from Pixhawk
        current_lon (float): Current longitude, subscribed from Pixhawk
        target_lat (float): Target latitude, taken in first mission
        target_lon (float): Target longitude, taken in first mission
        """
        self.current_lat = current_lat
        self.current_lon = current_lon
        self.target_lat = target_lat
        self.target_lon = target_lon
        
        # PID parameters - tune these values
        self.Kp = 5.0  # Proportional gain
        self.Ki = 0.1  # Integral gain
        self.Kd = 2.0  # Derivative gain
        
        # PID state variables
        self.error = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        
        # Effort constraints
        self.IDLE_EFFORT = 1500
        self.MAX_EFFORT = 1800
        self.MIN_EFFORT = 1200
        
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """
        Calculate the bearing from current position to target position
        Returns bearing in degrees (0-360)
        """
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)
        
        # Calculate bearing
        x = math.sin(delta_lon) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
            math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
        
        bearing = math.atan2(x, y)
        bearing = math.degrees(bearing)
        bearing = (bearing + 360) % 360  # Normalize to 0-360
        
        return bearing
    
    def normalize_angle(self, angle):
        """
        Normalize angle to range [-180, 180]
        """
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def calculate_yaw_effort(self, current_heading, dt):
        """
        Calculate the yaw effort using PID controller
        Parameters:
        current_heading (float): Current heading from Pixhawk in degrees (0-360)
        dt (float): Time step since last calculation in seconds
        
        Returns:
        int: Yaw effort value (1100-1900, where 1500 is idle)
        """
        # Calculate desired bearing to target
        desired_heading = self.calculate_bearing(
            self.current_lat, self.current_lon,
            self.target_lat, self.target_lon
        )
        
        # Calculate heading error (normalized to -180 to 180)
        self.error = self.normalize_angle(desired_heading - current_heading)
        
        # PID calculations
        self.integral += self.error * dt
        
        # Anti-windup: limit integral term
        max_integral = 50
        self.integral = max(-max_integral, min(max_integral, self.integral))
        
        derivative = (self.error - self.prev_error) / dt if dt > 0 else 0
        
        pid_output = (self.Kp * self.error + 
                      self.Ki * self.integral + 
                      self.Kd * derivative)
        
        # Convert PID output to yaw effort
        # Positive error means turn right, negative means turn left
        yaw_effort = self.IDLE_EFFORT + int(pid_output)
        yaw_effort = max(self.MIN_EFFORT, min(self.MAX_EFFORT, yaw_effort))

        self.prev_error = self.error
        
        return yaw_effort
    
    def update_position(self, current_lat, current_lon):
        """
        Update current position from Pixhawk
        """
        self.current_lat = current_lat
        self.current_lon = current_lon
    
    def get_distance_to_target(self):
        """
        Calculate distance to target in meters using Haversine formula
        """
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(self.current_lat)
        lat2_rad = math.radians(self.target_lat)
        delta_lat = math.radians(self.target_lat - self.current_lat)
        delta_lon = math.radians(self.target_lon - self.current_lon)
        
        a = math.sin(delta_lat/2)**2 + \
            math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        distance = R * c
        return distance