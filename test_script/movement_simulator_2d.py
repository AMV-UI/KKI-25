#!/usr/bin/env python3
"""
2D Movement Simulator with Docking Visualization and Movement Recording

Controls:
- W/S: Forward/Backward speed control (+ forward, - backward)
- A/D: Left/Right yaw control (A = - left, D = + right)
- E: Set docking init point / Go to docking point / Reset docking
- T: Record/Playback toggle
  * First press: Start recording movements
  * Second press: Stop recording and play back from initial position
  * Third press: Stop playback
- R: Reset/Respawn simulation
- Space: Stop movement
- Q: Quit

Conventions (PIXHAWK SYSTEM):
- Yaw effort: POSITIVE = turn RIGHT (CW), NEGATIVE = turn LEFT (CCW)
- Speed effort: POSITIVE = forward, NEGATIVE = backward
- Heading: 0 = North, 90 = East, 180 = South, 270 = West (CLOCKWISE)
- Internal display uses mathematical convention but reports Pixhawk heading

The simulator visualizes:
- Vehicle position (lat/lon as X/Y coordinates)
- Vehicle heading (orientation)
- Docking init point (red target when set)
- Initial recording position (green crosshair during playback)
- Path history (blue trail)
- Current yaw and speed efforts (bars)
- Recording indicator (blinking REC when recording)
"""

import pygame
import math
import sys
import random
from enum import Enum

# Import the modularized docking controller
sys.path.insert(0, '/home/amv/KKI-25')
from docking import DockingController

# Initialize Pygame
pygame.init()

# Constants
WINDOW_WIDTH = 2000
WINDOW_HEIGHT = 1200
FPS = 60

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 100, 255)
YELLOW = (255, 255, 0)
ORANGE = (255, 165, 0)
CYAN = (0, 255, 255)
GRAY = (128, 128, 128)
DARK_GRAY = (64, 64, 64)

# Physics constants (matching ROS system)
MAX_SPEED_EFFORT = 300.0  # Maximum effort value (matching ROS)
MAX_YAW_EFFORT = 300.0    # Maximum effort value (matching ROS)
SPEED_INCREMENT = 10.0    # Delta effort per key press (can go up to ±100)
YAW_INCREMENT = 10.0      # Delta effort per key press (can go up to ±100)
SPEED_TO_VELOCITY = 0.005 # Conversion factor (tuned for ±300 range)
YAW_TO_ANGULAR = 0.0002    # Conversion factor - reduced for less sensitive turning

# Coordinate conversion (pixels to lat/lon simulation)
PIXELS_TO_METERS = 0.1
METERS_TO_LATLON = 0.00001  # Approximate conversion

# Simulation realism constants
GPS_UPDATE_RATE = 500  # Hz (5-10 Hz typical)
GPS_POSITION_NOISE = 0.5  # meters (±2-5m typical)
GPS_HEADING_NOISE = 10  # degrees (±5-10° typical)
CURRENT_STRENGTH = 0.03  # m/s (0.5-2 m/s typical)
CURRENT_DIRECTION = 45.0  # degrees (can be changed)
COMM_DELAY = 0.01  # seconds (50ms typical MAVLink delay)


class DockingState(Enum):
    IDLE = 0
    INIT_POINT_SET = 1
    GOING_TO_DOCK = 2
    DOCKED = 3


class RecordingState(Enum):
    IDLE = 0
    RECORDING = 1
    RETURNING_TO_START = 2  # Navigating back to initial recording point
    PLAYING_BACK = 3


class Vehicle:
    def __init__(self, x, y):
        self.x = x  # Screen position (true position)
        self.y = y
        self.heading = 0.0  # Radians, 0 = right, positive = counterclockwise (true heading)
        self.speed_effort = 0.0
        self.yaw_effort = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.angular_velocity = 0.0
        self.path_history = []
        self.max_history = 500
        
        # GPS simulation (noisy readings)
        self.gps_x = x
        self.gps_y = y
        self.gps_heading = 0.0
        self.gps_update_timer = 0.0
        self.gps_update_interval = 1.0 / GPS_UPDATE_RATE
        
        # Environmental effects
        self.current_x = CURRENT_STRENGTH * math.cos(math.radians(CURRENT_DIRECTION)) / PIXELS_TO_METERS
        self.current_y = -CURRENT_STRENGTH * math.sin(math.radians(CURRENT_DIRECTION)) / PIXELS_TO_METERS
        
        # Communication delay buffer (store commands with timestamps)
        self.command_buffer = []
        
    def get_lat_lon(self, use_gps=True):
        """Convert screen coordinates to simulated lat/lon"""
        # Use noisy GPS position if requested, otherwise true position
        x = self.gps_x if use_gps else self.x
        y = self.gps_y if use_gps else self.y
        lat = (WINDOW_HEIGHT / 2 - y) * PIXELS_TO_METERS * METERS_TO_LATLON
        lon = (x - WINDOW_WIDTH / 2) * PIXELS_TO_METERS * METERS_TO_LATLON
        return lat, lon
    
    def update_gps(self, dt):
        """Update GPS readings with realistic noise and update rate"""
        self.gps_update_timer += dt
        
        if self.gps_update_timer >= self.gps_update_interval:
            self.gps_update_timer = 0.0
            
            # Add position noise (Gaussian distribution)
            noise_distance = random.gauss(0, GPS_POSITION_NOISE / 3)  # 3-sigma rule
            noise_angle = random.uniform(0, 2 * math.pi)
            noise_x = noise_distance * math.cos(noise_angle) / PIXELS_TO_METERS
            noise_y = noise_distance * math.sin(noise_angle) / PIXELS_TO_METERS
            
            self.gps_x = self.x + noise_x
            self.gps_y = self.y + noise_y
            
            # Add heading noise
            heading_noise = random.gauss(0, GPS_HEADING_NOISE / 3)  # degrees
            self.gps_heading = self.heading + math.radians(heading_noise)
    
    def get_heading_pixhawk(self, use_gps=True):
        """
        Get heading in Pixhawk convention (degrees).
        Internal: 0 = East (right), π/2 = North (up), CCW positive (radians)
        Pixhawk: 0 = North, 90 = East, 180 = South, 270 = West (degrees, clockwise)
        
        Parameters:
        use_gps: If True, return noisy GPS heading; if False, return true heading
        
        Returns:
        float: Heading in degrees (Pixhawk convention: 0=North, clockwise)
        """
        # Use noisy GPS heading if requested, otherwise true heading
        heading = self.gps_heading if use_gps else self.heading
        
        # Convert from mathematical convention (radians, East=0, CCW)
        # to Pixhawk convention (degrees, North=0, CW)
        
        # Step 1: Convert radians to degrees
        heading_deg = math.degrees(heading)
        
        # Step 2: Convert East=0 to North=0 and flip direction (CCW to CW)
        # Math: East=0, North=90 (CCW)
        # Pixhawk: North=0, East=90 (CW)
        # Formula: pixhawk = (90 - math) mod 360
        pixhawk_heading = (90.0 - heading_deg) % 360.0
        
        return pixhawk_heading
    
    def update(self, dt, apply_decay=True):
        """Update vehicle physics with realistic effects"""
        # Update GPS readings at realistic rate
        self.update_gps(dt)
        
        # Optional: Apply effort decay (simulates resistance/friction)
        if apply_decay:
            # Small decay to simulate natural resistance
            if abs(self.speed_effort) > 0.5:
                self.speed_effort *= 0.995
            else:
                self.speed_effort = 0.0
                
            if abs(self.yaw_effort) > 0.5:
                self.yaw_effort *= 0.995
            else:
                self.yaw_effort = 0.0
        
        # Update angular velocity from yaw effort
        # Convention: POSITIVE yaw effort = turn RIGHT (CW, negative angular change)
        self.angular_velocity = -self.yaw_effort * YAW_TO_ANGULAR
        self.heading += self.angular_velocity
        
        # Normalize heading to [-pi, pi]
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))
        
        # Update velocity from speed effort
        speed = self.speed_effort * SPEED_TO_VELOCITY
        self.velocity_x = speed * math.cos(self.heading)
        self.velocity_y = -speed * math.sin(self.heading)  # Negative because Y increases downward
        
        # Add environmental effects (current/drift)
        self.velocity_x += self.current_x
        self.velocity_y += self.current_y
        
        # Update position
        self.x += self.velocity_x
        self.y += self.velocity_y
        
        # Keep within bounds
        self.x = max(20, min(WINDOW_WIDTH - 20, self.x))
        self.y = max(20, min(WINDOW_HEIGHT - 20, self.y))
        
        # Record path history
        if len(self.path_history) == 0 or \
           (abs(self.x - self.path_history[-1][0]) > 2 or abs(self.y - self.path_history[-1][1]) > 2):
            self.path_history.append((self.x, self.y))
            if len(self.path_history) > self.max_history:
                self.path_history.pop(0)
    
    def set_speed_effort(self, effort, apply_delay=False, current_time=0.0):
        """Set speed effort with clamping and optional communication delay"""
        if apply_delay:
            self.command_buffer.append(('speed', effort, current_time + COMM_DELAY))
        else:
            self.speed_effort = max(-MAX_SPEED_EFFORT, min(MAX_SPEED_EFFORT, effort))
    
    def set_yaw_effort(self, effort, apply_delay=False, current_time=0.0):
        """Set yaw effort with clamping and optional communication delay"""
        if apply_delay:
            self.command_buffer.append(('yaw', effort, current_time + COMM_DELAY))
        else:
            self.yaw_effort = max(-MAX_YAW_EFFORT, min(MAX_YAW_EFFORT, effort))
    
    def process_delayed_commands(self, current_time):
        """Process commands that have passed their delay time"""
        remaining_commands = []
        for cmd_type, effort, execute_time in self.command_buffer:
            if current_time >= execute_time:
                if cmd_type == 'speed':
                    self.speed_effort = max(-MAX_SPEED_EFFORT, min(MAX_SPEED_EFFORT, effort))
                elif cmd_type == 'yaw':
                    self.yaw_effort = max(-MAX_YAW_EFFORT, min(MAX_YAW_EFFORT, effort))
            else:
                remaining_commands.append((cmd_type, effort, execute_time))
        self.command_buffer = remaining_commands
    
    def stop(self):
        """Stop all movement"""
        self.speed_effort = 0.0
        self.yaw_effort = 0.0
    
    def reset(self, x, y, heading=0.0):
        """Reset vehicle to position with optional heading"""
        self.x = x
        self.y = y
        self.heading = heading
        self.speed_effort = 0.0
        self.yaw_effort = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.angular_velocity = 0.0
        self.path_history = []
        
        # Reset GPS state
        self.gps_x = x
        self.gps_y = y
        self.gps_heading = heading
        self.gps_update_timer = 0.0
        self.command_buffer = []
    
    def draw(self, screen):
        """Draw the vehicle as a triangle with heading indicator"""
        # Vehicle body (triangle pointing in heading direction)
        size = 20
        points = []
        # Front point
        points.append((
            self.x + size * math.cos(self.heading),
            self.y - size * math.sin(self.heading)
        ))
        # Back left
        points.append((
            self.x + size * 0.5 * math.cos(self.heading + 2.5),
            self.y - size * 0.5 * math.sin(self.heading + 2.5)
        ))
        # Back right
        points.append((
            self.x + size * 0.5 * math.cos(self.heading - 2.5),
            self.y - size * 0.5 * math.sin(self.heading - 2.5)
        ))
        
        pygame.draw.polygon(screen, BLUE, points)
        pygame.draw.polygon(screen, WHITE, points, 2)
        
        # Heading line
        end_x = self.x + 30 * math.cos(self.heading)
        end_y = self.y - 30 * math.sin(self.heading)
        pygame.draw.line(screen, CYAN, (self.x, self.y), (end_x, end_y), 2)
        
        # Draw path history
        if len(self.path_history) > 1:
            for i in range(len(self.path_history) - 1):
                alpha = int(255 * (i / len(self.path_history)))
                color = (0, alpha // 2, alpha)
                pygame.draw.line(screen, color, self.path_history[i], self.path_history[i + 1], 2)


class Buoy:
    """Represents a buoy obstacle with collision detection."""
    
    def __init__(self, x, y, radius=15, color=ORANGE):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color
        self.bounding_box_padding = 2  # Extra padding for bounding box (increased collision zone)
    
    def get_bounding_box(self):
        """
        Get the bounding box for collision detection.
        Returns: (left, top, width, height)
        """
        size = (self.radius + self.bounding_box_padding) * 2
        left = self.x - self.radius - self.bounding_box_padding
        top = self.y - self.radius - self.bounding_box_padding
        return (left, top, size, size)
    
    def check_collision(self, vehicle_x, vehicle_y, vehicle_radius=3):
        """
        Check if vehicle collides with this buoy.
        
        Parameters:
        vehicle_x, vehicle_y: Vehicle position
        vehicle_radius: Vehicle collision radius (increased for safety margin)
        
        Returns:
        bool: True if collision detected
        """
        distance = math.sqrt((self.x - vehicle_x)**2 + (self.y - vehicle_y)**2)
        # Use bounding box padding to extend collision zone
        return distance < (self.radius + self.bounding_box_padding + vehicle_radius)
    
    def draw(self, screen, show_bounding_box=True):
        """Draw the buoy and optionally its bounding box."""
        # Draw buoy body (cylinder top view)
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)
        pygame.draw.circle(screen, WHITE, (int(self.x), int(self.y)), self.radius, 2)
        
        # Draw center dot
        pygame.draw.circle(screen, WHITE, (int(self.x), int(self.y)), 3)
        
        # Draw bounding box
        if show_bounding_box:
            bbox = self.get_bounding_box()
            pygame.draw.rect(screen, YELLOW, bbox, 1)


class DockingPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def get_lat_lon(self):
        """Convert screen coordinates to simulated lat/lon"""
        lat = (WINDOW_HEIGHT / 2 - self.y) * PIXELS_TO_METERS * METERS_TO_LATLON
        lon = (self.x - WINDOW_WIDTH / 2) * PIXELS_TO_METERS * METERS_TO_LATLON
        return lat, lon
    
    def draw(self, screen):
        """Draw the docking point as a target"""
        # Outer circle
        pygame.draw.circle(screen, RED, (int(self.x), int(self.y)), 15, 2)
        pygame.draw.circle(screen, RED, (int(self.x), int(self.y)), 10, 2)
        pygame.draw.circle(screen, RED, (int(self.x), int(self.y)), 5)
        
        # Cross
        pygame.draw.line(screen, RED, (self.x - 20, self.y), (self.x + 20, self.y), 2)
        pygame.draw.line(screen, RED, (self.x, self.y - 20), (self.x, self.y + 20), 2)


class Simulator:
    def __init__(self):
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("2D Movement Simulator with Docking")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 20)
        
        # Calculate starting position (entry to U-turn course)
        # Position the vehicle at the start of the left lane (bottom center)
        lane_width = 250  # Increased from 150
        straight_length = 400  # Increased from 200
        self.initial_spawn_x = WINDOW_WIDTH // 2  # Center horizontally in the lane
        self.initial_spawn_y = WINDOW_HEIGHT // 2 + straight_length // 2  # Start of the course
        self.initial_spawn_heading = 0.0  # Facing right (East)
        
        # Initialize vehicle at starting position
        self.vehicle = Vehicle(self.initial_spawn_x, self.initial_spawn_y)
        self.docking_point = None
        self.docking_state = DockingState.IDLE
        
        # Initialize docking controller (modularized)
        self.docking_controller = DockingController()
        
        # Recording/Playback state
        self.recording_state = RecordingState.IDLE
        self.playback_index = 0
        self.playback_time_accumulator = 0.0
        self.initial_vehicle_state = None  # (x, y, heading) for playback
        self.has_left_dock = False  # Track if vehicle has moved away from docking point
        
        # Setup U-turn buoy course
        self.buoys = self.create_uturn_course()
        self.show_bounding_boxes = True
        self.collision_count = 0
        self.is_colliding = False
        
        # Control state
        self.keys_pressed = set()
        self.running = True
        
        # Simulation time (for delayed commands)
        self.sim_time = 0.0
        
        # Toggle for showing simulation effects
        self.show_sim_effects = True
        
    def handle_input(self):
        """Handle keyboard input"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                self.keys_pressed.add(event.key)
                
                # Handle single-press actions
                if event.key == pygame.K_q:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.vehicle.stop()
                    self.docking_state = DockingState.IDLE
                elif event.key == pygame.K_t:
                    self.handle_recording_action()
                elif event.key == pygame.K_r:
                    self.reset_simulation()
                elif event.key == pygame.K_e:
                    self.handle_docking_action()
                elif event.key == pygame.K_b:
                    self.show_bounding_boxes = not self.show_bounding_boxes
                    print(f"Bounding boxes: {'ON' if self.show_bounding_boxes else 'OFF'}")
                elif event.key == pygame.K_n:
                    self.show_sim_effects = not self.show_sim_effects
                    print(f"Simulation effects display: {'ON' if self.show_sim_effects else 'OFF'}")
                    
            elif event.type == pygame.KEYUP:
                self.keys_pressed.discard(event.key)
        
        # Handle continuous controls (WASD) only if not in autonomous mode
        if self.docking_state != DockingState.GOING_TO_DOCK and self.recording_state not in [RecordingState.PLAYING_BACK, RecordingState.RETURNING_TO_START]:
            # Speed control (W/S)
            if pygame.K_w in self.keys_pressed:
                self.vehicle.set_speed_effort(self.vehicle.speed_effort + SPEED_INCREMENT)
            if pygame.K_s in self.keys_pressed:
                self.vehicle.set_speed_effort(self.vehicle.speed_effort - SPEED_INCREMENT)
            
            # Yaw control (A/D)
            # D = turn right (positive yaw effort), A = turn left (negative yaw effort)
            if pygame.K_d in self.keys_pressed:
                self.vehicle.set_yaw_effort(self.vehicle.yaw_effort + YAW_INCREMENT)
            if pygame.K_a in self.keys_pressed:
                self.vehicle.set_yaw_effort(self.vehicle.yaw_effort - YAW_INCREMENT)
    
    def create_uturn_course(self):
        """
        Create a U-turn buoy course pattern.
        Returns a list of Buoy objects forming a U-shape.
        """
        buoys = []
        
        # Center of the course
        center_x = WINDOW_WIDTH // 2
        center_y = WINDOW_HEIGHT // 2
        
        # U-turn course parameters (made bigger)
        lane_width = 250  # Distance between inner and outer buoys (increased from 150)
        straight_length = 400  # Length of the straight sections (increased from 200)
        turn_radius = 180  # Radius of the U-turn (increased from 100)
        buoy_spacing = 60  # Spacing between buoys (increased from 50)
        
        # Left straight section (outer buoys)
        for i in range(int(straight_length / buoy_spacing) + 1):
            y = center_y - straight_length / 2 + i * buoy_spacing
            buoys.append(Buoy(center_x - lane_width / 2, y, color=RED))
        
        # Left straight section (inner buoys)
        for i in range(int(straight_length / buoy_spacing) + 1):
            y = center_y - straight_length / 2 + i * buoy_spacing
            buoys.append(Buoy(center_x + lane_width / 2, y, color=GREEN))
        
        # U-turn section (top)
        num_turn_buoys = 8
        for i in range(num_turn_buoys + 1):
            angle = math.pi + i * math.pi / num_turn_buoys  # From left to right
            # Outer arc
            x_outer = center_x + (turn_radius + lane_width / 2) * math.cos(angle)
            y_outer = center_y - straight_length / 2 + (turn_radius + lane_width / 2) * math.sin(angle)
            buoys.append(Buoy(x_outer, y_outer, color=RED))
            
            # Inner arc
            x_inner = center_x + (turn_radius - lane_width / 2) * math.cos(angle)
            y_inner = center_y - straight_length / 2 + (turn_radius - lane_width / 2) * math.sin(angle)
            buoys.append(Buoy(x_inner, y_inner, color=GREEN))
        
        return buoys
    
    def check_collisions(self):
        """Check for collisions between vehicle and buoys."""
        for buoy in self.buoys:
            if buoy.check_collision(self.vehicle.x, self.vehicle.y):
                return True
        return False
    
    def handle_recording_action(self):
        """Handle T key press for combined docking/recording/playback actions"""
        if self.recording_state == RecordingState.IDLE:
            # Check if there's a docking point set - if yes, navigate to it
            if self.docking_point is not None and self.docking_state == DockingState.INIT_POINT_SET:
                # Navigate back to the docking point
                self.docking_state = DockingState.GOING_TO_DOCK
                print("Navigating back to docking point...")
            else:
                # Set docking point and start recording
                self.docking_point = DockingPoint(self.vehicle.x, self.vehicle.y)
                dock_lat, dock_lon = self.docking_point.get_lat_lon()
                
                # Configure docking controller with target (for later return)
                self.docking_controller.set_target(dock_lat, dock_lon)
                
                # Start recording
                lat, lon = self.vehicle.get_lat_lon()
                self.docking_controller.start_lat_lon_recording(lat, lon, self.vehicle.get_heading_pixhawk())
                self.initial_vehicle_state = (self.vehicle.x, self.vehicle.y, self.vehicle.heading)
                self.recording_state = RecordingState.RECORDING
                self.docking_state = DockingState.INIT_POINT_SET
                self.has_left_dock = False  # Reset flag when starting new recording
                print(f"Docking point set and recording started at: Lat={dock_lat:.6f}, Lon={dock_lon:.6f}")
                print("Recording will automatically stop when you return to the docking point.")
            
        elif self.recording_state == RecordingState.RECORDING:
            # Inform user that recording stops automatically
            print("Recording in progress... Return to the docking point to auto-stop and playback.")
                
        elif self.recording_state == RecordingState.PLAYING_BACK:
            # Stop playback and keep docking point available
            self.recording_state = RecordingState.IDLE
            self.docking_state = DockingState.INIT_POINT_SET  # Allow navigating back
            self.vehicle.stop()
            print("Playback stopped. Press T again to navigate back to docking point.")
    
    def handle_docking_action(self):
        """Handle E key press for docking actions (now reusable)"""
        if self.docking_state == DockingState.IDLE:
            # Set docking init point at current position
            self.docking_point = DockingPoint(self.vehicle.x, self.vehicle.y)
            dock_lat, dock_lon = self.docking_point.get_lat_lon()
            
            # Configure docking controller with target
            self.docking_controller.set_target(dock_lat, dock_lon)
            
            self.docking_state = DockingState.INIT_POINT_SET
            print(f"Docking init point set at: Lat={dock_lat:.6f}, Lon={dock_lon:.6f}")
        elif self.docking_state == DockingState.INIT_POINT_SET:
            # Start autonomous navigation to docking point
            self.docking_state = DockingState.GOING_TO_DOCK
            print("Starting autonomous docking...")
        elif self.docking_state == DockingState.DOCKED:
            # Reset docking to allow setting a new point
            self.docking_state = DockingState.IDLE
            self.docking_point = None
            print("Docking reset. Press E again to set a new docking point.")
    
    def reset_simulation(self):
        """Reset the entire simulation to initial state"""
        # Reset vehicle to spawn position
        self.vehicle.reset(self.initial_spawn_x, self.initial_spawn_y, self.initial_spawn_heading)
        
        # Reset docking state
        self.docking_point = None
        self.docking_state = DockingState.IDLE
        
        # Reset recording state
        self.recording_state = RecordingState.IDLE
        self.playback_index = 0
        self.playback_time_accumulator = 0.0
        self.initial_vehicle_state = None
        
        # Reset collision counter
        self.collision_count = 0
        self.is_colliding = False
        
        print("Simulation reset to initial state.")
    
    def update_recording(self, dt):
        """Update movement recording during manual control"""
        if self.recording_state == RecordingState.RECORDING:
            # Record current lat/lon
            lat, lon = self.vehicle.get_lat_lon()
            self.docking_controller.record_lat_lon(lat, lon, dt)
            
            # Check if vehicle has reached the docking point
            if self.docking_point is not None:
                current_lat, current_lon = self.vehicle.get_lat_lon(use_gps=True)
                distance_to_dock = self.docking_controller.get_distance_to_target(current_lat, current_lon)
                
                # Track if vehicle has left the docking area (moved at least 3m away)
                if not self.has_left_dock and distance_to_dock > 3.0:
                    self.has_left_dock = True
                    print("Left docking area - recording path...")
                
                # Only check for return to dock after vehicle has left the area
                if self.has_left_dock and distance_to_dock < self.docking_controller.docking_distance_threshold:
                    num_frames = self.docking_controller.stop_lat_lon_recording()
                    duration = self.docking_controller.get_lat_lon_duration()
                    print(f"Reached docking point! Recording stopped. Recorded {num_frames} frames ({duration:.1f}s)")
                    
                    if num_frames > 0:
                        # Navigate back to docking point (already set when recording started)
                        self.vehicle.stop()
                        
                        # Start return navigation phase
                        self.playback_index = 0
                        self.playback_time_accumulator = 0.0
                        self.recording_state = RecordingState.RETURNING_TO_START
                        init_x, init_y, init_heading = self.initial_vehicle_state
                        init_lat = (WINDOW_HEIGHT / 2 - init_y) * PIXELS_TO_METERS * METERS_TO_LATLON
                        init_lon = (init_x - WINDOW_WIDTH / 2) * PIXELS_TO_METERS * METERS_TO_LATLON
                        print(f"Navigating back to docking point: Lat={init_lat:.6f}, Lon={init_lon:.6f}")
                    else:
                        print("No movements recorded!")
                        self.recording_state = RecordingState.IDLE
    
    def update_return_navigation(self, dt):
        """Update autonomous navigation back to initial recording point"""
        if self.recording_state != RecordingState.RETURNING_TO_START:
            return
        
        current_lat, current_lon = self.vehicle.get_lat_lon(use_gps=True)
        current_heading_deg = self.vehicle.get_heading_pixhawk(use_gps=True)
        
        yaw_effort, speed_effort, is_docked = self.docking_controller.calculate_control_efforts(
            current_lat, current_lon, current_heading_deg, dt
        )
        
        if is_docked:
            # Reached the initial recording point, transition to playback
            self.vehicle.stop()
            distance = self.docking_controller.get_distance_to_target(current_lat, current_lon)
            print(f"Reached initial point! Distance: {distance:.2f}m")
            print("Starting playback...")
            
            # Transition to playback
            self.recording_state = RecordingState.PLAYING_BACK
            return
        
        # Apply control efforts to navigate back
        self.vehicle.set_yaw_effort(yaw_effort)
        self.vehicle.set_speed_effort(speed_effort)
    
    def update_playback(self, dt):
        """Update movement playback"""
        if self.recording_state != RecordingState.PLAYING_BACK:
            return
        
        recorded_lat_lon = self.docking_controller.get_recorded_lat_lon()
        
        if self.playback_index >= len(recorded_lat_lon):
            # Playback finished
            self.vehicle.stop()
            self.recording_state = RecordingState.IDLE
            print("Playback completed!")
            return
        
        # Get target lat/lon from recording
        target_lat, target_lon, frame_dt = recorded_lat_lon[self.playback_index]
        
        # Only update target if it has changed (to avoid resetting PID controller)
        if self.docking_controller.target_lat != target_lat or self.docking_controller.target_lon != target_lon:
            self.docking_controller.set_target(target_lat, target_lon)
        
        # Use docking controller to navigate to the recorded point
        current_lat, current_lon = self.vehicle.get_lat_lon(use_gps=True)
        current_heading_deg = self.vehicle.get_heading_pixhawk(use_gps=True)
        
        # Temporarily set target to recorded point
        # self.docking_controller.set_target(target_lat, target_lon)
        
        yaw_effort, speed_effort, is_docked = self.docking_controller.calculate_control_efforts(
            current_lat, current_lon, current_heading_deg, dt
        )
        
        # Apply efforts
        self.vehicle.set_yaw_effort(yaw_effort)
        self.vehicle.set_speed_effort(speed_effort)
        
        # Check distance to current target to advance to next waypoint
        distance_to_target = self.docking_controller.get_distance_to_target(current_lat, current_lon)
        
        # If close enough to target, move to next waypoint
        # Using 2.0 meters as threshold (slightly larger than docking threshold to ensure smooth path following)
        if distance_to_target < 2.0:
            self.playback_index += 1
    
    def update_docking(self, dt):
        """Update autonomous docking behavior using DockingController"""
        if self.docking_state != DockingState.GOING_TO_DOCK or self.docking_point is None:
            return

        current_lat, current_lon = self.vehicle.get_lat_lon(use_gps=True)
        current_heading_deg = self.vehicle.get_heading_pixhawk(use_gps=True)
        
        yaw_effort, speed_effort, is_docked = self.docking_controller.calculate_control_efforts(
            current_lat, current_lon, current_heading_deg, dt
        )
        
        if is_docked:
            self.vehicle.stop()
            self.docking_state = DockingState.DOCKED
            distance = self.docking_controller.get_distance_to_target(current_lat, current_lon)
            print(f"Docked successfully! Final distance: {distance:.2f}m")
            return
        
        self.vehicle.set_yaw_effort(yaw_effort, apply_delay=True, current_time=self.sim_time)
        self.vehicle.set_speed_effort(speed_effort, apply_delay=True, current_time=self.sim_time)
    
    def draw_ui(self):
        """Draw UI elements"""
        y_offset = 10
        
        # Title
        title = self.font.render("2D Movement Simulator", True, WHITE)
        self.screen.blit(title, (10, y_offset))
        y_offset += 35
        
        # Vehicle info
        lat, lon = self.vehicle.get_lat_lon()
        collision = self.check_collisions()
        heading_pixhawk = self.vehicle.get_heading_pixhawk()
        
        # Get true position for comparison
        true_lat, true_lon = self.vehicle.get_lat_lon(use_gps=False)
        true_heading = self.vehicle.get_heading_pixhawk(use_gps=False)
        
        info_texts = [
            f"GPS Position: {lat:.6f}, {lon:.6f}",
            f"GPS Heading: {heading_pixhawk:.1f}° (0=N, 90=E, CW)",
            f"Speed Effort: {self.vehicle.speed_effort:.1f}",
            f"Yaw Effort: {self.vehicle.yaw_effort:.1f}",
            f"Docking State: {self.docking_state.name}",
            f"Recording State: {self.recording_state.name}",
        ]
        
        # Add simulation effects info if enabled
        if self.show_sim_effects:
            pos_error = math.sqrt((true_lat - lat)**2 + (true_lon - lon)**2) / METERS_TO_LATLON
            heading_error = abs(true_heading - heading_pixhawk)
            if heading_error > 180:
                heading_error = 360 - heading_error
            info_texts.extend([
                f"--- Simulation Effects ---",
                f"Position Error: {pos_error:.2f}m",
                f"Heading Error: {heading_error:.1f}°",
                f"Current: {CURRENT_STRENGTH:.1f} m/s @ {CURRENT_DIRECTION:.0f}°",
                f"GPS Rate: {GPS_UPDATE_RATE} Hz",
                f"Comm Delay: {COMM_DELAY*1000:.0f}ms",
            ])
        
        # Add collision warning
        if collision:
            collision_text = self.small_font.render("⚠ COLLISION!", True, RED)
            self.screen.blit(collision_text, (10, y_offset))
            y_offset += 25
        
        # Add recording info if applicable
        if self.recording_state == RecordingState.RECORDING:
            duration = self.docking_controller.get_lat_lon_duration()
            info_texts.append(f"Recording Time: {duration:.1f}s")
        elif self.recording_state == RecordingState.PLAYING_BACK:
            recorded = self.docking_controller.get_recorded_lat_lon()
            progress = (self.playback_index / len(recorded) * 100) if recorded else 0
            info_texts.append(f"Playback: {progress:.0f}% ({self.playback_index}/{len(recorded)})")
        
        for text in info_texts:
            surface = self.small_font.render(text, True, WHITE)
            self.screen.blit(surface, (10, y_offset))
            y_offset += 25
        
        # Docking point info
        if self.docking_point:
            dock_lat, dock_lon = self.docking_point.get_lat_lon()
            current_lat, current_lon = self.vehicle.get_lat_lon()
            distance = self.docking_controller.get_distance_to_target(current_lat, current_lon)
            
            dock_text = self.small_font.render(
                f"Dock Point: {dock_lat:.6f}, {dock_lon:.6f} | Dist: {distance:.2f}m",
                True, RED
            )
            self.screen.blit(dock_text, (10, y_offset))
            y_offset += 25
            
            # Show heading error during autonomous docking
            if self.docking_state == DockingState.GOING_TO_DOCK:
                heading_error = self.docking_controller.get_heading_error_deg()
                error_text = self.small_font.render(
                    f"Heading Error: {heading_error:.1f}°",
                    True, YELLOW
                )
                self.screen.blit(error_text, (10, y_offset))
                y_offset += 25
        
        # Controls help
        y_offset = WINDOW_HEIGHT - 240
        controls = [
            "Controls:",
            "W/S - Speed +/-",
            "A/D - Yaw Right(-)/Left(+)",
            "E - Set Dock / Go / Reset Dock",
            "T - Set Home & Record / Auto-Playback (on dock) / Go Home",
            "R - Reset Simulation",
            "B - Toggle Bounding Boxes",
            "N - Toggle Sim Effects Info",
            "Space - Stop",
            "Q - Quit",
        ]
        
        for text in controls:
            surface = self.small_font.render(text, True, GRAY if text == "Controls:" else DARK_GRAY)
            self.screen.blit(surface, (10, y_offset))
            y_offset += 22
        
        # Draw effort bars
        self.draw_effort_bars()
    
    def draw_effort_bars(self):
        """Draw visual effort indicators"""
        bar_width = 240
        bar_height = 24
        bar_x = WINDOW_WIDTH - bar_width - 20
        bar_y = 20
        
        # Speed effort bar
        speed_label = self.small_font.render(f"Speed Effort: {self.vehicle.speed_effort:.1f}/±300", True, WHITE)
        self.screen.blit(speed_label, (bar_x, bar_y))
        bar_y += 25
        
        pygame.draw.rect(self.screen, DARK_GRAY, (bar_x, bar_y, bar_width, bar_height))
        speed_fill = int((self.vehicle.speed_effort / MAX_SPEED_EFFORT) * bar_width / 2)
        if speed_fill > 0:
            color = GREEN if self.vehicle.speed_effort <= 100 else RED
            pygame.draw.rect(self.screen, color, 
                           (bar_x + bar_width // 2, bar_y, speed_fill, bar_height))
        elif speed_fill < 0:
            color = ORANGE if self.vehicle.speed_effort >= -100 else RED
            pygame.draw.rect(self.screen, color, 
                           (bar_x + bar_width // 2 + speed_fill, bar_y, -speed_fill, bar_height))
        
        # Draw ±100 delta markers
        delta_marker_pos = int((100 / MAX_SPEED_EFFORT) * bar_width / 2)
        pygame.draw.line(self.screen, YELLOW, 
                        (bar_x + bar_width // 2 + delta_marker_pos, bar_y),
                        (bar_x + bar_width // 2 + delta_marker_pos, bar_y + bar_height), 1)
        pygame.draw.line(self.screen, YELLOW,
                        (bar_x + bar_width // 2 - delta_marker_pos, bar_y),
                        (bar_x + bar_width // 2 - delta_marker_pos, bar_y + bar_height), 1)
        
        pygame.draw.rect(self.screen, WHITE, (bar_x, bar_y, bar_width, bar_height), 2)
        
        # Yaw effort bar
        bar_y += 40
        yaw_label = self.small_font.render(f"Yaw Effort: {self.vehicle.yaw_effort:.1f}/±300 (L/R)", True, WHITE)
        self.screen.blit(yaw_label, (bar_x, bar_y))
        bar_y += 25
        
        pygame.draw.rect(self.screen, DARK_GRAY, (bar_x, bar_y, bar_width, bar_height))
        yaw_fill = int((self.vehicle.yaw_effort / MAX_YAW_EFFORT) * bar_width / 2)
        if yaw_fill > 0:
            color = CYAN if self.vehicle.yaw_effort <= 100 else RED
            pygame.draw.rect(self.screen, color, 
                           (bar_x + bar_width // 2, bar_y, yaw_fill, bar_height))
        elif yaw_fill < 0:
            color = YELLOW if self.vehicle.yaw_effort >= -100 else RED
            pygame.draw.rect(self.screen, color, 
                           (bar_x + bar_width // 2 + yaw_fill, bar_y, -yaw_fill, bar_height))
        
        # Draw ±100 delta markers
        pygame.draw.line(self.screen, YELLOW,
                        (bar_x + bar_width // 2 + delta_marker_pos, bar_y),
                        (bar_x + bar_width // 2 + delta_marker_pos, bar_y + bar_height), 1)
        pygame.draw.line(self.screen, YELLOW,
                        (bar_x + bar_width // 2 - delta_marker_pos, bar_y),
                        (bar_x + bar_width // 2 - delta_marker_pos, bar_y + bar_height), 1)
        
        pygame.draw.rect(self.screen, WHITE, (bar_x, bar_y, bar_width, bar_height), 2)
    
    def draw_grid(self):
        """Draw coordinate grid"""
        grid_spacing = 50
        
        # Vertical lines
        for x in range(0, WINDOW_WIDTH, grid_spacing):
            color = GRAY if x == WINDOW_WIDTH // 2 else DARK_GRAY
            pygame.draw.line(self.screen, color, (x, 0), (x, WINDOW_HEIGHT), 1)
        
        # Horizontal lines
        for y in range(0, WINDOW_HEIGHT, grid_spacing):
            color = GRAY if y == WINDOW_HEIGHT // 2 else DARK_GRAY
            pygame.draw.line(self.screen, color, (0, y), (WINDOW_WIDTH, y), 1)
    
    def lat_lon_to_screen(self, lat, lon):
        """Convert lat/lon to screen coordinates"""
        # Inverse of get_lat_lon
        # lat = (WINDOW_HEIGHT / 2 - y) * PIXELS_TO_METERS * METERS_TO_LATLON
        # y = WINDOW_HEIGHT / 2 - lat / (PIXELS_TO_METERS * METERS_TO_LATLON)
        y = WINDOW_HEIGHT / 2 - lat / (PIXELS_TO_METERS * METERS_TO_LATLON)
        x = WINDOW_WIDTH / 2 + lon / (PIXELS_TO_METERS * METERS_TO_LATLON)
        return int(x), int(y)

    def draw_recorded_path(self):
        """Draw the recorded lat/lon path"""
        recorded_lat_lon = self.docking_controller.get_recorded_lat_lon()
        if not recorded_lat_lon or len(recorded_lat_lon) < 2:
            return
            
        points = []
        for lat, lon, _ in recorded_lat_lon:
            x, y = self.lat_lon_to_screen(lat, lon)
            points.append((x, y))
            
        if len(points) > 1:
            pygame.draw.lines(self.screen, YELLOW, False, points, 2)
            
        # Draw start and end points
        start_x, start_y = points[0]
        end_x, end_y = points[-1]
        
        pygame.draw.circle(self.screen, GREEN, (start_x, start_y), 5)
        pygame.draw.circle(self.screen, RED, (end_x, end_y), 5)

    def run(self):
        """Main simulation loop"""
        while self.running:
            dt = self.clock.tick(FPS) / 1000.0  # Delta time in seconds
            self.sim_time += dt
            
            self.handle_input()
            
            # Process delayed commands
            self.vehicle.process_delayed_commands(self.sim_time)
            
            self.update_recording(dt)
            self.update_return_navigation(dt)
            self.update_playback(dt)
            self.update_docking(dt)
            
            # Check for collisions BEFORE updating vehicle
            self.is_colliding = self.check_collisions()
            
            if self.is_colliding:
                # Stop the vehicle on collision
                self.vehicle.stop()
                # Optionally: push back slightly to prevent getting stuck
                # self.vehicle.x -= self.vehicle.velocity_x * 2
                # self.vehicle.y -= self.vehicle.velocity_y * 2
            
            # Disable decay during autonomous operations
            apply_decay = (self.docking_state != DockingState.GOING_TO_DOCK and 
                          self.recording_state not in [RecordingState.PLAYING_BACK, RecordingState.RETURNING_TO_START])
            self.vehicle.update(dt, apply_decay=apply_decay)
            
            # Draw everything
            self.screen.fill(BLACK)
            self.draw_grid()
            
            # Draw buoys
            for buoy in self.buoys:
                buoy.draw(self.screen, show_bounding_box=self.show_bounding_boxes)
            
            if self.docking_point:
                self.docking_point.draw(self.screen)
                # Draw line from vehicle to docking point
                pygame.draw.line(self.screen, RED, 
                               (self.vehicle.x, self.vehicle.y),
                               (self.docking_point.x, self.docking_point.y), 1)
            
            # Draw initial position marker during playback
            if self.recording_state == RecordingState.PLAYING_BACK and self.initial_vehicle_state:
                init_x, init_y, _ = self.initial_vehicle_state
                pygame.draw.circle(self.screen, GREEN, (int(init_x), int(init_y)), 8, 2)
                pygame.draw.line(self.screen, GREEN, (init_x - 10, init_y), (init_x + 10, init_y), 2)
                pygame.draw.line(self.screen, GREEN, (init_x, init_y - 10), (init_x, init_y + 10), 2)
            
            self.vehicle.draw(self.screen)
            
            # Draw recorded path
            self.draw_recorded_path()
            
            # Draw simulation effects visualization
            if self.show_sim_effects:
                # Draw GPS position (noisy) vs true position
                pygame.draw.circle(self.screen, YELLOW, (int(self.vehicle.gps_x), int(self.vehicle.gps_y)), 8, 2)
                pygame.draw.line(self.screen, YELLOW, 
                               (int(self.vehicle.x), int(self.vehicle.y)),
                               (int(self.vehicle.gps_x), int(self.vehicle.gps_y)), 1)
                
                # Draw current direction arrow
                arrow_length = 50
                current_angle = math.radians(CURRENT_DIRECTION)
                arrow_end_x = self.vehicle.x + arrow_length * math.cos(current_angle)
                arrow_end_y = self.vehicle.y - arrow_length * math.sin(current_angle)
                pygame.draw.line(self.screen, CYAN, 
                               (int(self.vehicle.x), int(self.vehicle.y)),
                               (int(arrow_end_x), int(arrow_end_y)), 2)
                # Arrow head
                arrow_size = 8
                for angle_offset in [-2.5, 2.5]:
                    head_angle = current_angle + angle_offset
                    head_x = arrow_end_x - arrow_size * math.cos(head_angle)
                    head_y = arrow_end_y + arrow_size * math.sin(head_angle)
                    pygame.draw.line(self.screen, CYAN,
                                   (int(arrow_end_x), int(arrow_end_y)),
                                   (int(head_x), int(head_y)), 2)
            
            # Draw collision warning circle around vehicle
            if self.is_colliding:
                pygame.draw.circle(self.screen, RED, (int(self.vehicle.x), int(self.vehicle.y)), 35, 3)
            
            # Draw recording indicator
            if self.recording_state == RecordingState.RECORDING:
                # Blinking red circle in top-right corner
                if int(pygame.time.get_ticks() / 500) % 2 == 0:
                    pygame.draw.circle(self.screen, RED, (WINDOW_WIDTH - 30, 30), 12)
                    record_text = self.small_font.render("REC", True, WHITE)
                    self.screen.blit(record_text, (WINDOW_WIDTH - 55, 22))
            
            self.draw_ui()
            
            pygame.display.flip()
        
        pygame.quit()
        sys.exit()


if __name__ == "__main__":
    simulator = Simulator()
    simulator.run()
