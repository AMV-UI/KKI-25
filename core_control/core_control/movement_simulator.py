#!/usr/bin/env python3
"""
ROS2 Node for 2D Movement Simulator with Docking Visualization (rewritten)
- Visualizes: vehicle, vehicle history, waypoints loaded from waypoints.txt,
  and recorded path messages received via ROS.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from core_msgs.msg import Pixhawk
from core.utils.config import Topic, NodeConfig
import pygame
import math
import sys
import random
import traceback
from time import time
import json
import ast
import os
from pygame import gfxdraw

# Window / rendering
WINDOW_WIDTH = 2000
WINDOW_HEIGHT = 1000
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

# Physics constants
MAX_SPEED_EFFORT = 300.0
MAX_YAW_EFFORT = 300.0
SPEED_TO_VELOCITY = 0.005
YAW_TO_ANGULAR = 0.0001
PIXELS_TO_METERS = 1.0 / 30.0  # 30 pixels per meter
METERS_TO_LATLON = 0.00001

# GPS simulation
GPS_UPDATE_RATE = 50  # Hz
GPS_POSITION_NOISE = 0.0
GPS_HEADING_NOISE = 0.0

# Arena and buoy settings
LANE_WIDTH = 750   # 25m * 30 px/m
LANE_HEIGHT = 750  # 25m * 30 px/m
LANE_MARGIN_X = 200
LANE_MARGIN_Y = 125
GAP_BETWEEN_LANES = 100
BUOY_RADIUS = 15
BUOY_COLOR_RED = (255, 60, 60)
BUOY_COLOR_GREEN = (60, 255, 60)
CAMERA_FOV_DEG = 60
CAMERA_RANGE = 200

def generate_buoys():
    buoys = []
    # Offsets (scaled)
    # Inner: 5m -> 150px
    # Outer: 3m -> 90px
    # Spacing: 4m -> 120px
    inner_inset = 150
    outer_inset = 90
    spacing = 120
    
    # LINTASAN A (Left Box)
    ox, oy = LANE_MARGIN_X, LANE_MARGIN_Y
    w, h = LANE_WIDTH, LANE_HEIGHT
    
    # Right Side (3 pairs) - Going UP
    for i in range(3):
        y = oy + h - 240 - i * spacing # Start higher up? 
        # Image shows buoys along the straight parts.
        # Let's center them roughly. 25m side. 3 buoys span ~8-12m?
        # Let's start from bottom-ish.
        y = oy + h - 150 - i * spacing
        buoys.append((ox + w - inner_inset, y, 'green'))
        buoys.append((ox + w - outer_inset, y, 'red'))

    # Top Side (4 pairs) - Going LEFT
    for i in range(4):
        x = ox + w - 150 - i * spacing
        buoys.append((x, oy + inner_inset, 'green'))
        buoys.append((x, oy + outer_inset, 'red'))

    # Left Side (3 pairs) - Going DOWN
    for i in range(3):
        y = oy + 150 + i * spacing
        buoys.append((ox + inner_inset, y, 'green'))
        buoys.append((ox + outer_inset, y, 'red'))

    # LINTASAN B (Right Box)
    ox2 = LANE_MARGIN_X + LANE_WIDTH + GAP_BETWEEN_LANES
    oy2 = LANE_MARGIN_Y
    
    # Left Side (3 pairs) - Going UP
    for i in range(3):
        y = oy2 + h - 150 - i * spacing
        buoys.append((ox2 + inner_inset, y, 'green'))
        buoys.append((ox2 + outer_inset, y, 'red'))

    # Top Side (4 pairs) - Going RIGHT
    for i in range(4):
        x = ox2 + 150 + i * spacing
        buoys.append((x, oy2 + inner_inset, 'green'))
        buoys.append((x, oy2 + outer_inset, 'red'))

    # Right Side (3 pairs) - Going DOWN
    for i in range(3):
        y = oy2 + 150 + i * spacing
        buoys.append((ox2 + w - inner_inset, y, 'green'))
        buoys.append((ox2 + w - outer_inset, y, 'red'))
        
    return buoys

BUOY_POSITIONS = generate_buoys()


class Vehicle:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
        self.heading = 0.0  # radians (0 = right/East, CCW positive)
        self.speed_effort = 0.0
        self.yaw_effort = 0.0

        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.angular_velocity = 0.0

        self.path_history = []
        self.max_history = 500

        self.gps_x = float(x)
        self.gps_y = float(y)
        self.gps_heading = self.heading
        self.gps_update_timer = 0.0
        self.gps_update_interval = 1.0 / GPS_UPDATE_RATE

        self.dsc = 0.0  # Distance between simulated 2 buoys

    def get_lat_lon(self, use_gps=True):
        """
        Convert current screen coordinates to lat/lon-like values.
        lat: increases upward, lon: increases rightward.
        """
        x = self.gps_x if use_gps else self.x
        y = self.gps_y if use_gps else self.y
        # Center of window is reference
        lat = (WINDOW_HEIGHT / 2 - y) * PIXELS_TO_METERS * METERS_TO_LATLON
        lon = (x - WINDOW_WIDTH / 2) * PIXELS_TO_METERS * METERS_TO_LATLON
        return lat, lon

    def get_heading_pixhawk(self, use_gps=True):
        """
        Returns heading in Pixhawk convention:
            0 = North, 90 = East, clockwise positive.
        Our internal heading: 0 = East, CCW positive.
        Conversion: pixhawk_heading = (90 - deg) % 360
        """
        heading = self.gps_heading if use_gps else self.heading
        heading_deg = math.degrees(heading)
        pixhawk_heading = (90.0 - heading_deg) % 360.0
        return pixhawk_heading

    def update(self, dt):
        # GPS update (noisy)
        self.gps_update_timer += dt
        if self.gps_update_timer >= self.gps_update_interval:
            self.gps_update_timer = 0.0
            noise_dist = random.gauss(0, GPS_POSITION_NOISE / 3.0)
            noise_ang = random.uniform(0, 2 * math.pi)
            # gps coords are screen coords (same units as self.x/self.y)
            self.gps_x = self.x + (noise_dist * math.cos(noise_ang)) * 30.0 # Scale noise? No, noise is in meters usually? 
            # Wait, GPS_POSITION_NOISE is 0.0, so it doesn't matter.
            # But if it were non-zero, it should be converted to pixels.
            # Assuming GPS_POSITION_NOISE is in meters.
            self.gps_x = self.x + (noise_dist * math.cos(noise_ang)) / PIXELS_TO_METERS
            self.gps_y = self.y + (noise_dist * math.sin(noise_ang)) / PIXELS_TO_METERS
            self.gps_heading = self.heading + math.radians(random.gauss(0, GPS_HEADING_NOISE / 3.0))

        # Physics: yaw effort positive => turn right (CW) => negative angular velocity (we use CCW positive)
        self.angular_velocity = -self.yaw_effort * YAW_TO_ANGULAR
        self.heading += self.angular_velocity * dt * 60.0  # scale by dt; note constants chosen experimentally
        # normalize heading
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))

        # Linear velocity
        speed = self.speed_effort * SPEED_TO_VELOCITY
        self.velocity_x = speed * math.cos(self.heading)
        # note: y increases downward in screen coords, so subtract sin component
        self.velocity_y = -speed * math.sin(self.heading)

        # Scale velocity to pixels
        # speed is in "units per frame" roughly? 
        # SPEED_TO_VELOCITY = 0.005. Max effort 300 -> 1.5 units.
        # If we want 1.5 m/s, and 30 px/m, we need 45 px/s.
        # Currently: self.x += self.velocity_x * dt * 60.0
        # If velocity_x is 1.5, and dt*60 is ~1, then 1.5 px/frame.
        # At 60 FPS, that's 90 px/s. 90 px / 30 px/m = 3 m/s.
        # Seems reasonable.
        
        self.x += self.velocity_x * dt * 60.0
        self.y += self.velocity_y * dt * 60.0

        # Keep inside window bounds with margin
        margin = 20
        self.x = max(margin, min(WINDOW_WIDTH - margin, self.x))
        self.y = max(margin, min(WINDOW_HEIGHT - margin, self.y))

        # Append to path history if significantly different
        if (not self.path_history) or (abs(self.x - self.path_history[-1][0]) > 2 or abs(self.y - self.path_history[-1][1]) > 2):
            self.path_history.append((self.x, self.y))
            if len(self.path_history) > self.max_history:
                self.path_history.pop(0)

    def stop(self):
        self.speed_effort = 0.0
        self.yaw_effort = 0.0

    def draw(self, screen):
        # Draw vehicle as triangle pointing to heading
        size = 20
        p_front = (self.x + size * math.cos(self.heading), self.y - size * math.sin(self.heading))
        p_left = (self.x + size * 0.5 * math.cos(self.heading + 2.5), self.y - size * 0.5 * math.sin(self.heading + 2.5))
        p_right = (self.x + size * 0.5 * math.cos(self.heading - 2.5), self.y - size * 0.5 * math.sin(self.heading - 2.5))
        points = [p_front, p_left, p_right]
        pygame.draw.polygon(screen, BLUE, points)
        pygame.draw.polygon(screen, WHITE, points, 2)

        # heading line
        end_x = self.x + 30 * math.cos(self.heading)
        end_y = self.y - 30 * math.sin(self.heading)
        pygame.draw.line(screen, CYAN, (self.x, self.y), (end_x, end_y), 2)


class MovementSimulatorNode(Node):
    def __init__(self):
        super().__init__('movement_simulator_node')

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("ROS2 Movement Simulator Twin")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)

        # Simulation State
        # Start at Lintasan A Start Zone (Bottom Right of Left Arena), facing UP
        start_x = LANE_MARGIN_X + LANE_WIDTH - 120
        start_y = LANE_MARGIN_Y + LANE_HEIGHT - 60
        self.vehicle = Vehicle(start_x, start_y)
        self.vehicle.heading = math.pi / 2  # Face UP

        # Waypoints loaded from file (replayed path)
        self.loaded_waypoints_path = []  # list of (x,y) screen coords from waypoints.txt
        self.last_waypoints_mtime = 0.0

        # Path coming from ROS recorded_path topic (separate visual)
        self.ros_recorded_path = []  # list of (x,y) screen coords from ROS message

        # Setup ROS pubs/subs
        self._setup_communication()

        # On init, attempt to load waypoints
        self.load_waypoints_from_file()

        # Precompute buoy positions for both lanes
        self.buoys = []
        for x, y, color in BUOY_POSITIONS:
            self.buoys.append({'x': x, 'y': y, 'color': color})

        self.get_logger().info("Movement Simulator Node Initialized")

    def _setup_communication(self):
        # Subscribe to efforts from Controller
        self.yaw_effort_sub = Topic.yaw_effort.createSubscriber(self, self._yaw_effort_callback)
        self.speed_effort_sub = Topic.speed_effort.createSubscriber(self, self._speed_effort_callback)

        # Publish manual controls (WASD)
        self.manual_yaw_pub = Topic.manual_yaw.createPublisher(self)
        self.manual_speed_pub = Topic.manual_speed.createPublisher(self)

        # Publish simulated sensor data (Pixhawk)
        self.pixhawk_pub = Topic.pixhawk.createPublisher(self)

        # Also publish RC5/RC6 from keyboard for testing
        self.rc5_pub = Topic.rc5.createPublisher(self)

        # Publish DSC
        self.dsc_pub = Topic.dsc.createPublisher(self)

    # callbacks
    def _yaw_effort_callback(self, msg: Float64):
        try:
            self.vehicle.yaw_effort = float(msg.data)
        except Exception:
            pass

    def _speed_effort_callback(self, msg: Float64):
        try:
            self.vehicle.speed_effort = float(msg.data)
        except Exception:
            pass

    def draw_waypoints(self):
        """Draw loaded waypoints path (from waypoints.txt) in YELLOW."""
        pygame.draw.lines(self.screen, YELLOW, False, self.loaded_waypoints_path, 3)
        pygame.draw.circle(self.screen, YELLOW, (int(self.loaded_waypoints_path[0][0]), int(self.loaded_waypoints_path[0][1])), 6)
        pygame.draw.circle(self.screen, ORANGE, (int(self.loaded_waypoints_path[-1][0]), int(self.loaded_waypoints_path[-1][1])), 6)

    def load_waypoints_from_file(self):
        """
        Read waypoints.txt where each line is a tuple:
            (yaw_effort, speed_effort, dt)
        Replay on a temporary Vehicle to generate screen coords.
        """
        try:
            if not os.path.exists("waypoints.txt"):
                return

            mtime = os.path.getmtime("waypoints.txt")
            # only reload if changed (or first load)
            if mtime <= self.last_waypoints_mtime:
                return

            self.last_waypoints_mtime = mtime

            temp_vehicle = Vehicle(WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2)
            pts = []
            with open("waypoints.txt", "r") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        yaw_effort, speed_effort, dt = ast.literal_eval(line)
                        temp_vehicle.yaw_effort = float(yaw_effort)
                        temp_vehicle.speed_effort = float(speed_effort)
                        # update physics with dt
                        temp_vehicle.update(float(dt))
                        pts.append((temp_vehicle.x, temp_vehicle.y))
                    except Exception:
                        continue

            self.loaded_waypoints_path = pts
            self.get_logger().info(f"Loaded {len(pts)} waypoints from waypoints.txt")
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints.txt: {e}")

    def check_and_reload_waypoints(self):
        try:
            if os.path.exists("waypoints.txt"):
                mtime = os.path.getmtime("waypoints.txt")
                if mtime > self.last_waypoints_mtime:
                    self.get_logger().info("waypoints.txt changed, reloading...")
                    self.load_waypoints_from_file()
        except Exception:
            pass

    def _draw_label(self, text, pos, color=WHITE):
        """Draw small label near pos (pos = (x, y))."""
        try:
            label = self.font.render(str(text), True, color)
            x, y = int(pos[0]), int(pos[1])
            self.screen.blit(label, (x + 6, y - 12))
        except Exception:
            pass

    def draw_arena(self):
        # Draw instruction text at the top
        title_font = pygame.font.Font(None, 48)
        title = title_font.render("Lintasan Lomba ASV (25x25m)", True, BLACK)
        self.screen.blit(title, (WINDOW_WIDTH//2 - title.get_width()//2, 10))

        # Arena Dimensions
        w, h = LANE_WIDTH, LANE_HEIGHT
        gap = GAP_BETWEEN_LANES
        
        # Lintasan A (Left)
        ax, ay = LANE_MARGIN_X, LANE_MARGIN_Y
        # Lintasan B (Right)
        bx, by = LANE_MARGIN_X + w + gap, LANE_MARGIN_Y

        # Draw backgrounds
        # Lintasan A
        pygame.draw.rect(self.screen, WHITE, (ax, ay, w, h))
        pygame.draw.rect(self.screen, BLACK, (ax, ay, w, h), 3)
        # Lintasan B
        pygame.draw.rect(self.screen, WHITE, (bx, by, w, h))
        pygame.draw.rect(self.screen, BLACK, (bx, by, w, h), 3)

        # Draw Grid (5m grid = 150px)
        grid_color = (200, 220, 255)
        grid_spacing = 150
        for x in range(ax, ax + w + 1, grid_spacing): 
            pygame.draw.line(self.screen, grid_color, (x, ay), (x, ay + h), 1)
        for y in range(ay, ay + h + 1, grid_spacing):
            pygame.draw.line(self.screen, grid_color, (ax, y), (ax + w, y), 1)
            
        for x in range(bx, bx + w + 1, grid_spacing): 
            pygame.draw.line(self.screen, grid_color, (x, by), (x, by + h), 1)
        for y in range(by, by + h + 1, grid_spacing):
            pygame.draw.line(self.screen, grid_color, (bx, y), (bx + w, y), 1)

        # Draw Start/Finish Zones (3m = 90px)
        # Lintasan A: Start/Finish Red Box at Bottom Right
        sf_size = 90
        pygame.draw.rect(self.screen, RED, (ax + w - sf_size - 60, ay + h - sf_size, sf_size, sf_size))
        
        # Lintasan B: Start/Finish Green Box at Bottom Left
        pygame.draw.rect(self.screen, GREEN, (bx + 60, by + h - sf_size, sf_size, sf_size))

        # Draw Arrows (Yellow)
        arrow_color = ORANGE
        def draw_arrow(start, end, width=8):
            pygame.draw.line(self.screen, arrow_color, start, end, width)
            # simple arrow head
            angle = math.atan2(end[1]-start[1], end[0]-start[0])
            size = 30
            p1 = (end[0] - size*math.cos(angle - 0.5), end[1] - size*math.sin(angle - 0.5))
            p2 = (end[0] - size*math.cos(angle + 0.5), end[1] - size*math.sin(angle + 0.5))
            pygame.draw.polygon(self.screen, arrow_color, [end, p1, p2])

        # Lintasan A Arrows (CCW-ish)
        # Start (Up)
        draw_arrow((ax + w - 120, ay + h - 150), (ax + w - 120, ay + h - 300))
        # Top (Left)
        draw_arrow((ax + w - 150, ay + 120), (ax + w - 300, ay + 120))
        # Left (Down)
        draw_arrow((ax + 120, ay + 150), (ax + 120, ay + 300))
        # Bottom (Right)
        draw_arrow((ax + 150, ay + h - 120), (ax + 300, ay + h - 120))

        # Lintasan B Arrows (CW-ish)
        # Start (Up)
        draw_arrow((bx + 120, by + h - 150), (bx + 120, by + h - 300))
        # Top (Right)
        draw_arrow((bx + 150, by + 120), (bx + 300, by + 120))
        # Right (Down)
        draw_arrow((bx + w - 120, by + 150), (bx + w - 120, by + 300))
        # Bottom (Left)
        draw_arrow((bx + w - 150, by + h - 120), (bx + w - 300, by + h - 120))

        # Draw Dashed Path (Approximate)
        def draw_rounded_rect_dashed(rect, color, width=3):
            x, y, w, h = rect
            r = 120 # radius scaled up
            # points along the rounded rect
            pts = []
            # Top line
            for i in range(x+r, x+w-r, 20): pts.append((i, y))
            # Top Right corner
            for i in range(0, 90, 10):
                ang = math.radians(-90 + i)
                pts.append((x+w-r + r*math.cos(ang), y+r + r*math.sin(ang)))
            # Right line
            for i in range(y+r, y+h-r, 20): pts.append((x+w, i))
            # Bottom Right corner
            for i in range(0, 90, 10):
                ang = math.radians(i)
                pts.append((x+w-r + r*math.cos(ang), y+h-r + r*math.sin(ang)))
            # Bottom line
            for i in range(x+w-r, x+r, -20): pts.append((i, y+h))
            # Bottom Left corner
            for i in range(0, 90, 10):
                ang = math.radians(90 + i)
                pts.append((x+r + r*math.cos(ang), y+h-r + r*math.sin(ang)))
            # Left line
            for i in range(y+h-r, y+r, -20): pts.append((x, i))
            # Top Left corner
            for i in range(0, 90, 10):
                ang = math.radians(180 + i)
                pts.append((x+r + r*math.cos(ang), y+r + r*math.sin(ang)))
            
            # Draw dashed
            for k in range(0, len(pts)-1, 2):
                if k+1 < len(pts):
                    pygame.draw.line(self.screen, color, pts[k], pts[k+1], width)

        draw_rounded_rect_dashed((ax+120, ay+120, w-240, h-240), BLACK)
        draw_rounded_rect_dashed((bx+120, by+120, w-240, h-240), BLACK)

        # Draw buoys (keep existing buoy layout but only draw those within arena)
        for buoy in self.buoys:
            bx_pos, by_pos = buoy['x'], buoy['y']
            color = BUOY_COLOR_RED if buoy['color'] == 'red' else BUOY_COLOR_GREEN
            pygame.gfxdraw.filled_circle(self.screen, int(bx_pos), int(by_pos), BUOY_RADIUS, color)
            pygame.gfxdraw.aacircle(self.screen, int(bx_pos), int(by_pos), BUOY_RADIUS, (0,0,0))

        # Labels
        label_font = pygame.font.Font(None, 48)
        la = label_font.render("LINTASAN-A", True, WHITE)
        lb = label_font.render("LINTASAN-B", True, WHITE)
        # Red pill for A
        pygame.draw.rect(self.screen, RED, (ax + w//2 - 100, ay + h + 20, 200, 50), border_radius=25)
        self.screen.blit(la, (ax + w//2 - 90, ay + h + 30))
        # Green pill for B
        pygame.draw.rect(self.screen, GREEN, (bx + w//2 - 100, by + h + 20, 200, 50), border_radius=25)
        self.screen.blit(lb, (bx + w//2 - 90, by + h + 30))

    def draw_camera_cone(self):
        """Draw a triangular camera FOV cone from the vehicle."""
        x, y = self.vehicle.x, self.vehicle.y
        heading = self.vehicle.heading
        left_angle = heading - math.radians(CAMERA_FOV_DEG / 2)
        right_angle = heading + math.radians(CAMERA_FOV_DEG / 2)
        left_pt = (x + CAMERA_RANGE * math.cos(left_angle), y - CAMERA_RANGE * math.sin(left_angle))
        right_pt = (x + CAMERA_RANGE * math.cos(right_angle), y - CAMERA_RANGE * math.sin(right_angle))
        # Draw filled triangle (semi-transparent blue)
        s = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
        pygame.draw.polygon(s, (100, 100, 255, 60), [(x, y), left_pt, right_pt])
        self.screen.blit(s, (0, 0))
        # Draw outline
        pygame.draw.polygon(self.screen, (100, 100, 255), [(x, y), left_pt, right_pt], 2)

    def buoys_in_camera_view(self):
        """Return buoys inside the camera cone."""
        x, y = self.vehicle.x, self.vehicle.y
        heading = self.vehicle.heading
        result = []
        for buoy in self.buoys:
            dx = buoy['x'] - x
            dy = y - buoy['y']  # screen y axis down
            dist = math.hypot(dx, dy)
            if dist > CAMERA_RANGE:
                continue
            angle = math.atan2(dy, dx)
            rel_angle = (angle - heading + math.pi*3) % (2*math.pi) - math.pi
            if abs(rel_angle) <= math.radians(CAMERA_FOV_DEG/2):
                result.append((buoy, dist, rel_angle))
        return result

    def compute_dsc(self):
        """Compute DSC (yaw effort to keep heading between 2 nearest buoys in FOV)."""
        visible_buoys = self.buoys_in_camera_view()
        if len(visible_buoys) == 0:
            return 0.0

        # Determine Arena (A is left, B is right)
        # Split point is roughly the gap between lanes
        split_x = LANE_MARGIN_X + LANE_WIDTH + GAP_BETWEEN_LANES / 2
        arena = "A" if self.vehicle.x < split_x else "B"
        pid_adjust = 200.0

        if len(visible_buoys) == 1:
            b = visible_buoys[0][0]  # The buoy dict
            color = b['color']  # 'red' or 'green'
            
            if color == 'green':
                 # Green: A -> Left (-), B -> Right (+)
                 return -pid_adjust if arena == "A" else pid_adjust
            elif color == 'red':
                 # Red: A -> Right (+), B -> Left (-)
                 return pid_adjust if arena == "A" else -pid_adjust

        # Find the two buoys closest to the heading direction (smallest |rel_angle|)
        visible_buoys.sort(key=lambda b: abs(b[2]))
        b1, b2 = visible_buoys[0], visible_buoys[1]
        # Get their positions
        bx1, by1 = b1[0]['x'], b1[0]['y']
        bx2, by2 = b2[0]['x'], b2[0]['y']
        # Compute the center line between buoys
        mx, my = (bx1 + bx2)/2, (by1 + by2)/2
        # Desired heading: from vehicle to midpoint
        dx = mx - self.vehicle.x
        dy = self.vehicle.y - my
        desired_heading = math.atan2(dy, dx)
        # Heading error (normalize to [-pi, pi])
        err = (desired_heading - self.vehicle.heading + math.pi*3) % (2*math.pi) - math.pi
        # Convert to DSC effort in range [-300, 300]
        dsc = max(-300, min(300, err / math.radians(60) * 300))
        return dsc

    def process_logic(self, dt):
        # check file changes
        self.check_and_reload_waypoints()

        # Publish manual controls based on WASD
        keys = pygame.key.get_pressed()
        manual_yaw = 0.0
        manual_speed = 0.0
        if keys[pygame.K_w]: manual_speed += 300.0
        if keys[pygame.K_s]: manual_speed -= 300.0
        if keys[pygame.K_d]: manual_yaw += 300.0
        if keys[pygame.K_a]: manual_yaw -= 300.0

        yaw_msg = Float64()
        yaw_msg.data = float(manual_yaw)
        self.manual_yaw_pub.publish(yaw_msg)

        speed_msg = Float64()
        speed_msg.data = float(manual_speed)
        self.manual_speed_pub.publish(speed_msg)

        # Publish simulated Pixhawk message
        lat, lon = self.vehicle.get_lat_lon()
        heading = self.vehicle.get_heading_pixhawk()
        pix_msg = Pixhawk()
        pix_msg.lat = float(lat)
        pix_msg.lon = float(lon)
        pix_msg.msg_heading = int(heading)
        # approximate speed scalar magnitude
        pix_msg.msg_spd = float(math.hypot(self.vehicle.velocity_x, self.vehicle.velocity_y))
        try:
            self.pixhawk_pub.publish(pix_msg)
        except Exception:
            # in case of publish error, ignore
            pass

        # DSC calculation and publish
        dsc_value = self.compute_dsc()
        dsc_msg = Float64()
        dsc_msg.data = float(dsc_value)
        try:
            self.dsc_pub.publish(dsc_msg)
        except Exception:
            pass

    def reset_vehicle(self):
        """Reset vehicle to start position and clear path."""
        start_x = LANE_MARGIN_X + LANE_WIDTH - 120
        start_y = LANE_MARGIN_Y + LANE_HEIGHT - 60
        self.vehicle.x = start_x
        self.vehicle.y = start_y
        self.vehicle.heading = math.pi / 2
        self.vehicle.speed_effort = 0.0
        self.vehicle.yaw_effort = 0.0
        self.vehicle.path_history = []

    def run(self):
        try:
            while rclpy.ok():
                # Event handling
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        rclpy.shutdown()
                        return
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_q:
                            rclpy.shutdown()
                            return
                        elif event.key == pygame.K_r:
                            self.reset_vehicle()  # Reset on R
                        # Simulate RC5 via keys
                        elif event.key == pygame.K_1:
                            try:
                                self.rc5_pub.publish(Float64(data=1000.0))
                            except Exception:
                                pass
                        elif event.key == pygame.K_2:
                            try:
                                self.rc5_pub.publish(Float64(data=1500.0))
                            except Exception:
                                pass
                        elif event.key == pygame.K_3:
                            try:
                                self.rc5_pub.publish(Float64(data=2000.0))
                            except Exception:
                                pass

                # Physics update
                dt = self.clock.tick(FPS) / 1000.0
                self.vehicle.update(dt)

                # Logic & ROS spin
                self.process_logic(dt)
                rclpy.spin_once(self, timeout_sec=0)

                # Rendering
                self.screen.fill(BLACK)
                self.draw_arena()
                self.draw_camera_cone()

                # Draw loaded waypoints path (yellow)
                if len(self.loaded_waypoints_path) > 1:
                    self.draw_waypoints()

                # Draw vehicle history (blue trail)
                if len(self.vehicle.path_history) > 1:
                    try:
                        pygame.draw.lines(self.screen, (0, 100, 200), False, self.vehicle.path_history, 2)
                    except Exception:
                        pass

                # Draw vehicle
                self.vehicle.draw(self.screen)

                # UI text
                lat, lon = self.vehicle.get_lat_lon()
                heading = self.vehicle.get_heading_pixhawk()
                ui_lines = [
                    f"Lat: {lat:.6f}",
                    f"Lon: {lon:.6f}",
                    f"Heading (pixhawk): {heading:.1f}",
                    f"Effort: Spd={self.vehicle.speed_effort:.1f}, Yaw={self.vehicle.yaw_effort:.1f}",
                    "Keys: 1=RC_LOW, 2=RC_MID, 3=RC_HIGH, WASD=Manual"
                ]
                for i, ln in enumerate(ui_lines):
                    surf = self.font.render(ln, True, WHITE)
                    self.screen.blit(surf, (10, 10 + i * 22))

                pygame.display.flip()

        except Exception as e:
            self.get_logger().error(f"Runtime error: {traceback.format_exc()}")
        finally:
            try:
                pygame.quit()
            except Exception:
                pass


def main(args=None):
    try:
        rclpy.init(args=args)
        node = MovementSimulatorNode()
        node.run()
    except Exception:
        print(f"Error starting node:\n{traceback.format_exc()}")
    finally:
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
