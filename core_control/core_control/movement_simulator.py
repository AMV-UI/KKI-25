#!/usr/bin/env python3
"""
ROS2 Node for 2D Movement Simulator with Docking Visualization
Acts as a digital twin for testing DockingController logic.
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

# Constants (matching simulator)
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
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
YAW_TO_ANGULAR = 0.0002
PIXELS_TO_METERS = 0.1
METERS_TO_LATLON = 0.0001

# Simulation realism
GPS_UPDATE_RATE = 10  # Hz
GPS_POSITION_NOISE = 0.5
GPS_HEADING_NOISE = 5

class Vehicle:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.heading = 0.0  # Radians, 0 = right (East), CCW positive
        self.speed_effort = 0.0
        self.yaw_effort = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.angular_velocity = 0.0
        self.path_history = []
        self.max_history = 500
        
        # GPS simulation
        self.gps_x = x
        self.gps_y = y
        self.gps_heading = 0.0
        self.gps_update_timer = 0.0
        self.gps_update_interval = 1.0 / GPS_UPDATE_RATE
        
    def get_lat_lon(self, use_gps=True):
        x = self.gps_x if use_gps else self.x
        y = self.gps_y if use_gps else self.y
        lat = (WINDOW_HEIGHT / 2 - y) * PIXELS_TO_METERS * METERS_TO_LATLON
        lon = (x - WINDOW_WIDTH / 2) * PIXELS_TO_METERS * METERS_TO_LATLON
        return lat, lon
    
    def get_heading_pixhawk(self, use_gps=True):
        """Return heading in Pixhawk convention (0=North, 90=East, CW)"""
        heading = self.gps_heading if use_gps else self.heading
        heading_deg = math.degrees(heading)
        # Convert East=0 (CCW) to North=0 (CW)
        # Math: 0=E, 90=N. Pixhawk: 0=N, 90=E.
        # Formula: (90 - deg) % 360
        pixhawk_heading = (90.0 - heading_deg) % 360.0
        return pixhawk_heading

    def update(self, dt):
        # Update GPS
        self.gps_update_timer += dt
        if self.gps_update_timer >= self.gps_update_interval:
            self.gps_update_timer = 0.0
            noise_dist = random.gauss(0, GPS_POSITION_NOISE / 3)
            noise_ang = random.uniform(0, 2 * math.pi)
            self.gps_x = self.x + noise_dist * math.cos(noise_ang) / PIXELS_TO_METERS
            self.gps_y = self.y + noise_dist * math.sin(noise_ang) / PIXELS_TO_METERS
            self.gps_heading = self.heading + math.radians(random.gauss(0, GPS_HEADING_NOISE / 3))

        # Physics
        # Yaw effort: Positive = Turn Right (CW) = Negative angular velocity (CCW)
        self.angular_velocity = -self.yaw_effort * YAW_TO_ANGULAR
        self.heading += self.angular_velocity
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))
        
        speed = self.speed_effort * SPEED_TO_VELOCITY
        self.velocity_x = speed * math.cos(self.heading)
        self.velocity_y = -speed * math.sin(self.heading) # Y increases downward
        
        self.x += self.velocity_x
        self.y += self.velocity_y
        
        # Bounds
        self.x = max(20, min(WINDOW_WIDTH - 20, self.x))
        self.y = max(20, min(WINDOW_HEIGHT - 20, self.y))
        
        # Path history
        if not self.path_history or (abs(self.x - self.path_history[-1][0]) > 2 or abs(self.y - self.path_history[-1][1]) > 2):
            self.path_history.append((self.x, self.y))
            if len(self.path_history) > self.max_history:
                self.path_history.pop(0)

    def stop(self):
        self.speed_effort = 0.0
        self.yaw_effort = 0.0

    def draw(self, screen):
        size = 20
        points = [
            (self.x + size * math.cos(self.heading), self.y - size * math.sin(self.heading)),
            (self.x + size * 0.5 * math.cos(self.heading + 2.5), self.y - size * 0.5 * math.sin(self.heading + 2.5)),
            (self.x + size * 0.5 * math.cos(self.heading - 2.5), self.y - size * 0.5 * math.sin(self.heading - 2.5))
        ]
        pygame.draw.polygon(screen, BLUE, points)
        pygame.draw.polygon(screen, WHITE, points, 2)
        
        # Heading line
        end_x = self.x + 30 * math.cos(self.heading)
        end_y = self.y - 30 * math.sin(self.heading)
        pygame.draw.line(screen, CYAN, (self.x, self.y), (end_x, end_y), 2)
        
        # Path
        if len(self.path_history) > 1:
            pygame.draw.lines(screen, (0, 100, 200), False, self.path_history, 2)

import json

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
        self.vehicle = Vehicle(WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2)
        
        self.manual_yaw = 0.0
        self.manual_speed = 0.0
        
        self.recorded_path_points = [] # List of (x, y) tuples for visualization
        
        # ROS2 Communication
        self._setup_communication()
        
        self.get_logger().info("Movement Simulator Node Initialized")

    def _setup_communication(self):
        # Subscribe to efforts from Controller
        self.yaw_effort_sub = Topic.yaw_effort.createSubscriber(self, self._yaw_effort_callback)
        self.speed_effort_sub = Topic.speed_effort.createSubscriber(self, self._speed_effort_callback)
        
        # Subscribe to recorded path for visualization
        self.recorded_path_sub = Topic.recorded_path.createSubscriber(self, self._recorded_path_callback)
        
        # Publish manual controls (WASD)
        self.manual_yaw_pub = Topic.manual_yaw.createPublisher(self)
        self.manual_speed_pub = Topic.manual_speed.createPublisher(self)
        
        # Publish simulated sensor data
        self.pixhawk_pub = Topic.pixhawk.createPublisher(self)
        
        # Also publish RC5/RC6 from keyboard for testing
        self.rc5_pub = Topic.rc5.createPublisher(self)

    def _yaw_effort_callback(self, msg: Float64):
        self.vehicle.yaw_effort = msg.data

    def _speed_effort_callback(self, msg: Float64):
        self.vehicle.speed_effort = msg.data
        
    def _recorded_path_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            # data is list of [lat, lon, dt]
            # Convert lat/lon back to screen coordinates
            points = []
            for lat, lon, _ in data:
                # Inverse of get_lat_lon
                # lat = (WINDOW_HEIGHT / 2 - y) * PIXELS_TO_METERS * METERS_TO_LATLON
                # y = WINDOW_HEIGHT / 2 - lat / (PIXELS_TO_METERS * METERS_TO_LATLON)
                y = WINDOW_HEIGHT / 2 - lat / (PIXELS_TO_METERS * METERS_TO_LATLON)
                x = lon / (PIXELS_TO_METERS * METERS_TO_LATLON) + WINDOW_WIDTH / 2
                points.append((x, y))
            
            self.recorded_path_points = points
            self.get_logger().info(f"Received path with {len(points)} points for visualization.")
        except Exception as e:
            self.get_logger().error(f"Failed to parse recorded path: {e}")

    def process_logic(self, dt):
        current_lat, current_lon = self.vehicle.get_lat_lon()
        current_heading = self.vehicle.get_heading_pixhawk()
        
        # Manual Input (WASD) -> Publish to Controller
        keys = pygame.key.get_pressed()
        manual_yaw = 0.0
        manual_speed = 0.0
        
        if keys[pygame.K_w]: manual_speed += 300.0
        if keys[pygame.K_s]: manual_speed -= 300.0
        if keys[pygame.K_d]: manual_yaw += 300.0
        if keys[pygame.K_a]: manual_yaw -= 300.0
        
        # Publish manual efforts
        yaw_msg = Float64()
        yaw_msg.data = float(manual_yaw)
        self.manual_yaw_pub.publish(yaw_msg)
        
        speed_msg = Float64()
        speed_msg.data = float(manual_speed)
        self.manual_speed_pub.publish(speed_msg)
        
        # Publish Simulated Pixhawk
        pix_msg = Pixhawk()
        pix_msg.lat = current_lat
        pix_msg.lon = current_lon
        pix_msg.msg_heading = int(current_heading)
        pix_msg.msg_spd = self.vehicle.velocity_x # approx
        self.pixhawk_pub.publish(pix_msg)

    def run(self):
        while rclpy.ok():
            # Pygame Event Loop
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rclpy.shutdown()
                    return
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        rclpy.shutdown()
                        return
                    # Simulate RC5 with keys for testing
                    elif event.key == pygame.K_1: self.rc5_pub.publish(Float64(data=1000.0)) # LOW
                    elif event.key == pygame.K_2: self.rc5_pub.publish(Float64(data=1500.0)) # MID
                    elif event.key == pygame.K_3: self.rc5_pub.publish(Float64(data=2000.0)) # HIGH

            # Update Physics
            dt = self.clock.tick(FPS) / 1000.0
            self.vehicle.update(dt)
            
            # Process Logic & ROS
            self.process_logic(dt)
            rclpy.spin_once(self, timeout_sec=0)
            
            # Draw
            self.screen.fill(BLACK)
            
            # Draw Recorded Path
            if len(self.recorded_path_points) > 1:
                pygame.draw.lines(self.screen, GREEN, False, self.recorded_path_points, 3)
                # Draw start/end points
                pygame.draw.circle(self.screen, GREEN, (int(self.recorded_path_points[0][0]), int(self.recorded_path_points[0][1])), 5)
                pygame.draw.circle(self.screen, RED, (int(self.recorded_path_points[-1][0]), int(self.recorded_path_points[-1][1])), 5)

            self.vehicle.draw(self.screen)
            
            # UI Text
            ui_text = [
                f"Lat: {self.vehicle.get_lat_lon()[0]:.6f}",
                f"Lon: {self.vehicle.get_lat_lon()[1]:.6f}",
                f"Heading: {self.vehicle.get_heading_pixhawk():.1f}",
                f"Effort: Spd={self.vehicle.speed_effort:.1f}, Yaw={self.vehicle.yaw_effort:.1f}",
                "Keys: 1=Low, 2=Mid, 3=High, WASD=Move"
            ]
            for i, line in enumerate(ui_text):
                text = self.font.render(line, True, WHITE)
                self.screen.blit(text, (10, 10 + i * 25))
            
            pygame.display.flip()

        pygame.quit()

def main(args=None):
    try:
        rclpy.init(args=args)
        node = MovementSimulatorNode()
        node.run()
    except Exception as e:
        print(f"Error: {traceback.format_exc()}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
