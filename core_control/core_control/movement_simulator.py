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
YAW_TO_ANGULAR = 0.0002
PIXELS_TO_METERS = 0.1
METERS_TO_LATLON = 0.00001

# GPS simulation
GPS_UPDATE_RATE = 50  # Hz
GPS_POSITION_NOISE = 0.0
GPS_HEADING_NOISE = 0.0


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

        # path history of vehicle (screen coords)
        self.path_history = []
        self.max_history = 500

        # Simulated GPS values (noisy)
        self.gps_x = float(x)
        self.gps_y = float(y)
        self.gps_heading = self.heading
        self.gps_update_timer = 0.0
        self.gps_update_interval = 1.0 / GPS_UPDATE_RATE

    def get_lat_lon(self, use_gps=True):
        """
        Convert current screen coordinates to lat/lon-like values.
        lat: increases upward, lon: increases rightward.
        """
        x = self.gps_x if use_gps else self.x
        y = self.gps_y if use_gps else self.y
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
        self.vehicle = Vehicle(WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2)

        # Waypoints loaded from file (replayed path)
        self.loaded_waypoints_path = []  # list of (x,y) screen coords from waypoints.txt
        self.last_waypoints_mtime = 0.0

        # Path coming from ROS recorded_path topic (separate visual)
        self.ros_recorded_path = []  # list of (x,y) screen coords from ROS message

        # Setup ROS pubs/subs
        self._setup_communication()

        # On init, attempt to load waypoints
        self.load_waypoints_from_file()

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
