import sys
import math
import pygame
import rclpy    
from rclpy.node import Node
from std_msgs.msg import Float64

# --- Constants ---
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
BG_COLOR = (30, 30, 30)
SHIP_COLOR = (0, 255, 0)
OBJECT_COLOR = (255, 50, 50)
FOV_COLOR = (200, 200, 200)
DETECT_COLOR = (255, 255, 0)

FOV_ANGLE = math.radians(60)  # 60 degrees Field of View
VIEW_DISTANCE = 250
SHIP_SIZE = 20

class SimulationNode(Node):
    def __init__(self):
        super().__init__('pygame_ship_sim')
        
        # --- ROS2 Communications ---
        self.sub_yaw = self.create_subscription(
            Float64,
            '/core/motor/yaw_effort',
            self.yaw_effort_callback,
            10
        )
        
        self.sub_speed = self.create_subscription(
            Float64,
            '/core/motor/speed_effort',
            self.speed_effort_callback,
            10
        )
        
        self.pub_dsc = self.create_publisher(
            Float64,
            '/core/vision/image/dsc',
            10
        )

        # --- Simulation State ---
        self.ship_x = SCREEN_WIDTH / 2
        self.ship_y = SCREEN_HEIGHT / 2
        self.ship_yaw = 0.0  # Radians
        self.ship_speed = 0.0
        
        # Control inputs
        self.yaw_effort = 0.0
        self.speed_effort = 0.0
        
        # World Objects
        self.objects = []
        
        # Add some initial objects using the required function
        self.add_object(200, 150, 30)
        self.add_object(600, 400, 40)
        self.add_object(300, 500, 25)
        self.add_object(650, 100, 20)

    def add_object(self, x, y, size):
        """
        Function to make specific objects in the simulation world.
        """
        self.objects.append({
            'x': x,
            'y': y,
            'size': size
        })

    def yaw_effort_callback(self, msg):
        self.yaw_effort = msg.data / 225.0

    def speed_effort_callback(self, msg):
        self.speed_effort = msg.data / 225.0

    def update_physics(self):
        # simple physics model
        # yaw_effort controls turn rate directly
        turn_speed = 0.05 * self.yaw_effort
        self.ship_yaw += turn_speed
        
        # speed_effort controls velocity directly (simplified)
        # In a real ship this might be thrust, but direct mapping is smoother for simple sims
        target_speed = self.speed_effort * 5.0 # Max speed multiplier
        
        # Simple smoothing
        self.ship_speed += (target_speed - self.ship_speed) * 0.1
        
        # Update position
        self.ship_x += math.cos(self.ship_yaw) * self.ship_speed
        self.ship_y += math.sin(self.ship_yaw) * self.ship_speed
        
        # Screen wrap
        self.ship_x = self.ship_x % SCREEN_WIDTH
        self.ship_y = self.ship_y % SCREEN_HEIGHT

    def process_vision(self):
        """
        Calculates the Distance from Center (DSC) for objects in the cone.
        Publishes the DSC of the object closest to the center line.
        """
        closest_obj = None
        min_angle_diff = float('inf')
        detected_dsc = 0.0
        
        # Direction vector of the ship
        ship_dir_x = math.cos(self.ship_yaw)
        ship_dir_y = math.sin(self.ship_yaw)

        for obj in self.objects:
            dx = obj['x'] - self.ship_x
            dy = obj['y'] - self.ship_y
            
            # Distance to object center
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist > VIEW_DISTANCE:
                continue
                
            # Angle to object
            angle_to_obj = math.atan2(dy, dx)
            
            # Relative angle (Delta between look angle and object angle)
            angle_diff = angle_to_obj - self.ship_yaw
            
            # Normalize angle to -pi to pi
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            
            # Check if within FOV half-angle
            if abs(angle_diff) < (FOV_ANGLE / 2):
                
                # We found a candidate.
                # "Distance between the center of the ship cone with a viewed object"
                # This is the perpendicular distance from the centerline (dsc).
                # dsc = distance_to_obj * sin(angle_offset)
                
                current_dsc = dist * math.sin(angle_diff)
                
                # Logic: Choose the one closest to center (smallest angle deviation)
                if abs(angle_diff) < min_angle_diff:
                    min_angle_diff = abs(angle_diff)
                    closest_obj = obj
                    detected_dsc = current_dsc

        # Publish result
        
        msg = Float64()
        if closest_obj:
            msg.data = float(detected_dsc)
            # self.pub_dsc.publish(msg)
            return closest_obj, detected_dsc
        else:
            # If nothing seen, maybe publish 0 or a flag. 
            # We'll publish 0.0 implies centered, which is ambiguous, 
            # but usually vision sensors have a 'valid' flag. 
            # For this simple sim, we just won't publish or publish 0.
            pass
            return None, None

def draw_game(screen, node, closest_obj):
    screen.fill(BG_COLOR)
    
    # --- Draw Objects ---
    for obj in node.objects:
        color = OBJECT_COLOR
        if obj == closest_obj:
            color = DETECT_COLOR
        pygame.draw.circle(screen, color, (int(obj['x']), int(obj['y'])), obj['size'])

    # --- Draw Ship ---
    # Tip of the triangle
    tip_x = node.ship_x + math.cos(node.ship_yaw) * SHIP_SIZE
    tip_y = node.ship_y + math.sin(node.ship_yaw) * SHIP_SIZE
    
    # Back corners
    left_x = node.ship_x + math.cos(node.ship_yaw + 2.5) * SHIP_SIZE
    left_y = node.ship_y + math.sin(node.ship_yaw + 2.5) * SHIP_SIZE
    
    right_x = node.ship_x + math.cos(node.ship_yaw - 2.5) * SHIP_SIZE
    right_y = node.ship_y + math.sin(node.ship_yaw - 2.5) * SHIP_SIZE
    
    pygame.draw.polygon(screen, SHIP_COLOR, [(tip_x, tip_y), (left_x, left_y), (right_x, right_y)])
    
    # --- Draw Vision Cone ---
    cone_left_x = node.ship_x + math.cos(node.ship_yaw - FOV_ANGLE/2) * VIEW_DISTANCE
    cone_left_y = node.ship_y + math.sin(node.ship_yaw - FOV_ANGLE/2) * VIEW_DISTANCE
    
    cone_right_x = node.ship_x + math.cos(node.ship_yaw + FOV_ANGLE/2) * VIEW_DISTANCE
    cone_right_y = node.ship_y + math.sin(node.ship_yaw + FOV_ANGLE/2) * VIEW_DISTANCE
    
    pygame.draw.line(screen, FOV_COLOR, (node.ship_x, node.ship_y), (cone_left_x, cone_left_y), 1)
    pygame.draw.line(screen, FOV_COLOR, (node.ship_x, node.ship_y), (cone_right_x, cone_right_y), 1)
    # Connect the ends for a complete cone look
    pygame.draw.line(screen, FOV_COLOR, (cone_left_x, cone_left_y), (cone_right_x, cone_right_y), 1)

    # --- Draw Detection Line ---
    if closest_obj:
        pygame.draw.line(screen, DETECT_COLOR, (node.ship_x, node.ship_y), (closest_obj['x'], closest_obj['y']), 2)

    pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)
    sim_node = SimulationNode()

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("ROS2 Ship Vision Sim")
    clock = pygame.time.Clock()

    running = True
    while running:
        # 1. Handle Pygame Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            # Optional: Allow mouse click to add objects
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                sim_node.add_object(mx, my, 20)

        # 2. Handle ROS2 Callbacks (non-blocking)
        rclpy.spin_once(sim_node, timeout_sec=0)

        # 3. Update Simulation Logic
        sim_node.update_physics()
        closest_obj, dsc_val = sim_node.process_vision()

        # 4. Draw
        draw_game(screen, sim_node, closest_obj)

        # Cap framerate
        clock.tick(60)

    # Cleanup
    sim_node.destroy_node()
    rclpy.shutdown()
    pygame.quit()
    sys.exit()

if __name__ == '__main__':
    main()