import pygame
import math
from math import sin, cos, radians

# --- Constants ---
WIDTH, HEIGHT = 800, 600
BG_COLOR = (30, 30, 30)
SHIP_COLOR = (0, 255, 255)
TARGET_COLOR = (255, 50, 50)
UI_BG_COLOR = (50, 50, 60)
BTN_COLOR = (100, 100, 100)
BTN_HOVER_COLOR = (150, 150, 150)
TEXT_COLOR = (255, 255, 255)
FPS = 60

# --- The Logic Function ---
def turner(ship_coor, heading, goal):
    goal_v_x = goal[0] - ship_coor[0]
    goal_v_y = goal[1] - ship_coor[1]

    ship_v_x = cos(radians(heading))
    ship_v_y = sin(radians(heading))

    # Math check: Heading - 90 is the "Right" vector in this coordinate space
    right_heading = heading - 90
    right_v_x = cos(radians(right_heading))
    right_v_y = sin(radians(right_heading))

    magnitude = ship_v_x * goal_v_x + ship_v_y * goal_v_y
    
    # Calculate dot product against the Right Vector to determine side
    sign_dot_product = goal_v_x * right_v_x + goal_v_y * right_v_y
    sign = 1 if sign_dot_product >= 0 else -1

    return sign

# --- UI Button Class ---
class Button:
    def __init__(self, x, y, w, h, text, action):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.action = action
        self.font = pygame.font.SysFont("Arial", 20, bold=True)
        self.hovered = False

    def draw(self, surface):
        color = BTN_HOVER_COLOR if self.hovered else BTN_COLOR
        pygame.draw.rect(surface, color, self.rect, border_radius=5)
        pygame.draw.rect(surface, (200, 200, 200), self.rect, 2, border_radius=5)
        
        text_surf = self.font.render(self.text, True, TEXT_COLOR)
        text_rect = text_surf.get_rect(center=self.rect.center)
        surface.blit(text_surf, text_rect)

    def check_hover(self, mouse_pos):
        self.hovered = self.rect.collidepoint(mouse_pos)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.hovered:
                self.action()

# --- Ship Class ---
class Ship:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.heading = 0
        self.speed = 3.0      # Default Speed
        self.turn_speed = 3.0 # Default Turn Speed
        self.radius = 15

    def update(self, target):
        if target:
            dist_x = target[0] - self.x
            dist_y = target[1] - self.y
            distance = math.sqrt(dist_x**2 + dist_y**2)

            if distance < 5:
                return

            # Get turn direction (-1 or 1)
            turn_direction = turner((self.x, self.y), self.heading, target)

            # Apply turn speed
            # Inverted subtraction because of the -90 vector logic vs screen coords
            self.heading -= turn_direction * self.turn_speed 

            # Move forward
            self.x += math.cos(math.radians(self.heading)) * self.speed
            self.y += math.sin(math.radians(self.heading)) * self.speed

    def draw(self, surface):
        angle_rad = math.radians(self.heading)
        tip_x = self.x + math.cos(angle_rad) * 20
        tip_y = self.y + math.sin(angle_rad) * 20
        left_wing_x = self.x + math.cos(angle_rad + 2.5) * 15
        left_wing_y = self.y + math.sin(angle_rad + 2.5) * 15
        right_wing_x = self.x + math.cos(angle_rad - 2.5) * 15
        right_wing_y = self.y + math.sin(angle_rad - 2.5) * 15

        pygame.draw.polygon(surface, SHIP_COLOR, [(tip_x, tip_y), (left_wing_x, left_wing_y), (right_wing_x, right_wing_y)])

# --- Main Execution ---

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Ship Steering & Speed Control")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 16)

    ship = Ship(WIDTH // 2, HEIGHT // 2)
    target_pos = None

    # --- Button Actions ---
    def inc_speed(): ship.speed = min(ship.speed + 0.5, 15.0)
    def dec_speed(): ship.speed = max(ship.speed - 0.5, 0.5)
    def inc_turn(): ship.turn_speed = min(ship.turn_speed + 0.5, 10.0)
    def dec_turn(): ship.turn_speed = max(ship.turn_speed - 0.5, 0.5)

    # --- Create Buttons ---
    # Layout: x, y, width, height
    btn_speed_down = Button(20, HEIGHT - 100, 40, 30, "-", dec_speed)
    btn_speed_up   = Button(130, HEIGHT - 100, 40, 30, "+", inc_speed)
    
    btn_turn_down  = Button(20, HEIGHT - 50, 40, 30, "-", dec_turn)
    btn_turn_up    = Button(130, HEIGHT - 50, 40, 30, "+", inc_turn)

    buttons = [btn_speed_down, btn_speed_up, btn_turn_down, btn_turn_up]

    running = True
    while running:
        mouse_pos = pygame.mouse.get_pos()

        # 1. Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # Check Button Clicks
            for btn in buttons:
                btn.handle_event(event)

            # Check Map Clicks (Only if not clicking UI area)
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                # Only set target if we aren't clicking a button
                clicked_button = False
                for btn in buttons:
                    if btn.rect.collidepoint(event.pos):
                        clicked_button = True
                        break
                
                if not clicked_button:
                    target_pos = event.pos

        # 2. Updates
        for btn in buttons:
            btn.check_hover(mouse_pos)
        
        ship.update(target_pos)

        # 3. Drawing
        screen.fill(BG_COLOR)

        # Draw Target
        if target_pos:
            pygame.draw.circle(screen, TARGET_COLOR, target_pos, 5)
            pygame.draw.line(screen, (60, 60, 60), (ship.x, ship.y), target_pos, 1)

        ship.draw(screen)

        # --- UI Area ---
        # Draw UI Background panel
        ui_rect = pygame.Rect(10, HEIGHT - 130, 180, 120)
        pygame.draw.rect(screen, UI_BG_COLOR, ui_rect, border_radius=10)
        pygame.draw.rect(screen, (100, 100, 100), ui_rect, 2, border_radius=10)

        # Draw Buttons
        for btn in buttons:
            btn.draw(screen)

        # Draw Value Text
        speed_text = font.render(f"Speed: {ship.speed:.1f}", True, TEXT_COLOR)
        screen.blit(speed_text, (70, HEIGHT - 95))

        turn_text = font.render(f"Turn: {ship.turn_speed:.1f}", True, TEXT_COLOR)
        screen.blit(turn_text, (70, HEIGHT - 45))

        # Instructions
        info_text = font.render("Click Map to Set Target", True, (150, 150, 150))
        screen.blit(info_text, (10, 10))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()

if __name__ == "__main__":
    main()