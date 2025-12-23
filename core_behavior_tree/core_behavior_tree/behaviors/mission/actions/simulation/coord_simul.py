import pygame
import math
import sys

# --- Your original function logic adapted for the simulation ---
def calculate_turn_direction(target, heading):
    dsc = target - heading
    dsc = dsc if abs(dsc) <= 180 else (360 - abs(dsc)) * (-1 if dsc > 0 else 1)

    return 1 if dsc > 0 else -1

def abcd(target, heading):
    dsc = target - heading
    dsc = dsc if abs(dsc) <= 180 else (360 - abs(dsc)) * (-1 if dsc > 0 else 1)

    return dsc


# --- Pygame Setup ---
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("DSC Turning Direction Simulation")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Center point for the compass/ship
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
RADIUS = 200

# Fonts
font = pygame.font.Font(None, 36)
small_font = pygame.font.Font(None, 24)

# Initial values
heading = 255.0  # Ship's current heading (East)
target = 180.0  # Desired target bearing (South)

# Constants for movement
TURN_SPEED = 1.0  # Degrees per key press

def draw_arrow(surface, color, center, angle_deg, length, thickness=5):
    """Draws an arrow indicating a direction (0=North, 90=East, 180=South, 270=West)."""
    # Convert angle from degrees (0-360, 0 is North/Up) to Pygame's radians (0 is East/Right)
    # 0 deg (North) = 90 deg (Pygame)
    # 90 deg (East) = 0 deg (Pygame)
    # 180 deg (South) = -90 deg (Pygame)
    # Pygame angle is 'math.radians(90 - angle_deg)'
    
    angle_rad = math.radians(90 - angle_deg)
    
    end_x = center[0] + length * math.cos(angle_rad)
    end_y = center[1] - length * math.sin(angle_rad) # Note: Pygame Y increases downwards

    pygame.draw.line(surface, color, center, (end_x, end_y), thickness)
    
    # Simple arrowhead (short lines angled back)
    arrowhead_len = 15
    arrowhead_angle = math.radians(15) # 15 degrees angle for arrowhead lines
    
    # Line 1
    point1_x = end_x - arrowhead_len * math.cos(angle_rad + arrowhead_angle)
    point1_y = end_y + arrowhead_len * math.sin(angle_rad + arrowhead_angle)
    pygame.draw.line(surface, color, (end_x, end_y), (point1_x, point1_y), thickness)
    
    # Line 2
    point2_x = end_x - arrowhead_len * math.cos(angle_rad - arrowhead_angle)
    point2_y = end_y + arrowhead_len * math.sin(angle_rad - arrowhead_angle)
    pygame.draw.line(surface, color, (end_x, end_y), (point2_x, point2_y), thickness)


def draw_compass_markings(surface, center, radius):
    """Draws N, E, S, W and the compass circle."""
    pygame.draw.circle(surface, BLACK, center, radius, 1)

    # N, E, S, W markings
    marks = [(0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')]
    
    for angle, label in marks:
        angle_rad = math.radians(90 - angle)
        
        # Position for the dot on the compass circle
        mark_x = center[0] + radius * math.cos(angle_rad)
        mark_y = center[1] - radius * math.sin(angle_rad)
        
        # Position for the text (slightly outside the circle)
        text_x = center[0] + (radius + 20) * math.cos(angle_rad)
        text_y = center[1] - (radius + 20) * math.sin(angle_rad)
        
        text_surface = small_font.render(label, True, BLACK)
        text_rect = text_surface.get_rect(center=(text_x, text_y))
        
        pygame.draw.circle(surface, BLACK, (int(mark_x), int(mark_y)), 3)
        surface.blit(text_surface, text_rect)


# --- Main Game Loop ---
running = True
clock = pygame.time.Clock()

while running:
    # Handle Events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        # Key presses to change heading and target
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                heading = (heading - TURN_SPEED) % 360
            elif event.key == pygame.K_RIGHT:
                heading = (heading + TURN_SPEED) % 360
            elif event.key == pygame.K_a:
                target = (target - TURN_SPEED) % 360
            elif event.key == pygame.K_d:
                target = (target + TURN_SPEED) % 360

    # Clamp and wrap angles
    heading = heading % 360
    target = target % 360

    # Calculate turn direction
    turn_dir = calculate_turn_direction(target, heading)
    
    # --- Drawing ---
    screen.fill(WHITE)

    # Draw compass
    draw_compass_markings(screen, (CENTER_X, CENTER_Y), RADIUS)

    # Draw Heading Arrow (BLUE)
    draw_arrow(screen, BLUE, (CENTER_X, CENTER_Y), heading, RADIUS * 0.8)
    
    # Draw Target Arrow (RED)
    draw_arrow(screen, RED, (CENTER_X, CENTER_Y), target, RADIUS)

    # --- Display Info Text ---
    
    # 1. Heading and Target values
    heading_text = font.render(f"HEADING: {heading:.1f}°", True, BLUE)
    target_text = font.render(f"TARGET: {target:.1f}°", True, RED)
    a = font.render(f"DSC: {abcd(target, heading)}", True, BLACK)

    screen.blit(a, (20, 20))
    screen.blit(heading_text, (50, 50))
    screen.blit(target_text, (50, 80))

    # 2. Turn Direction Result
    if turn_dir == 1:
        direction_text = "TURN RIGHT (STARBOARD)"
        color = GREEN
    elif turn_dir == -1:
        direction_text = "TURN LEFT (PORT)"
        color = RED
    else:
        direction_text = "ON TARGET"
        color = BLACK
        
    result_text = font.render(f"DSC Result: {direction_text} ({turn_dir})", True, color)
    screen.blit(result_text, (50, 150))
    
    # 3. Control Instructions
    controls_text = [
        "Controls:",
        "Left/Right Arrow: Change HEADING",
        "A/D Key: Change TARGET",
        "ESC/Q: Quit"
    ]
    y_offset = HEIGHT - 100
    for line in controls_text:
        text_surface = small_font.render(line, True, BLACK)
        screen.blit(text_surface, (50, y_offset))
        y_offset += 20


    # Update the display
    pygame.display.flip()

    # Cap the frame rate
    clock.tick(60)

pygame.quit()
sys.exit()