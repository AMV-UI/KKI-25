import pygame
import math

# --- Constants ---
WIDTH, HEIGHT = 800, 600
FPS = 60
BACKGROUND_COLOR = (30, 30, 30)
AGENT_COLOR = (0, 255, 200)
TARGET_COLOR = (255, 50, 50)
TEXT_COLOR = (200, 200, 200)

# Physics Constants (UNCHANGED)
MAX_SPEED = 6
MAX_FORCE = 0.2
APPROACH_RADIUS = 100

class TargetPoint:
    """
    Represents the 'Input Point' that the user can interact with.
    """
    def __init__(self, x, y):
        self.pos = pygame.math.Vector2(x, y)
        self.radius = 10
        self.dragging = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                # Check if mouse clicked inside the point
                mouse_pos = pygame.math.Vector2(event.pos)
                if self.pos.distance_to(mouse_pos) <= self.radius:
                    self.dragging = True
                else:
                    # Teleport point to click location if not dragging
                    self.pos = mouse_pos
                    
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self.dragging = False

        elif event.type == pygame.MOUSEMOTION:
            if self.dragging:
                self.pos = pygame.math.Vector2(event.pos)

    def draw(self, screen):
        # Draw the physical point
        pygame.draw.circle(screen, TARGET_COLOR, (int(self.pos.x), int(self.pos.y)), self.radius)
        # Draw the 'Approach Radius' visual guide around the point
        pygame.draw.circle(screen, TARGET_COLOR, (int(self.pos.x), int(self.pos.y)), APPROACH_RADIUS, 1)

class Agent:
    def __init__(self, x, y):
        self.pos = pygame.math.Vector2(x, y)
        self.vel = pygame.math.Vector2(0, 0)
        self.acc = pygame.math.Vector2(0, 0)
        self.r = 16

    def apply_force(self, force):
        self.acc += force

    # --- THE FUNCTION (UNCHANGED) ---
    def arrive(self, target):
        """
        Calculates steering force to seek a target.
        Slows down when close (Arrive behavior).
        """
        desired = target - self.pos
        distance = desired.length()

        if distance == 0:
            return

        desired.normalize_ip()
        
        if distance < APPROACH_RADIUS:
            m = (distance / APPROACH_RADIUS) * MAX_SPEED
            desired *= m
        else:
            desired *= MAX_SPEED

        steer = desired - self.vel
        
        if steer.length() > MAX_FORCE:
            steer.scale_to_length(MAX_FORCE)

        self.apply_force(steer)
    # --------------------------------

    def update(self):
        self.vel += self.acc
        self.pos += self.vel
        self.acc *= 0 

    def draw(self, screen):
        if self.vel.length() > 0:
            angle = self.vel.angle_to(pygame.math.Vector2(1, 0))
        else:
            angle = 0

        tip = self.pos + pygame.math.Vector2(self.r, 0).rotate(-angle)
        left = self.pos + pygame.math.Vector2(-self.r/2, -self.r/2).rotate(-angle)
        right = self.pos + pygame.math.Vector2(-self.r/2, self.r/2).rotate(-angle)

        pygame.draw.polygon(screen, AGENT_COLOR, [tip, left, right])

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Simulation: Draggable Input Point")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 18)

    # Initialize Entities
    agent = Agent(WIDTH // 2, HEIGHT // 2)
    # The 'Input' is now an object
    input_point = TargetPoint(WIDTH // 2 + 200, HEIGHT // 2)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            # Pass events to the Input Point
            input_point.handle_event(event)

        # Physics
        # We pass the input_point's position vector to the arrive function
        agent.arrive(input_point.pos)
        agent.update()

        # Drawing
        screen.fill(BACKGROUND_COLOR)

        # Draw connection line (Visualizing the vector input)
        pygame.draw.line(screen, (50, 50, 50), agent.pos, input_point.pos, 1)

        input_point.draw(screen)
        agent.draw(screen)
        
        # UI
        ui_text = font.render(f"Input Point: ({int(input_point.pos.x)}, {int(input_point.pos.y)})", True, TEXT_COLOR)
        screen.blit(ui_text, (10, 10))
        inst_text = font.render("Click or Drag the Red Point", True, TEXT_COLOR)
        screen.blit(inst_text, (10, HEIGHT - 30))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()

if __name__ == "__main__":
    main()