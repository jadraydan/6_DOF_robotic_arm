# ui/robot_ui.py
from render.mpl_renderer import MatplotRenderer
import pygame
import sys

BUTTON_W = 40
BUTTON_H = 30
ROW_SPACING = 50
FONT_SIZE = 24
DELTA_THETA = 0.05    # radians per click

class RobotUI:
    def __init__(self, model):
        pygame.init()
        self.model = model

        self.W = 600
        self.H = 100 + len(model.theta) * ROW_SPACING

        self.screen = pygame.display.set_mode((self.W, self.H))
        pygame.display.set_caption("Robot Joint Control")

        self.font = pygame.font.SysFont("consolas", FONT_SIZE)

        # Pre-build button rectangles for each joint
        self.buttons = []   # list of (minus_rect, plus_rect)
        x_minus = 100
        x_plus = 300

        for i in range(len(model.theta)):
            y = 80 + i * ROW_SPACING
            minus_rect = pygame.Rect(x_minus, y, BUTTON_W, BUTTON_H)
            plus_rect  = pygame.Rect(x_plus,  y, BUTTON_W, BUTTON_H)
            self.buttons.append((minus_rect, plus_rect))

        self.renderer = MatplotRenderer(width=500, height=500)
        self.plot_area_x = 350
        self.plot_area_y = 20

    # --------------------------------------------------

    def handle_click(self, pos):
        """Check if any +/- button was clicked."""
        for i, (minus_rect, plus_rect) in enumerate(self.buttons):
            if minus_rect.collidepoint(pos):
                self.model.theta[i] -= DELTA_THETA
                return
            if plus_rect.collidepoint(pos):
                self.model.theta[i] += DELTA_THETA
                return

    # --------------------------------------------------

    def draw(self):
        self.screen.fill((30, 30, 30))

        title = self.font.render("Joint Controls", True, (200, 200, 200))
        self.screen.blit(title, (20, 20))

        for i, (minus_rect, plus_rect) in enumerate(self.buttons):
            theta_val = self.model.theta[i]

            # label
            label = self.font.render(f"θ{i}: {theta_val:.3f} rad", True, (220, 220, 220))
            self.screen.blit(label, (20, minus_rect.y))

            # minus button
            pygame.draw.rect(self.screen, (100, 100, 250), minus_rect)
            minus_text = self.font.render("-", True, (255, 255, 255))
            self.screen.blit(minus_text, (minus_rect.x + 12, minus_rect.y + 3))

            # plus button
            pygame.draw.rect(self.screen, (100, 250, 100), plus_rect)
            plus_text = self.font.render("+", True, (255, 255, 255))
            self.screen.blit(plus_text, (plus_rect.x + 12, plus_rect.y + 3))

    # --------------------------------------------------

    def run(self):
        clock = pygame.time.Clock()

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

                if event.type == pygame.MOUSEBUTTONDOWN:
                    self.handle_click(event.pos)

            # update robot state
            T_list = self.model.compute_fk()
            self.renderer.plot_robot(T_list)
            surface = self.renderer.get_surface()
            self.screen.blit(surface, (self.plot_area_x, self.plot_area_y))

            # later: draw arm here using T_list

            self.draw()
            pygame.display.flip()
            clock.tick(60)
