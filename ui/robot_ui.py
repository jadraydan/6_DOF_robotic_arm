# ui/robot_ui.py
from render.mpl_renderer import MatplotRenderer
import pygame
import sys

BUTTON_W = 40
BUTTON_H = 30
ROW_SPACING = 50
FONT_SIZE = 24
DELTA_THETA = 0.05

class RobotUI:
    def __init__(self, model):
        pygame.init()
        self.model = model

        # FIXED WINDOW SIZE
        self.W = 1000
        self.H = 700

        self.screen = pygame.display.set_mode((self.W, self.H))
        pygame.display.set_caption("Robot Joint Control")

        self.font = pygame.font.SysFont("consolas", FONT_SIZE)

        # LEFT PANEL BUTTONS
        self.buttons = []
        x_minus = 40
        x_plus = 180

        for i in range(len(model.theta)):
            y = 120 + i * ROW_SPACING
            minus_rect = pygame.Rect(x_minus, y, BUTTON_W, BUTTON_H)
            plus_rect  = pygame.Rect(x_plus,  y, BUTTON_W, BUTTON_H)
            self.buttons.append((minus_rect, plus_rect))

        # RIGHT SIDE PLOT
        self.renderer = MatplotRenderer(width=600, height=600)
        self.plot_area_x = 350
        self.plot_area_y = 40

    # ----------------------------

    def handle_click(self, pos):
        for i, (minus_rect, plus_rect) in enumerate(self.buttons):
            if minus_rect.collidepoint(pos):
                self.model.theta[i] -= DELTA_THETA
                return
            if plus_rect.collidepoint(pos):
                self.model.theta[i] += DELTA_THETA
                return

    # ----------------------------

    def draw_controls(self):
        self.screen.fill((30, 30, 30))

        title = self.font.render("Joint Controls", True, (200, 200, 200))
        self.screen.blit(title, (20, 40))

        for i, (minus_rect, plus_rect) in enumerate(self.buttons):
            theta_val = self.model.theta[i]

            label = self.font.render(f"θ{i}: {theta_val:.3f} rad",
                                     True, (220, 220, 220))
            self.screen.blit(label, (20, minus_rect.y))

            pygame.draw.rect(self.screen, (100, 100, 250), minus_rect)
            minus_text = self.font.render("-", True, (255, 255, 255))
            self.screen.blit(minus_text, (minus_rect.x + 12, minus_rect.y + 3))

            pygame.draw.rect(self.screen, (100, 250, 100), plus_rect)
            plus_text = self.font.render("+", True, (255, 255, 255))
            self.screen.blit(plus_text, (plus_rect.x + 12, plus_rect.y + 3))

    # ----------------------------

    def run(self):
        clock = pygame.time.Clock()

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

                if event.type == pygame.MOUSEBUTTONDOWN:
                    self.handle_click(event.pos)

            # UPDATE FK
            T_list = self.model.compute_fk()

            # FIRST draw controls
            self.draw_controls()

            # THEN draw updated plot
            self.renderer.plot_robot(T_list)
            surface = self.renderer.get_surface()
            self.screen.blit(surface, (self.plot_area_x, self.plot_area_y))

            pygame.display.flip()
            clock.tick(60)
