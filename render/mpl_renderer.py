import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np
import pygame


class MatplotRenderer:
    def __init__(self, width=800, height=800):
        self.width = width
        self.height = height

        # internal clean mode: "none", "dh", "joint"
        self.view_mode = "none"

        self.fig = plt.figure(figsize=(width/100, height/100), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasAgg(self.fig)

    # -------------------------------------------------------------
    # NEW: set_view_mode() called from RobotUI
    # -------------------------------------------------------------
    def set_view_mode(self, mode):
        """
        UI sends: "none", "dh_frames", "joint_frames"
        Internally we use: "none", "dh", "joint"
        """
        if mode == "dh_frames":
            self.view_mode = "dh"
        elif mode == "joint_frames":
            self.view_mode = "joint"
        else:
            self.view_mode = "none"

    # -------------------------------------------------------------
    def plot_robot(self, T_list, model):
        self.ax.clear()

        # Draw robot links
        xs = [T[0, 3] for T in T_list]
        ys = [T[1, 3] for T in T_list]
        zs = [T[2, 3] for T in T_list]
        self.ax.plot(xs, ys, zs, marker='o')

        # Axis limits
        self.ax.set_xlim([-0.4, 0.4])
        self.ax.set_ylim([-0.4, 0.4])
        self.ax.set_zlim([0.0, 0.6])

        # Draw frames
        if self.view_mode == "dh":
            frames = model.get_dh_frames()
            self.draw_all_frames(frames)

        elif self.view_mode == "joint":
            frames = model.get_joint_frames()
            self.draw_all_frames(frames)

        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

    # -------------------------------------------------------------
    def draw_all_frames(self, frames, axis_len=0.05):
        for R, p in frames:
            x, y, z = p

            self.ax.quiver(x, y, z, R[0,0], R[1,0], R[2,0], length=axis_len, color="red")
            self.ax.quiver(x, y, z, R[0,1], R[1,1], R[2,1], length=axis_len, color="green")
            self.ax.quiver(x, y, z, R[0,2], R[1,2], R[2,2], length=axis_len, color="blue")

    # -------------------------------------------------------------
    def get_surface(self):
        self.canvas.draw()
        raw = self.canvas.buffer_rgba()
        size = self.canvas.get_width_height()
        surf = pygame.image.frombuffer(raw, size, "RGBA")
        return surf.convert_alpha()
