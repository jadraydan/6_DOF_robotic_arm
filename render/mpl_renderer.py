# render/mpl_renderer.py
import pygame
import matplotlib
matplotlib.use("Agg")  # non-interactive backend for rendering to image

import matplotlib.pyplot as plt
import numpy as np
from io import BytesIO

class MatplotRenderer:
    def __init__(self, width=500, height=500):
        self.width = width
        self.height = height

        # build a figure ONCE and keep reusing it
        self.fig = plt.figure(figsize=(width / 100, height / 100), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')

    # ------------------------------------------------------

    def plot_robot(self, T_list):
        """
        T_list = list of 4x4 transforms (base->joint0, base->joint1, ...)
        Use them to draw the robot as a 3D polyline.
        """
        self.ax.clear()

        xs, ys, zs = [], [], []

        for T in T_list:
            xs.append(T[0, 3])
            ys.append(T[1, 3])
            zs.append(T[2, 3])

        # draw segments
        self.ax.plot(xs, ys, zs, marker='o')

        # auto scale
        self.ax.set_xlim([-0.3, 0.3])
        self.ax.set_ylim([-0.3, 0.3])
        self.ax.set_zlim([0.0, 0.6])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

    # ------------------------------------------------------

    def get_surface(self):
        buf = BytesIO()
        self.fig.savefig(buf, format="png", dpi=100, bbox_inches='tight')
        buf.seek(0)
        surf = pygame.image.load(buf)
        buf.close()
        return surf.convert()

