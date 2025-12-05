"""
Interactive robot arm visualization with matplotlib.
Shows joints as cylinders aligned with rotation axis (z-axis in DH).
Includes +/- buttons for joint control and frame toggle button.
Provides access to theta values for real-time robot control.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math

class RobotUI:
    def __init__(self, robot_model):
        self.model = robot_model
        self.show_frames = True
        
        # Step size for joint increments (in radians)
        self.step_size = 0.1  # ~5.7 degrees
        
        # Create figure with space for controls
        self.fig = plt.figure(figsize=(16, 10))
        
        # Main 3D plot - shift to the right to make room for controls
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Adjust plot position - more space on the left for controls
        self.fig.subplots_adjust(left=0.20, bottom=0.05, right=0.98, top=0.95)
        
        # Initialize buttons and labels
        self.buttons = []
        self.theta_labels = []
        self._create_controls()
        
        # Frame toggle button
        self._create_frame_toggle()
        
        # Initial plot
        self.update_plot()
        
    def _create_controls(self):
        """Create +/- buttons and theta value labels for variable joints in a vertical layout."""
        n_var = len(self.model.variable_joints)
        if n_var == 0:
            return
        
        # Layout parameters - vertical on the left
        button_size = 0.025  # Smaller buttons
        label_width = 0.10
        x_start = 0.02
        y_start = 0.70  # Start from top
        y_spacing = 0.10  # Space between joints
        
        for i, joint_idx in enumerate(self.model.variable_joints):
            y_pos = y_start - i * y_spacing
            
            # Joint label at the top
            ax_joint_label = self.fig.add_axes([x_start, y_pos + 0.04, label_width, 0.025])
            ax_joint_label.axis('off')
            ax_joint_label.text(0.5, 0.5, f'Joint {joint_idx}', 
                               ha='center', va='center', fontsize=10, fontweight='bold')
            
            # Plus button (top)
            ax_plus = self.fig.add_axes([x_start + 0.005, y_pos, button_size, button_size])
            btn_plus = Button(ax_plus, '+', color='lightgreen', hovercolor='lime')
            btn_plus.label.set_fontsize(12)
            btn_plus.on_clicked(lambda event, idx=joint_idx: self._increment_theta(idx))
            
            # Theta value label (middle)
            ax_label = self.fig.add_axes([x_start + button_size + 0.010, y_pos, 
                                         label_width - 2*button_size - 0.020, button_size])
            ax_label.axis('off')
            theta_text = ax_label.text(0.5, 0.5, '0.00°', 
                                      ha='center', va='center', 
                                      fontsize=10, family='monospace',
                                      bbox=dict(boxstyle='round,pad=0.4', 
                                              facecolor='lightblue', alpha=0.6))
            self.theta_labels.append((joint_idx, theta_text))
            
            # Minus button (bottom)
            ax_minus = self.fig.add_axes([x_start + label_width - button_size - 0.005, y_pos, 
                                         button_size, button_size])
            btn_minus = Button(ax_minus, '-', color='lightcoral', hovercolor='salmon')
            btn_minus.label.set_fontsize(12)
            btn_minus.on_clicked(lambda event, idx=joint_idx: self._decrement_theta(idx))
            
            self.buttons.append((btn_minus, btn_plus))
    
    def _create_frame_toggle(self):
        """Create button to toggle frame display."""
        ax_button = self.fig.add_axes([0.80, 0.92, 0.12, 0.05])
        self.frame_button = Button(ax_button, 'Hide Frames', 
                                   color='lightyellow', hovercolor='yellow')
        self.frame_button.on_clicked(self._toggle_frames)
    
    def _increment_theta(self, joint_idx):
        """Increment joint angle by step size."""
        current = self.model.thetas[joint_idx]
        new_value = current + self.step_size
        self.model.set_theta(joint_idx, new_value)
        self._update_theta_labels()
        self.update_plot()
    
    def _decrement_theta(self, joint_idx):
        """Decrement joint angle by step size."""
        current = self.model.thetas[joint_idx]
        new_value = current - self.step_size
        self.model.set_theta(joint_idx, new_value)
        self._update_theta_labels()
        self.update_plot()
    
    def _update_theta_labels(self):
        """Update the displayed theta values."""
        for joint_idx, text_obj in self.theta_labels:
            theta_rad = self.model.thetas[joint_idx]
            theta_deg = math.degrees(theta_rad)
            text_obj.set_text(f'{theta_deg:6.2f}°')
    
    def _toggle_frames(self, event):
        """Toggle frame visibility."""
        self.show_frames = not self.show_frames
        self.frame_button.label.set_text('Show Frames' if not self.show_frames else 'Hide Frames')
        self.update_plot()
    
    def get_current_thetas(self):
        """
        Get current theta values for all joints.
        Returns: list of float (radians)
        This method is used to send commands to the actual robot.
        """
        return self.model.thetas.copy()
    
    def get_current_thetas_degrees(self):
        """
        Get current theta values in degrees.
        Returns: list of float (degrees)
        """
        return [math.degrees(theta) for theta in self.model.thetas]
    
    def set_thetas(self, thetas):
        """
        Set theta values programmatically (e.g., from robot feedback).
        Args:
            thetas: list of float (radians)
        """
        self.model.set_all_thetas(thetas)
        self._update_theta_labels()
        self.update_plot()
    
    def update_plot(self):
        """Redraw the robot arm."""
        self.ax.clear()
        
        # Set up axes
        self._setup_axes()
        
        # Plot base frame (frame 0)
        self._plot_frame(np.eye(4), label='Base', scale=0.3)
        
        # Plot links between joints
        self._plot_links()
        
        # Plot joints as cylinders
        self._plot_joints()
        
        # Plot frames if enabled
        if self.show_frames:
            for i in range(self.model.n_joints):
                self._plot_frame(self.model.T_actual[i], 
                               label=f'J{i}', 
                               scale=0.2)
        
        plt.draw()
    
    def _setup_axes(self):
        """Configure 3D axes."""
        # Calculate workspace bounds
        positions = [self.model.get_joint_position(i) for i in range(self.model.n_joints)]
        if positions:
            positions = np.array(positions)
            max_reach = np.max(np.abs(positions)) + 0.5
        else:
            max_reach = 2.0
        
        limit = max(max_reach, 1.0)
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([0, limit * 1.5])
        
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Robot Arm Visualization')
    
    def _plot_links(self):
        """Draw lines connecting joints."""
        base_pos = np.array([0.0, 0.0, 0.0])
        
        for i in range(self.model.n_joints):
            current_pos = self.model.get_joint_position(i)
            
            if i == 0:
                start_pos = base_pos
            else:
                start_pos = self.model.get_joint_position(i - 1)
            
            # Draw link
            self.ax.plot3D([start_pos[0], current_pos[0]],
                          [start_pos[1], current_pos[1]],
                          [start_pos[2], current_pos[2]],
                          'b-', linewidth=3)
    
    def _plot_joints(self):
        """Plot joints as cylinders aligned with rotation axis (z-axis)."""
        for i in range(self.model.n_joints):
            pos = self.model.get_joint_position(i)
            x_axis, y_axis, z_axis = self.model.get_joint_axes(i)
            
            # Create cylinder along z-axis of joint frame
            self._draw_cylinder(pos, z_axis, radius=0.08, height=0.15, color='red')
    
    def _draw_cylinder(self, center, axis, radius=0.1, height=0.2, 
                      color='red', resolution=20):
        """Draw a cylinder centered at 'center' along 'axis' direction."""
        # Normalize axis
        axis = axis / np.linalg.norm(axis)
        
        # Create cylinder in local coords (along z)
        theta = np.linspace(0, 2 * np.pi, resolution)
        z_local = np.linspace(-height/2, height/2, 2)
        theta_grid, z_grid = np.meshgrid(theta, z_local)
        
        x_local = radius * np.cos(theta_grid)
        y_local = radius * np.sin(theta_grid)
        
        # Find rotation matrix to align [0,0,1] with axis
        z_vec = np.array([0, 0, 1])
        R = self._rotation_matrix_from_vectors(z_vec, axis)
        
        # Transform points
        points = np.stack([x_local.flatten(), y_local.flatten(), z_grid.flatten()])
        points_transformed = R @ points
        
        x_world = points_transformed[0, :].reshape(x_local.shape) + center[0]
        y_world = points_transformed[1, :].reshape(y_local.shape) + center[1]
        z_world = points_transformed[2, :].reshape(z_grid.shape) + center[2]
        
        # Plot surface
        self.ax.plot_surface(x_world, y_world, z_world, 
                           color=color, alpha=0.7, shade=True)
        
        # Draw caps
        for z_cap in [-height/2, height/2]:
            cap_points = []
            for t in theta:
                pt_local = np.array([radius * np.cos(t), 
                                    radius * np.sin(t), 
                                    z_cap])
                pt_world = R @ pt_local + center
                cap_points.append(pt_world)
            
            cap = Poly3DCollection([cap_points], alpha=0.7, 
                                  facecolor=color, edgecolor='darkred')
            self.ax.add_collection3d(cap)
    
    def _rotation_matrix_from_vectors(self, vec1, vec2):
        """Find rotation matrix that aligns vec1 to vec2."""
        a = vec1 / np.linalg.norm(vec1)
        b = vec2 / np.linalg.norm(vec2)
        
        v = np.cross(a, b)
        c = np.dot(a, b)
        
        # Handle parallel vectors
        if np.allclose(v, 0):
            if c > 0:
                return np.eye(3)
            else:
                # 180 degree rotation
                perp = np.array([1, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1, 0])
                perp = perp - np.dot(perp, a) * a
                perp = perp / np.linalg.norm(perp)
                return 2 * np.outer(perp, perp) - np.eye(3)
        
        # Rodrigues' rotation formula
        vx = np.array([[0, -v[2], v[1]],
                      [v[2], 0, -v[0]],
                      [-v[1], v[0], 0]])
        
        R = np.eye(3) + vx + vx @ vx * (1 / (1 + c))
        return R
    
    def _plot_frame(self, T, label='', scale=0.2):
        """Plot coordinate frame axes."""
        origin = T[0:3, 3]
        x_axis = T[0:3, 0] * scale
        y_axis = T[0:3, 1] * scale
        z_axis = T[0:3, 2] * scale
        
        # Plot axes
        self.ax.quiver(origin[0], origin[1], origin[2],
                      x_axis[0], x_axis[1], x_axis[2],
                      color='r', arrow_length_ratio=0.2, linewidth=2)
        self.ax.quiver(origin[0], origin[1], origin[2],
                      y_axis[0], y_axis[1], y_axis[2],
                      color='g', arrow_length_ratio=0.2, linewidth=2)
        self.ax.quiver(origin[0], origin[1], origin[2],
                      z_axis[0], z_axis[1], z_axis[2],
                      color='b', arrow_length_ratio=0.2, linewidth=2)
        
        # Add label
        if label:
            self.ax.text(origin[0], origin[1], origin[2], 
                        label, fontsize=8, color='black')
    
    def run(self):
        """Start the interactive visualization."""
        plt.show()