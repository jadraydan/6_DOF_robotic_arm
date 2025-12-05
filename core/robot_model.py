"""
Robot model holding DH parameters, offsets, and current joint angles.
"""

import numpy as np
from typing import List, Dict
from core.transformations import forward_kinematics

class RobotModel:
    def __init__(self, dh_rows: List[Dict], offsets: List[Dict]):
        """
        dh_rows: list of DH parameter dicts
        offsets: list of offset dicts (one per joint)
        """
        self.dh_rows = dh_rows
        self.offsets = offsets
        self.n_joints = len(dh_rows)
        
        # Initialize joint angles (only for variable joints)
        self.thetas = [0.0] * self.n_joints
        
        # Track which joints are variable
        self.variable_joints = [i for i, row in enumerate(dh_rows) 
                               if row["theta"] is None]
        
        # Current transforms
        self.T_dh = []
        self.T_actual = []
        self.update_transforms()
    
    def set_theta(self, joint_idx: int, value: float):
        """Set joint angle for a variable joint (in radians)."""
        if joint_idx < 0 or joint_idx >= self.n_joints:
            return
        if joint_idx in self.variable_joints:
            self.thetas[joint_idx] = value
            self.update_transforms()
    
    def set_all_thetas(self, values: List[float]):
        """Set all variable joint angles at once."""
        for i, val in enumerate(values):
            if i < len(self.thetas):
                self.thetas[i] = val
        self.update_transforms()
    
    def update_transforms(self):
        """Recompute forward kinematics."""
        self.T_dh, self.T_actual = forward_kinematics(
            self.dh_rows, 
            self.thetas, 
            self.offsets
        )
    
    def get_joint_position(self, joint_idx: int, use_actual: bool = True) -> np.ndarray:
        """Get the position of a joint (3D vector)."""
        transforms = self.T_actual if use_actual else self.T_dh
        if joint_idx < 0 or joint_idx >= len(transforms):
            return np.array([0.0, 0.0, 0.0])
        return transforms[joint_idx][0:3, 3]
    
    def get_joint_axes(self, joint_idx: int, use_actual: bool = True) -> tuple:
        """Get the x, y, z axes of a joint frame."""
        transforms = self.T_actual if use_actual else self.T_dh
        if joint_idx < 0 or joint_idx >= len(transforms):
            I = np.eye(4)
            return I[0:3, 0], I[0:3, 1], I[0:3, 2]
        
        T = transforms[joint_idx]
        x_axis = T[0:3, 0]
        y_axis = T[0:3, 1]
        z_axis = T[0:3, 2]
        return x_axis, y_axis, z_axis