# core/robot_model.py
from typing import List, Dict
from core.transformations import forward_kinematics

class RobotModel:
    """
    Light-weight container for DH rows and offsets and theta state.
    dh_rows: list of dicts: {'theta': float|None, 'alpha': float (rad), 'd': float, 'a': float}
    offsets: list of dicts: {'tx','ty','tz','rx','ry','rz','is_identity'} (rx.. in radians)
    """
    def __init__(self, dh_rows: List[Dict], offsets: List[Dict]):
        assert len(dh_rows) == len(offsets), "dh_rows and offsets must have same length"
        self.dh_table = dh_rows
        self.offsets = offsets
        self.theta = [0.0] * len(dh_rows)  # UI updates this

    def set_theta(self, idx: int, value: float):
        self.theta[idx] = float(value)

    def get_theta(self) -> List[float]:
        return list(self.theta)

    def compute_fk(self):
        """
        Returns a list of 4x4 numpy transforms (T1..Tn) for current theta values.
        """
        return forward_kinematics(self.dh_table, self.theta, self.offsets)
