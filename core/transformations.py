"""
Transformations: DH -> A matrices, offset -> O matrices, and FK.

Expectations:
- dh_rows: list of dicts:
    { "theta": float | None, "alpha": float (rad), "d": float, "a": float }
  (theta == None means variable; the UI supplies values in RobotModel.theta)
- offsets: list of dicts: { "tx","ty","tz","rx","ry","rz","is_identity" }
  rx,ry,rz must be in radians (setup converts if needed).

FK convention used:
    T_dh[i] = T_dh[i-1] @ A_i  (pure DH chain)
    T_actual[i] = T_dh[i] @ O_i  (offset AFTER DH to get actual joint frame)

Returns two lists:
    - T_dh: pure DH frames (respecting DH convention)
    - T_actual: actual physical joint frames (DH + offsets)
"""

import math
import numpy as np
from typing import List, Dict, Optional, Tuple

# -----------------------------
# DH A-matrix
# -----------------------------
def dh_transform(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    """Standard DH transformation matrix."""
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,     sa,      ca,     d    ],
        [0.0,    0.0,     0.0,    1.0   ]
    ], dtype=float)

# -----------------------------
# Build offset matrix O_i from offset dict
# -----------------------------
def offset_row_to_matrix(off: Dict) -> np.ndarray:
    """
    off: {tx,ty,tz, rx,ry,rz, is_identity}
    rx,ry,rz are radians.
    Rotation order: Rz * Ry * Rx (applied right-to-left) to match RPY input order.
    """
    if off.get("is_identity", False):
        return np.eye(4, dtype=float)

    tx = float(off.get("tx", 0.0))
    ty = float(off.get("ty", 0.0))
    tz = float(off.get("tz", 0.0))
    rx = float(off.get("rx", 0.0))
    ry = float(off.get("ry", 0.0))
    rz = float(off.get("rz", 0.0))

    # rotation matrices
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    Rx = np.array([[1, 0, 0],
                   [0, cx, -sx],
                   [0, sx, cx]], dtype=float)
    Ry = np.array([[cy, 0, sy],
                   [0, 1, 0],
                   [-sy,0, cy]], dtype=float)
    Rz = np.array([[cz, -sz, 0],
                   [sz, cz,  0],
                   [0,  0,   1]], dtype=float)

    # R = Rz * Ry * Rx (applies Rx, then Ry, then Rz)
    R = Rz @ Ry @ Rx

    T = np.eye(4, dtype=float)
    T[0:3, 0:3] = R
    T[0:3, 3] = [tx, ty, tz]
    return T

# -----------------------------
# Forward kinematics - returns BOTH DH frames and actual frames
# -----------------------------
def forward_kinematics(dh_rows: List[Dict], thetas: List[float],
                       offsets: Optional[List[Dict]] = None,
                       debug: bool = False) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """
    dh_rows: list of dicts {theta, alpha, d, a}
    thetas:  list of floats (current joint values) length must equal len(dh_rows)
    offsets: list of offset dicts (same length), or None -> identity for all links.

    Returns:
        (T_dh, T_actual):
            T_dh: list of pure DH cumulative transforms [T1_dh, T2_dh, ..., Tn_dh]
            T_actual: list of actual joint frames [T1_actual, T2_actual, ..., Tn_actual]
                      where Ti_actual = Ti_dh @ Oi
    """
    n = len(dh_rows)
    assert len(thetas) == n, "thetas length must match dh_rows"

    if offsets is None:
        offsets = [{"is_identity": True} for _ in range(n)]
    assert len(offsets) == n, "offsets length must match dh_rows"

    # running transform from base (pure DH)
    T_dh_cumulative = np.eye(4, dtype=float)
    T_dh_list: List[np.ndarray] = []
    T_actual_list: List[np.ndarray] = []

    for i in range(n):
        row = dh_rows[i]
        # dh row contains possibly a constant theta or None (variable)
        theta_val = row["theta"] if row["theta"] is not None else thetas[i]
        a = float(row["a"])
        d = float(row["d"])
        alpha = float(row["alpha"])

        # Pure DH transformation
        A = dh_transform(a, d, alpha, theta_val)
        T_dh_cumulative = T_dh_cumulative @ A
        
        # Offset transformation
        O = offset_row_to_matrix(offsets[i])
        
        # Actual joint frame = DH frame + offset
        T_actual = T_dh_cumulative @ O

        if debug:
            print(f"=== Joint {i} ===")
            print(f"theta = {theta_val:.6f} rad")
            np.set_printoptions(precision=6, suppress=True)
            print("A =\n", A)
            print("O =\n", O)
            print("T_dh_cumulative =\n", T_dh_cumulative)
            print("T_actual =\n", T_actual)
            print("-" * 40)

        T_dh_list.append(T_dh_cumulative.copy())
        T_actual_list.append(T_actual.copy())

    return T_dh_list, T_actual_list