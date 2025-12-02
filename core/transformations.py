# core/transformations.py
"""
Transformations: DH -> A matrices, offset -> O matrices, and FK.

Expectations:
- dh_rows: list of dicts:
    { "theta": float | None, "alpha": float (rad), "d": float, "a": float }
  (theta == None means variable; the UI supplies values in RobotModel.theta)
- offsets: list of dicts (from setup/offset_inputs.py):
    { "tx","ty","tz","rx","ry","rz","is_identity" }
  rx,ry,rz must be in radians (setup converts if needed).

FK convention used:
    T_prev -> T_prev @ A_i -> T_prev @ A_i @ O_i (offset AFTER A_i)
Returns list of numpy 4x4 matrices: [T1, T2, ..., Tn]
"""

import math
import numpy as np
from typing import List, Dict, Optional

# -----------------------------
# DH A-matrix
# -----------------------------
def dh_transform(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
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
    Rotation order: Rz * Ry * Rx (applied right-to-left).
    """
    if off.get("is_identity", False):
        return np.eye(4, dtype=float)

    tx = off.get("tx", 0.0)
    ty = off.get("ty", 0.0)
    tz = off.get("tz", 0.0)
    rx = off.get("rx", 0.0)
    ry = off.get("ry", 0.0)
    rz = off.get("rz", 0.0)

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

    R = Rz @ Ry @ Rx

    T = np.eye(4, dtype=float)
    T[0:3, 0:3] = R
    T[0:3, 3] = [tx, ty, tz]
    return T

# -----------------------------
# Forward kinematics using dh_rows (dict form) and raw offsets (dict form)
# -----------------------------
def forward_kinematics(dh_rows: List[Dict], thetas: List[float],
                       offsets: Optional[List[Dict]] = None) -> List[np.ndarray]:
    """
    dh_rows: list of dicts {theta, alpha, d, a}
    thetas:  list of floats (current joint values) length must equal len(dh_rows)
    offsets: list of offset dicts (same length) as produced by setup.offset_inputs,
             or None -> treated as identity for all links.
    Returns:
        transforms: list of cumulative 4x4 numpy arrays T_1 .. T_n
    """
    n = len(dh_rows)
    assert len(thetas) == n, "thetas length must match dh_rows"

    if offsets is None:
        offsets = [{"is_identity": True} for _ in range(n)]
    assert len(offsets) == n, "offsets length must match dh_rows"

    # running transform from base
    T = np.eye(4, dtype=float)
    transforms: List[np.ndarray] = []

    for i in range(n):
        row = dh_rows[i]
        # dh row contains possibly a constant theta or None (variable)
        theta_val = row["theta"] if row["theta"] is not None else thetas[i]
        a = row["a"]
        d = row["d"]
        alpha = row["alpha"]

        A = dh_transform(a, d, alpha, theta_val)   # DH transform
        O = offset_row_to_matrix(offsets[i])       # mesh offset (4x4)

        # Apply A then O (offset AFTER Ai)
        T = T @ A
        T = T @ O

        transforms.append(T.copy())

    return transforms
