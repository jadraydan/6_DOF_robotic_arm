"""
Test file to quickly visualize a robot arm without manual input.
Creates a simple 3-DOF robot arm (like a SCARA or simple manipulator).
"""

import numpy as np
from core.robot_model import RobotModel
from ui.robot_ui import RobotUI

def create_test_robot_simple():
    """
    Simple 3-DOF robot arm:
    - Joint 0: Revolute base (rotates about Z)
    - Joint 1: Revolute shoulder (rotates about Z) 
    - Joint 2: Revolute elbow (rotates about Z)
    
    All angles in radians, distances in meters.
    """
    
    # DH Parameters: theta (None=variable), alpha, d, a
    dh_rows = [
        # Joint 0: Base rotation
        {
            "theta": None,    # Variable joint
            "alpha": 0.0,     # No twist
            "d": 0.5,         # Base height
            "a": 0.0          # No offset
        },
        # Joint 1: Shoulder
        {
            "theta": None,    # Variable joint
            "alpha": 0.0,     # No twist
            "d": 0.0,         # No Z offset
            "a": 1.0          # Link length 1m
        },
        # Joint 2: Elbow
        {
            "theta": None,    # Variable joint
            "alpha": 0.0,     # No twist
            "d": 0.0,         # No Z offset
            "a": 0.8          # Link length 0.8m
        }
    ]
    
    # Offsets: all identity (no physical offsets needed)
    offsets = [
        {"tx": 0.0, "ty": 0.0, "tz": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0, "is_identity": True},
        {"tx": 0.0, "ty": 0.0, "tz": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0, "is_identity": True},
        {"tx": 0.0, "ty": 0.0, "tz": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0, "is_identity": True}
    ]
    
    return RobotModel(dh_rows, offsets)


def create_test_robot_with_offsets():
    """
    3-DOF robot with physical offsets.
    Demonstrates how offsets move the actual joint location from DH frame.
    """
    
    dh_rows = [
        # Joint 0: Base
        {
            "theta": None,
            "alpha": 0.0,
            "d": 0.3,
            "a": 0.0
        },
        # Joint 1: Shoulder with 90° twist
        {
            "theta": None,
            "alpha": np.pi/2,  # 90° twist
            "d": 0.0,
            "a": 0.5
        },
        # Joint 2: Elbow
        {
            "theta": None,
            "alpha": 0.0,
            "d": 0.0,
            "a": 0.7
        }
    ]
    
    # Offsets: Joint 1 has a physical offset
    offsets = [
        # Joint 0: no offset
        {"tx": 0.0, "ty": 0.0, "tz": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0, "is_identity": True},
        # Joint 1: offset 0.2m along X, rotate 45° about Z
        {"tx": 0.2, "ty": 0.0, "tz": 0.1, "rx": 0.0, "ry": 0.0, "rz": np.pi/4, "is_identity": False},
        # Joint 2: no offset
        {"tx": 0.0, "ty": 0.0, "tz": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0, "is_identity": True}
    ]
    
    return RobotModel(dh_rows, offsets)


def create_test_robot_6dof():
    """
    6-DOF robot arm (like industrial manipulator).
    More complex with various twists and offsets.
    """
    
    dh_rows = [
        # Joint 0: Base rotation
        {"theta": None, "alpha": np.pi/2, "d": 0.4, "a": 0.0},
        # Joint 1: Shoulder
        {"theta": None, "alpha": 0.0, "d": 0.0, "a": 0.6},
        # Joint 2: Elbow
        {"theta": None, "alpha": np.pi/2, "d": 0.0, "a": 0.2},
        # Joint 3: Wrist 1
        {"theta": None, "alpha": -np.pi/2, "d": 0.5, "a": 0.0},
        # Joint 4: Wrist 2
        {"theta": None, "alpha": np.pi/2, "d": 0.0, "a": 0.0},
        # Joint 5: Wrist 3
        {"theta": None, "alpha": 0.0, "d": 0.3, "a": 0.0}
    ]
    
    # All identity offsets
    offsets = [{"is_identity": True} for _ in range(6)]
    
    return RobotModel(dh_rows, offsets)


def create_puma_like_robot():
    """
    PUMA 560-like robot parameters (classic industrial robot).
    """
    
    dh_rows = [
        # Joint 1: Base rotation
        {"theta": None, "alpha": np.pi/2, "d": 0.6718, "a": 0.0},
        # Joint 2: Shoulder
        {"theta": None, "alpha": 0.0, "d": 0.0, "a": 0.4318},
        # Joint 3: Elbow
        {"theta": None, "alpha": -np.pi/2, "d": 0.0, "a": 0.0203},
        # Joint 4: Wrist roll
        {"theta": None, "alpha": np.pi/2, "d": 0.4318, "a": 0.0},
        # Joint 5: Wrist pitch
        {"theta": None, "alpha": -np.pi/2, "d": 0.0, "a": 0.0},
        # Joint 6: Wrist yaw
        {"theta": None, "alpha": 0.0, "d": 0.0, "a": 0.0}
    ]
    
    offsets = [{"is_identity": True} for _ in range(6)]
    
    return RobotModel(dh_rows, offsets)


if __name__ == "__main__":
    print("\n=== Robot Arm Test ===")
    print("Choose a test robot:")
    print("1. Simple 3-DOF planar arm (easiest to understand)")
    print("2. 3-DOF with offsets (demonstrates offset feature)")
    print("3. 6-DOF industrial-style arm")
    print("4. PUMA 560-like robot (classic)")
    
    choice = input("Enter choice (1-4) [default: 1]: ").strip() or "1"
    
    if choice == "1":
        model = create_test_robot_simple()
        print("Created simple 3-DOF robot")
    elif choice == "2":
        model = create_test_robot_with_offsets()
        print("Created 3-DOF robot with offsets")
    elif choice == "3":
        model = create_test_robot_6dof()
        print("Created 6-DOF robot")
    elif choice == "4":
        model = create_puma_like_robot()
        print("Created PUMA-like robot")
    else:
        model = create_test_robot_simple()
        print("Invalid choice, using simple 3-DOF robot")
    
    print(f"Number of joints: {model.n_joints}")
    print(f"Variable joints: {model.variable_joints}")
    print("\nStarting UI... Move the sliders to control the robot!")
    
    ui = RobotUI(model)
    ui.run()