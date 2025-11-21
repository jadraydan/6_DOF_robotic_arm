# 6_DOF_robotic_arm
This project aims to control a 6 dof robotic arm using both forward and inverse kinematics

# Robotic Arm 3D Visualization & DH Simulation

This project provides a Python-based 3D visualization tool for robotic arms using STL/OBJ models and Denavit–Hartenberg (DH) transformations.  
It loads each robotic arm link as a mesh, applies the correct kinematic transformations, and renders an interactive 3D scene.

The goal is to allow users to:
- Import robotic arm parts as STL/OBJ
- Apply real-world dimensions to the meshes
- Use DH parameters to control joints and simulate kinematics
- Visualize coordinate frames and transformations in real time

---

## Requirements

Before running the project, you must install the Python dependencies listed in `requirements.txt`.  
These are cross-platform and work on **Windows, Linux, and macOS**.

---

## Setup Instructions

1. **Create a virtual environment**

    Windows:
            python -m venv venv
            venv\Scripts\activate
    Linux/macOS:
            python3 -m venv venv
            source venv/bin/activate

2. **Install the required dependencies**

You should run this code on your terminal after running the activate command:
            pip install -r requirements.txt


3. **Run the application**
python main.py

Make sure you are inside the virtual environment every time you run the project.

---

## Notes

- Do **not** commit your virtual environment folder to Git.
- Add `venv/` to your `.gitignore`.
- If you update dependencies, run:

pip freeze > requirements.txt

yaml
Copy code

to keep the team in sync.

---

## Project Status

This is the initial setup.  
Further development will include:
- Loading robotic arm link meshes
- Applying correct DH transforms
- Rendering real-time coordinate frames
- Editing meshes to match manufacturer dimensions