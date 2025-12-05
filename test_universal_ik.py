"""
Test the Universal IK Solver with any robot configuration.
"""

import numpy as np
import matplotlib.pyplot as plt
from test_robot import create_test_robot_simple, create_test_robot_6dof, create_puma_like_robot
from core.ik_solver import UniversalIKSolver, solve_ik
from ui.robot_ui import RobotUI


def test_simple():
    """Simple IK test with visualization."""
    print("=== Universal IK Solver Test ===\n")
    
    # Create robot
    print("Choose robot:")
    print("1. Simple 3-DOF planar arm")
    print("2. 6-DOF industrial arm")
    print("3. PUMA-like robot")
    
    choice = input("Enter choice (1-3) [1]: ").strip() or "1"
    
    if choice == "2":
        model = create_test_robot_6dof()
        print("\n✓ Created 6-DOF robot")
    elif choice == "3":
        model = create_puma_like_robot()
        print("\n✓ Created PUMA-like robot")
    else:
        model = create_test_robot_simple()
        print("\n✓ Created 3-DOF robot")
    
    # Get current end-effector position
    current_pos = model.get_joint_position(model.n_joints - 1)
    print(f"\nCurrent end-effector position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
    
    # Define target (offset from current position)
    target = current_pos + np.array([0.3, 0.2, 0.1])
    print(f"Target position: [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
    
    # Check reachability
    solver = UniversalIKSolver(model)
    reachable, distance = solver.check_reachability(target)
    print(f"\nReachability check: {'✓ Likely reachable' if reachable else '✗ May be out of reach'}")
    print(f"Distance from base: {distance:.3f} m")
    
    # Solve IK
    print("\n" + "="*60)
    print("Solving IK...")
    print("="*60)
    
    success, joint_angles, info = solver.solve(
        target,
        max_iterations=200,
        tolerance=0.001,
        verbose=True
    )
    
    if success:
        print("\n🎯 SUCCESS! IK solution found!")
        
        # Apply solution
        model.set_all_thetas(joint_angles)
        
        # Verify
        final_pos = model.get_joint_position(model.n_joints - 1)
        error = np.linalg.norm(target - final_pos)
        
        print(f"\nVerification:")
        print(f"  Target:   [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
        print(f"  Achieved: [{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}]")
        print(f"  Error:    {error*1000:.3f} mm")
        
    else:
        print("\n⚠️  Did not fully converge, but here's the best solution:")
        model.set_all_thetas(joint_angles)
    
    # Plot convergence
    plot_convergence(info['error_history'])
    
    # Visualize robot with target
    print("\nStarting visualization...")
    ui = RobotUI(model)
    
    # Add target marker
    ui.ax.scatter(*target, color='red', s=200, marker='*', 
                  label='Target', edgecolors='black', linewidths=2)
    ui.ax.legend()
    
    ui.run()


def test_multiple_targets():
    """Test IK with multiple sequential targets."""
    print("=== Multiple Targets Test ===\n")
    
    model = create_test_robot_simple()
    solver = UniversalIKSolver(model)
    
    # Define a path of targets
    targets = [
        np.array([1.0, 0.5, 0.5]),
        np.array([1.2, 0.8, 0.4]),
        np.array([0.8, 1.0, 0.6]),
        np.array([0.6, 0.6, 0.3]),
        np.array([1.0, 0.2, 0.5]),
    ]
    
    print(f"Testing {len(targets)} sequential targets...\n")
    
    results = []
    
    for i, target in enumerate(targets):
        print(f"\n--- Target {i+1} ---")
        print(f"Position: [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
        
        success, joint_angles, info = solver.solve(
            target,
            max_iterations=150,
            tolerance=0.001,
            verbose=False
        )
        
        if success:
            model.set_all_thetas(joint_angles)
            final_pos = model.get_joint_position(model.n_joints - 1)
            error = np.linalg.norm(target - final_pos)
            
            print(f"✓ Success in {info['iterations']} iterations")
            print(f"  Final error: {error*1000:.2f} mm")
            print(f"  Joint angles (deg): {np.degrees(joint_angles).round(1).tolist()}")
            
            results.append({
                'target': target,
                'success': True,
                'error': error,
                'iterations': info['iterations']
            })
        else:
            print(f"✗ Failed to converge")
            results.append({
                'target': target,
                'success': False,
                'error': info['final_error'],
                'iterations': info['iterations']
            })
    
    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    success_count = sum(1 for r in results if r['success'])
    print(f"Success rate: {success_count}/{len(targets)} ({100*success_count/len(targets):.0f}%)")
    
    if success_count > 0:
        avg_error = np.mean([r['error'] for r in results if r['success']])
        avg_iters = np.mean([r['iterations'] for r in results if r['success']])
        print(f"Average error: {avg_error*1000:.2f} mm")
        print(f"Average iterations: {avg_iters:.1f}")


def test_workspace_limits():
    """Test IK at workspace boundaries."""
    print("=== Workspace Limits Test ===\n")
    
    model = create_test_robot_simple()
    solver = UniversalIKSolver(model)
    
    # Test targets at various distances
    print("Testing targets at different distances from base:\n")
    
    distances = [0.5, 1.0, 1.5, 2.0, 2.5]
    
    for dist in distances:
        target = np.array([dist, 0.0, 0.5])
        
        reachable, _ = solver.check_reachability(target)
        success, joint_angles, info = solver.solve(
            target,
            max_iterations=200,
            tolerance=0.001,
            verbose=False
        )
        
        status = "✓" if success else "✗"
        reach_str = "reachable" if reachable else "out of reach"
        
        print(f"Distance {dist:.1f}m: {status} ({reach_str})")


def plot_convergence(error_history):
    """Plot convergence history."""
    plt.figure(figsize=(10, 6))
    plt.plot(error_history, linewidth=2)
    plt.xlabel('Iteration', fontsize=12)
    plt.ylabel('Position Error (m)', fontsize=12)
    plt.title('IK Solver Convergence', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.yscale('log')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    print("\n" + "="*60)
    print("UNIVERSAL IK SOLVER - TEST SUITE")
    print("Works for ANY robot arm configuration!")
    print("="*60 + "\n")
    
    print("Choose test mode:")
    print("1. Simple test with visualization")
    print("2. Multiple sequential targets")
    print("3. Workspace limits test")
    
    choice = input("\nEnter choice (1-3) [1]: ").strip() or "1"
    
    if choice == "2":
        test_multiple_targets()
    elif choice == "3":
        test_workspace_limits()
    else:
        test_simple()