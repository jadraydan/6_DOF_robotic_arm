"""
Universal Inverse Kinematics Solver using Jacobian Method
Works for ANY robot arm configuration (2-DOF, 3-DOF, 6-DOF, etc.)

Uses damped least-squares method (Levenberg-Marquardt style)
to solve for joint angles given target position.
"""

import numpy as np
from typing import Tuple, List, Optional
from core.transformations import forward_kinematics


class UniversalIKSolver:
    """
    IK Solver that works for any robot arm using numerical Jacobian.
    """
    
    def __init__(self, robot_model):
        """
        Initialize IK solver.
        
        Args:
            robot_model: RobotModel instance
        """
        self.model = robot_model
        self.n_joints = robot_model.n_joints
        
    def solve(self,
              target_pos: np.ndarray,
              max_iterations: int = 200,
              tolerance: float = 0.001,
              lambda_damping: float = 0.1,
              step_size: float = 0.5,
              verbose: bool = False) -> Tuple[bool, List[float], dict]:
        """
        Solve inverse kinematics for target position.
        
        Args:
            target_pos: Target position [x, y, z] in meters
            max_iterations: Maximum number of iterations
            tolerance: Position error tolerance in meters (default: 1mm)
            lambda_damping: Damping factor for pseudo-inverse (default: 0.1)
            step_size: Step size for joint updates (0-1, default: 0.5)
            verbose: Print iteration details
            
        Returns:
            success: True if solution found within tolerance
            joint_angles: List of joint angles in radians (best solution found)
            info: Dictionary with convergence information
                  {'iterations': int, 'final_error': float, 'error_history': list}
        """
        # Start from current joint configuration
        thetas = np.array(self.model.thetas.copy())
        
        # Store error history for analysis
        error_history = []
        
        if verbose:
            print(f"\n{'='*60}")
            print(f"Starting IK Solver")
            print(f"Target: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
            print(f"Initial config (deg): {np.degrees(thetas).round(2).tolist()}")
            print(f"{'='*60}\n")
        
        for iteration in range(max_iterations):
            # Compute forward kinematics to get current end-effector position
            T_dh, T_actual = forward_kinematics(
                self.model.dh_rows,
                thetas.tolist(),
                self.model.offsets
            )
            
            # Get current end-effector position
            current_pos = T_actual[-1][0:3, 3]
            
            # Compute position error vector
            error_vector = target_pos - current_pos
            error_norm = np.linalg.norm(error_vector)
            error_history.append(error_norm)
            
            if verbose and iteration % 10 == 0:
                print(f"Iter {iteration:3d}: error = {error_norm:.6f} m  "
                      f"thetas(deg) = {np.degrees(thetas).round(1).tolist()}")
            
            # Check convergence
            if error_norm < tolerance:
                if verbose:
                    print(f"\n✓ Converged in {iteration} iterations!")
                    print(f"Final error: {error_norm:.6f} m")
                    print(f"Final config (deg): {np.degrees(thetas).round(2).tolist()}")
                
                info = {
                    'iterations': iteration,
                    'final_error': error_norm,
                    'error_history': error_history
                }
                return True, thetas.tolist(), info
            
            # Compute Jacobian matrix (3 x n_joints)
            J = self._compute_jacobian_numerical(thetas)
            
            # Damped Least Squares (Levenberg-Marquardt)
            # Solves: (J*J^T + λ*I) * delta_x = error
            # Then: delta_theta = J^T * delta_x
            
            JJT = J @ J.T + lambda_damping * np.eye(3)
            
            try:
                # Solve for delta_x
                delta_x = np.linalg.solve(JJT, error_vector)
                
                # Compute joint angle update
                delta_theta = J.T @ delta_x
                
                # Update joint angles with step size
                thetas = thetas + step_size * delta_theta
                
                # Optional: Clip to joint limits if you have them
                # thetas = np.clip(thetas, joint_min, joint_max)
                
            except np.linalg.LinAlgError:
                if verbose:
                    print(f"Warning: Singular matrix at iteration {iteration}")
                break
        
        # Did not converge within max iterations
        if verbose:
            print(f"\n✗ Did not converge in {max_iterations} iterations")
            print(f"Final error: {error_norm:.6f} m")
            print(f"Best config (deg): {np.degrees(thetas).round(2).tolist()}")
        
        info = {
            'iterations': max_iterations,
            'final_error': error_norm,
            'error_history': error_history
        }
        return False, thetas.tolist(), info
    
    def _compute_jacobian_numerical(self, thetas: np.ndarray, epsilon: float = 1e-6) -> np.ndarray:
        """
        Compute 3xN Jacobian matrix using numerical differentiation.
        
        Jacobian J relates joint velocities to end-effector linear velocity:
            dx/dt = J * dθ/dt
        
        Where:
            - dx/dt is the end-effector velocity (3x1)
            - dθ/dt is the joint velocities (Nx1)
            - J is the Jacobian (3xN)
        
        Args:
            thetas: Current joint angles (N,)
            epsilon: Small perturbation for numerical derivative
            
        Returns:
            J: Jacobian matrix (3 x n_joints)
        """
        # Get current end-effector position
        T_dh, T_actual = forward_kinematics(
            self.model.dh_rows,
            thetas.tolist(),
            self.model.offsets
        )
        pos_current = T_actual[-1][0:3, 3]
        
        # Initialize Jacobian
        J = np.zeros((3, self.n_joints))
        
        # Compute each column of Jacobian by perturbing one joint
        for i in range(self.n_joints):
            # Perturb joint i
            thetas_perturbed = thetas.copy()
            thetas_perturbed[i] += epsilon
            
            # Compute forward kinematics with perturbed angle
            T_dh_p, T_actual_p = forward_kinematics(
                self.model.dh_rows,
                thetas_perturbed.tolist(),
                self.model.offsets
            )
            pos_perturbed = T_actual_p[-1][0:3, 3]
            
            # Numerical derivative: (f(x+h) - f(x)) / h
            J[:, i] = (pos_perturbed - pos_current) / epsilon
        
        return J
    
    def check_reachability(self, target_pos: np.ndarray) -> Tuple[bool, float]:
        """
        Check if target position is within robot's workspace.
        
        Args:
            target_pos: Target position [x, y, z]
            
        Returns:
            reachable: True if likely reachable
            distance_from_base: Distance from base to target
        """
        # Compute distance from base (origin) to target
        distance = np.linalg.norm(target_pos)
        
        # Estimate maximum reach (sum of all link lengths)
        max_reach = 0.0
        for dh_row in self.model.dh_rows:
            max_reach += abs(dh_row['a']) + abs(dh_row['d'])
        
        # Check if target is within sphere of max reach
        reachable = distance <= max_reach * 1.1  # 10% margin
        
        return reachable, distance


def solve_ik(robot_model,
             target_pos: np.ndarray,
             verbose: bool = False) -> Tuple[bool, List[float]]:
    """
    Convenience function to solve IK.
    
    Args:
        robot_model: RobotModel instance
        target_pos: Target position [x, y, z]
        verbose: Print solving progress
        
    Returns:
        success: True if solution found
        joint_angles: List of joint angles in radians
    """
    solver = UniversalIKSolver(robot_model)
    success, thetas, info = solver.solve(target_pos, verbose=verbose)
    return success, thetas


# Example usage
if __name__ == "__main__":
    print("Universal IK Solver - Example Usage\n")
    print("This solver works for ANY robot arm configuration!")
    print("\nTo use in your code:")
    print("-" * 50)
    print("from core.ik_solver import solve_ik")
    print("")
    print("target = np.array([1.0, 0.5, 0.3])")
    print("success, joint_angles = solve_ik(model, target, verbose=True)")
    print("")
    print("if success:")
    print("    model.set_all_thetas(joint_angles)")
    print("-" * 50)