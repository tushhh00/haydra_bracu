#!/usr/bin/env python3
"""
Numerical IK Solver for 4-DOF Robot

Robot arm points in +Y direction when j1=0.
j1 rotates the arm around Z axis.
j2, j3 control the planar arm in the Y-Z plane.
"""

import numpy as np
import math
from typing import Optional, List, Tuple
from scipy.optimize import minimize


def rotation_z(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def rotation_x(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

def translation(x, y, z):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


class Robot4DOF_IK:
    """Numerical IK solver based on actual URDF"""
    
    def __init__(self):
        self.d0 = 0.3498
        self.d1 = 0.0962
        self.a1 = 0.0065
        self.d2 = 0.187
        self.a2 = -0.0123
        self.d3 = 0.1205
        self.a3 = 0.006
        self.d_ee = 0.225
        
        self.limits = np.array([
            [-1.745329, 1.745329],  # joint_1 (~±100 deg)
            [-0.872665, 1.745329],  # joint_2 (-50 to +100 deg)
            [-0.872665, 1.745329],  # joint_3 (-50 to +100 deg)
            [-1.570796, 1.570796],  # joint_4 (~±90 deg)
        ])
    
    def forward_kinematics(self, joints: List[float]) -> np.ndarray:
        j1, j2, j3, j4 = joints
        T = np.eye(4)
        T = T @ translation(0, 0, self.d0)
        T = T @ rotation_z(j1)
        T = T @ translation(self.a1, 0, self.d1)
        T = T @ rotation_x(-j2)
        T = T @ translation(self.a2, 0, self.d2)
        T = T @ rotation_x(-j3)
        T = T @ translation(self.a3, 0, self.d3)
        T = T @ rotation_z(j4)
        T = T @ translation(0, 0, self.d_ee)
        return T
    
    def fk_position(self, joints: List[float]) -> Tuple[float, float, float]:
        T = self.forward_kinematics(joints)
        return T[0, 3], T[1, 3], T[2, 3]
    
    def _cost_function(self, joints, target):
        pos = self.fk_position(joints)
        return (pos[0]-target[0])**2 + (pos[1]-target[1])**2 + (pos[2]-target[2])**2
    
    def inverse_kinematics(self, x: float, y: float, z: float) -> Optional[List[float]]:
        """Solve IK using numerical optimization"""
        target = np.array([x, y, z])
        
        # Base rotation: arm faces +Y when j1=0
        # To face target (x,y), we need j1 = atan2(x, y) - pi/2
        # But because arm is at +Y, j1 = atan2(-x, y) works
        j1_init = math.atan2(-x, y) if abs(x) > 0.001 or abs(y) > 0.001 else 0
        
        # Multiple initial guesses
        initial_guesses = [
            [j1_init, 0.3, 0.3, 0],
            [j1_init, 0.5, 0.5, 0],
            [j1_init, 0.8, 0.5, 0],
            [j1_init, 1.0, 0.3, 0],
            [j1_init, 0.2, 0.8, 0],
            [j1_init, 1.2, 0.8, 0],
            [j1_init + 0.2, 0.5, 0.5, 0],
            [j1_init - 0.2, 0.5, 0.5, 0],
        ]
        
        best_solution = None
        best_error = float('inf')
        
        for guess in initial_guesses:
            try:
                result = minimize(
                    self._cost_function,
                    guess,
                    args=(target,),
                    method='SLSQP',
                    bounds=self.limits,
                    options={'ftol': 1e-12, 'maxiter': 1000}
                )
                
                error = math.sqrt(result.fun)
                if error < best_error:
                    best_error = error
                    best_solution = result.x.tolist()
            except:
                continue
        
        if best_solution and best_error < 0.01:  # 1cm tolerance
            return best_solution
        return None
    
    def get_solutions(self, x: float, y: float, z: float) -> List[List[float]]:
        sol = self.inverse_kinematics(x, y, z)
        return [sol] if sol else []
    
    def print_workspace(self):
        print("Workspace Analysis:")
        z_min, z_max = float('inf'), float('-inf')
        r_max = 0
        
        for j2 in np.linspace(self.limits[1][0], self.limits[1][1], 20):
            for j3 in np.linspace(self.limits[2][0], self.limits[2][1], 20):
                pos = self.fk_position([0, j2, j3, 0])
                z_min = min(z_min, pos[2])
                z_max = max(z_max, pos[2])
                r = math.sqrt(pos[0]**2 + pos[1]**2)
                r_max = max(r_max, r)
        
        print(f"  Z range: {z_min:.3f} to {z_max:.3f} m")
        print(f"  Max reach: ~{r_max:.3f} m")
        print(f"  Arm faces +Y direction when j1=0")


def main():
    import sys
    
    ik = Robot4DOF_IK()
    
    print("=" * 50)
    print("4-DOF Robot IK Solver")
    print("=" * 50)
    
    ik.print_workspace()
    
    # FK test
    print("\n--- FK Test ---")
    tests = [
        [0, 0, 0, 0],
        [0, 0.5, 0.5, 0],
        [0.5, 0.5, 0.3, 0],
        [-0.5, 0.8, 0.5, 0],
    ]
    for j in tests:
        p = ik.fk_position(j)
        print(f"  {j} -> ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
    
    # IK test using FK positions
    print("\n--- IK Test (verify with FK) ---")
    for joints in tests:
        pos = ik.fk_position(joints)
        sol = ik.get_solutions(pos[0], pos[1], pos[2])
        if sol:
            fk = ik.fk_position(sol[0])
            err = math.sqrt((fk[0]-pos[0])**2 + (fk[1]-pos[1])**2 + (fk[2]-pos[2])**2)
            status = f"✓ err={err*1000:.1f}mm"
        else:
            status = "✗ NO SOLUTION"
        print(f"  ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}): {status}")
    
    # Test typical pick positions
    print("\n--- Test Pick Positions ---")
    pick_tests = [
        (0.0, 0.3, 0.5),   # In front (+Y)
        (0.0, 0.4, 0.6),   # Higher
        (0.2, 0.2, 0.5),   # Side
        (-0.2, 0.3, 0.5),  # Other side
        (0.0, 0.35, 0.4),  # Lower
    ]
    
    for x, y, z in pick_tests:
        sol = ik.get_solutions(x, y, z)
        if sol:
            fk = ik.fk_position(sol[0])
            err = math.sqrt((fk[0]-x)**2 + (fk[1]-y)**2 + (fk[2]-z)**2)
            j = sol[0]
            print(f"  ({x:.2f}, {y:.2f}, {z:.2f}): ✓ [{j[0]:.2f}, {j[1]:.2f}, {j[2]:.2f}, {j[3]:.2f}]")
        else:
            print(f"  ({x:.2f}, {y:.2f}, {z:.2f}): ✗ UNREACHABLE")
    
    # Interactive
    print("\n" + "=" * 50)
    print("Enter 'x y z' or 'q' to quit")
    print("=" * 50)
    
    while True:
        try:
            inp = input("\nPos: ").strip()
            if inp.lower() == 'q':
                break
            parts = inp.split()
            if len(parts) != 3:
                continue
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            
            sol = ik.get_solutions(x, y, z)
            if sol:
                j = sol[0]
                print(f"  Joints: [{j[0]:.4f}, {j[1]:.4f}, {j[2]:.4f}, {j[3]:.4f}]")
                print(f"  Degrees: [{math.degrees(j[0]):.1f}, {math.degrees(j[1]):.1f}, {math.degrees(j[2]):.1f}, {math.degrees(j[3]):.1f}]")
                fk = ik.fk_position(j)
                print(f"  FK: ({fk[0]:.3f}, {fk[1]:.3f}, {fk[2]:.3f})")
            else:
                print("  UNREACHABLE")
        except:
            pass
    
    print("Done!")


if __name__ == '__main__':
    main()
