#!/usr/bin/env python3
"""
Delta Robot Inverse Kinematics Module

Pure Python implementation matching visual_kinematics.RobotDelta output.
No external dependencies beyond math.

Usage:
    from delta_ik import DeltaIK
    ik = DeltaIK()  # Uses default geometry
    angles = ik.inverse(x, y, z)  # Returns (theta1, theta2, theta3) in radians
"""
import math
from typing import Tuple, Optional
import numpy as np

class DeltaIK:
    """
    Delta Robot Inverse Kinematics Solver.
    
    Geometry params match: RobotDelta(np.array([0.104, 0.040, 0.105, 0.205]))
    """
    
    def __init__(self, r_base: float = 0.104, r_ee: float = 0.040, 
                 l_upper: float = 0.105, l_lower: float = 0.205):
        """
        Args:
            r_base: Base platform radius (meters)
            r_ee: End-effector platform radius (meters)  
            l_upper: Upper arm length (meters)
            l_lower: Lower arm / parallelogram length (meters)
        """
        self.l1 = l_upper
        self.l2 = l_lower
        self.r1 = r_base
        
        # Attachment points (matching visual_kinematics internals)
        # These are end-effector attachment offsets relative to center
        self.ap_x = [-r_ee, r_ee / 2, r_ee / 2]
        self.ap_y = [0.0, -r_ee * math.sqrt(3) / 2, r_ee * math.sqrt(3) / 2]
        
        # Motor arm angles (120° spacing)
        self.phi = [0.0, 2.0943951023931953, 4.1887902047863905]  # 0, 2π/3, 4π/3
        self.cos_phi = [math.cos(p) for p in self.phi]
        self.sin_phi = [math.sin(p) for p in self.phi]
    
    def _simplify_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        while angle <= -math.pi:
            angle += 2 * math.pi
        while angle > math.pi:
            angle -= 2 * math.pi
        return angle
    
    def inverse(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """
        Compute inverse kinematics for Cartesian position.
        
        Args:
            x, y, z: Target position in meters
            
        Returns:
            (theta1, theta2, theta3): Joint angles in RADIANS
            
        Raises:
            ValueError: If position is outside workspace
        """
        theta = []
        
        for i in range(3):
            # Vector from attachment point to target
            oa_x = x - self.ap_x[i]
            oa_y = y - self.ap_y[i]
            oa_z = z
            
            norm_oa_sq = oa_x**2 + oa_y**2 + oa_z**2
            
            # Coefficients for: a*sin(θ) + b*cos(θ) = c
            a = 2.0 * self.l1 * z
            
            cp = self.cos_phi[i]
            sp = self.sin_phi[i]
            
            term1 = (self.r1 * cp) - oa_x
            term2 = (self.r1 * sp) - oa_y
            b = 2.0 * self.l1 * (cp * term1 + sp * term2)
            
            c = (self.l2**2 - self.l1**2 - norm_oa_sq - self.r1**2 + 
                 2.0 * self.r1 * (cp * oa_x + sp * oa_y))
            
            # Solve using: θ = atan2(c, -√(a²+b²-c²)) - atan2(b, a)
            disc = a*a + b*b - c*c
            if disc < 0:
                raise ValueError(f"Position ({x}, {y}, {z}) outside workspace")
            
            angle = math.atan2(c, -math.sqrt(disc)) - math.atan2(b, a)
            theta.append(self._simplify_angle(angle))
        
        return (theta[0], theta[1], theta[2])
    
    def inverse_deg(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """Same as inverse() but returns angles in DEGREES."""
        rads = self.inverse(x, y, z)
        return (math.degrees(rads[0]), math.degrees(rads[1]), math.degrees(rads[2]))
    
    def check_position(self, x: float, y: float, z: float) -> dict:
        """
        Check if position is valid and how close to workspace limits.
        
        Returns:
            dict with keys:
                'valid': bool - True if position is reachable
                'warning': str or None - Warning message if close to limits
                'margin': float - Smallest discriminant margin (lower = closer to limit)
        """
        min_disc = float('inf')
        
        for i in range(3):
            # Calculate discriminant (same as in inverse())
            oa_x = x - self.ap_x[i]
            oa_y = y - self.ap_y[i]
            oa_z = z
            
            norm_oa_sq = oa_x**2 + oa_y**2 + oa_z**2
            
            a = 2.0 * self.l1 * z
            cp = self.cos_phi[i]
            sp = self.sin_phi[i]
            
            term1 = (self.r1 * cp) - oa_x
            term2 = (self.r1 * sp) - oa_y
            b = 2.0 * self.l1 * (cp * term1 + sp * term2)
            
            c = (self.l2**2 - self.l1**2 - norm_oa_sq - self.r1**2 + 
                 2.0 * self.r1 * (cp * oa_x + sp * oa_y))
            
            disc = a*a + b*b - c*c
            min_disc = min(min_disc, disc)
        
        # Determine status
        if min_disc < 0:
            return {
                'valid': False,
                'warning': f'Position ({x:.3f}, {y:.3f}, {z:.3f}) is OUTSIDE workspace',
                'margin': min_disc
            }
        elif min_disc < 0.001:  # Very close to limit
            return {
                'valid': True,
                'warning': f'WARNING: Position near workspace boundary (margin={min_disc:.4f})',
                'margin': min_disc
            }
        elif min_disc < 0.01:  # Somewhat close
            return {
                'valid': True,
                'warning': f'Caution: Approaching workspace limit (margin={min_disc:.4f})',
                'margin': min_disc
            }
        else:
            return {
                'valid': True,
                'warning': None,
                'margin': min_disc
            }
    
    def is_reachable(self, x: float, y: float, z: float) -> bool:
        """Quick check if position is reachable."""
        return self.check_position(x, y, z)['valid']


# Convenience function for drop-in replacement
_default_ik = None

def get_joint_angles(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """
    Quick function to get joint angles (radians) for a position.
    Uses default robot geometry.
    """
    global _default_ik
    if _default_ik is None:
        _default_ik = DeltaIK()
    return _default_ik.inverse(x, y, z)


if __name__ == "__main__":
    # Quick test
    ik = DeltaIK()
    test_points = [
        (0.0, 0.0, -0.22),
        (0.05, 0.05, -0.25),
        (-0.05, 0.02, -0.20),
    ]
    
    print("Delta IK Test (comparing with your verify_ik_math.py values)")
    print("-" * 50)
    for pt in test_points:
        try:
            angles = ik.inverse_deg(*pt)
            print(f"Position {pt} -> Angles: {angles[0]:.2f}°, {angles[1]:.2f}°, {angles[2]:.2f}°")
        except ValueError as e:
            print(f"Position {pt} -> {e}")
