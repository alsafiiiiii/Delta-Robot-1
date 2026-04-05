#!/usr/bin/env python3
"""
Delta Robot Inverse & Forward Kinematics (Radius/Vector Method)

Adapted to use specific Center-to-Joint measurements (Radius) 
while maintaining Forward Kinematics capabilities.
"""
import math
from typing import Tuple

class DeltaIK:
    """
    Delta Robot Kinematics Solver (Radius-Based).
    
    Geometry parameters (in meters):
        r_base: Distance from absolute center to motor shaft (Horizontal)
        r_ee:   Distance from absolute center to end-effector joint (Horizontal)
        l_upper: Upper arm (bicep) length (Center-to-Center)
        l_lower: Lower arm (forearm) length (Center-to-Center)
    """
    
    def __init__(self, r_base: float = 0.100, r_ee: float = 0.040, 
                 l_upper: float = 0.100, l_lower: float = 0.200):
        """
        Args:
            r_base: Base radius (center to motor axis)
            r_ee: End-effector radius (center to ball joint)
            l_upper: Upper arm length
            l_lower: Lower arm length
        """
        self.r_base = r_base
        self.r_ee = r_ee
        self.rf = l_upper
        self.re = l_lower
        
        # Pre-calculated constants
        self.t = self.r_base - self.r_ee  # Horizontal distance difference
        self.sqrt3 = math.sqrt(3.0)
        self.sin120 = self.sqrt3 / 2.0
        self.cos120 = -0.5
        self.tan60 = self.sqrt3
        self.sin30 = 0.5
        self.tan30 = 1.0 / self.sqrt3
    
    def _calc_angle_yz(self, x0: float, y0: float, z0: float) -> Tuple[bool, float]:
        """
        Helper: Calculate angle theta for a leg projected onto the YZ plane.
        Uses direct Radius logic: Motor is at Y = -r_base
        """
        # Motor position (y1) and Target relative to EE (y0)
        y1 = -self.r_base
        y0 = y0 - self.r_ee
        
        # Distance squared from origin to target point
        # (This replaces the complex apothem math)
        dist_sq = x0*x0 + y0*y0 + z0*z0
        
        # Intersection of circle (rf) and sphere (re)
        # z = a + b*y
        a = (dist_sq + self.rf*self.rf - self.re*self.re - y1*y1) / (2.0 * z0)
        b = (y1 - y0) / z0
        
        # Discriminant
        d = -(a + b*y1)*(a + b*y1) + self.rf*(b*b*self.rf + self.rf)
        
        if d < 0:
            return (False, 0.0)  # Unreachable
        
        # Solve for yj (joint y position)
        # We choose the outer point (standard delta config)
        yj = (y1 - a*b - math.sqrt(d)) / (b*b + 1.0)
        zj = a + b*yj
        
        # Calculate theta relative to motor shaft
        theta = math.atan2(-zj, y1 - yj) * 180.0 / math.pi
        
        return (True, theta)
    
    def inverse_deg(self, x_in: float, y_in: float, z_in: float) -> Tuple[float, float, float]:
        """
        Inverse kinematics: (x, y, z) -> (theta1, theta2, theta3) in DEGREES.
        INCLUDES ROTATION: Inputs are rotated so +X aligns with Motor 1.
        """
        # 1. Coordinate Rotation: User(+X) -> Solver(-Y)
        # We map User X to Solver Y, and User Y to Solver -X
        # (Rotation of -90 degrees)
        x0 = y_in
        y0 = -x_in
        z0 = z_in
        success1, theta1 = self._calc_angle_yz(x0, y0, z0)
        if not success1:
            raise ValueError(f"Position ({x0:.3f}, {y0:.3f}, {z0:.3f}) unreachable (Leg 1)")
        
        # Leg 2: Rotate +120
        x2 = x0 * self.cos120 + y0 * self.sin120
        y2 = y0 * self.cos120 - x0 * self.sin120
        success2, theta2 = self._calc_angle_yz(x2, y2, z0)
        if not success2:
            raise ValueError(f"Position ({x0:.3f}, {y0:.3f}, {z0:.3f}) unreachable (Leg 2)")
        
        # Leg 3: Rotate -120  
        x3 = x0 * self.cos120 - y0 * self.sin120
        y3 = y0 * self.cos120 + x0 * self.sin120
        success3, theta3 = self._calc_angle_yz(x3, y3, z0)
        if not success3:
            raise ValueError(f"Position ({x0:.3f}, {y0:.3f}, {z0:.3f}) unreachable (Leg 3)")
        
        return (theta1, theta2, theta3)
    
    def inverse(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """Returns angles in RADIANS"""
        deg = self.inverse_deg(x, y, z)
        return (math.radians(deg[0]), math.radians(deg[1]), math.radians(deg[2]))
    
    def forward(self, theta1: float, theta2: float, theta3: float) -> Tuple[float, float, float]:
        """
        Forward kinematics: (theta1, theta2, theta3) -> (x, y, z)
        Args: Angles in DEGREES
        """
        # t is the horizontal radius difference
        t = self.t 
        
        # Convert to radians
        t1 = math.radians(theta1)
        t2 = math.radians(theta2)
        t3 = math.radians(theta3)
        
        # Calculate elbow positions (spheres 1, 2, 3)
        # y1, z1 are in the local frame of Leg 1
        y1 = -(t + self.rf * math.cos(t1))
        z1 = -self.rf * math.sin(t1)
        
        # y2, x2, z2 for Leg 2 (rotated 120)
        y2 = (t + self.rf * math.cos(t2)) * self.sin30
        x2 = y2 * self.tan60
        z2 = -self.rf * math.sin(t2)
        
        # y3, x3, z3 for Leg 3 (rotated 240)
        y3 = (t + self.rf * math.cos(t3)) * self.sin30
        x3 = -y3 * self.tan60
        z3 = -self.rf * math.sin(t3)
        
        dnm = (y2 - y1) * x3 - (y3 - y1) * x2
        
        w1 = y1*y1 + z1*z1
        w2 = x2*x2 + y2*y2 + z2*z2
        w3 = x3*x3 + y3*y3 + z3*z3
        
        # Intersection of 3 spheres (trilateration)
        # x = (a1*z + b1) / dnm
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0
        
        # y = (a2*z + b2) / dnm
        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0
        
        # a*z^2 + b*z + c = 0
        a = a1*a1 + a2*a2 + dnm*dnm
        b = 2.0 * (a1*b1 + a2*(b2 - y1*dnm) - z1*dnm*dnm)
        c = (b2 - y1*dnm)*(b2 - y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - self.re*self.re)
        
        # Discriminant
        d = b*b - 4.0*a*c
        if d < 0:
            raise ValueError("Angles result in unreachable position (Non-intersecting spheres)")
        
        # Choose the solution (usually negative Z for delta robots)
        z0 = -0.5 * (b + math.sqrt(d)) / a
        x0 = (a1*z0 + b1) / dnm
        y0 = (a2*z0 + b2) / dnm
        
        # Rotate back to User Frame: Solver(-Y) -> User(+X)
        # x_user = -y_solver, y_user = x_solver
        return (-y0, x0, z0)

    # ------------------------------------------------------------------
    # UTILITY FUNCTIONS
    # ------------------------------------------------------------------
    
    def check_position(self, x: float, y: float, z: float) -> dict:
        try:
            self.inverse_deg(x, y, z)
            return {'valid': True, 'warning': None}
        except ValueError as e:
            return {'valid': False, 'warning': str(e)}

    def is_reachable(self, x: float, y: float, z: float) -> bool:
        return self.check_position(x, y, z)['valid']


# Convenience functions
_default_ik = None

def get_joint_angles(x: float, y: float, z: float) -> Tuple[float, float, float]:
    global _default_ik
    if _default_ik is None:
        _default_ik = DeltaIK()
    return _default_ik.inverse(x, y, z)


if __name__ == "__main__":
    # Test with your setup
    ik = DeltaIK()
    
    # Test points (X, Y, Z)
    points = [
        (0.0, 0.0, -0.25),
        (0.05, 0.05, -0.20),
    ]
    
    print("Corrected Delta IK Test (Radius-Based)")
    print("=" * 60)
    for p in points:
        try:
            # 1. Inverse Kinematics
            angs = ik.inverse_deg(*p)
            print(f"Pos: {p}")
            print(f" -> Angles: {angs[0]:.2f}, {angs[1]:.2f}, {angs[2]:.2f}")
            
            # 2. Forward Kinematics Verification
            calc_pos = ik.forward(*angs)
            err = math.sqrt(sum((a-b)**2 for a,b in zip(p, calc_pos)))
            print(f" <- FK Calc: ({calc_pos[0]:.4f}, {calc_pos[1]:.4f}, {calc_pos[2]:.4f})")
            print(f"    Error:   {err*1000:.4f} mm")
            print("-" * 60)
        except ValueError as e:
            print(f"Error for {p}: {e}")