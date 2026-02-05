#!/usr/bin/env python3
"""
Test script to verify delta_ik.py matches visual_kinematics output.
Shows full precision to verify accuracy.
"""
import math

# Test our new module
from delta_ik import DeltaIK

# Try to import visual_kinematics for comparison
try:
    import numpy as np
    from visual_kinematics.RobotDelta import RobotDelta
    from visual_kinematics.Frame import Frame
    HAS_VK = True
except ImportError:
    HAS_VK = False
    print("visual_kinematics not installed, skipping comparison")

def main():
    test_points = [
        (0.0, 0.0, -0.22),
        (0.05, 0.05, -0.25),
        (-0.05, 0.02, -0.20),
        (0.0, 0.08, -0.18),
    ]
    
    # Our new IK
    ik = DeltaIK()
    
    # Visual kinematics (if available)
    if HAS_VK:
        vk_robot = RobotDelta(np.array([0.104, 0.040, 0.105, 0.205]))
    
    print("=" * 70)
    print("Delta IK Verification Test")
    print("=" * 70)
    
    for pt in test_points:
        print(f"\nPosition: {pt}")
        
        # Our IK (radians for precision comparison)
        try:
            angles_rad = ik.inverse(*pt)
            angles_deg = ik.inverse_deg(*pt)
            print(f"  delta_ik:          {angles_deg[0]:7.2f}°, {angles_deg[1]:7.2f}°, {angles_deg[2]:7.2f}°")
        except ValueError as e:
            print(f"  delta_ik:          Error - {e}")
            continue
        
        # Visual kinematics
        if HAS_VK:
            f = Frame.from_euler_3(np.array([0.,0.,0.]), np.array([[pt[0]],[pt[1]],[pt[2]]]))
            vk_angles_rad = vk_robot.inverse(f).flatten()
            vk_angles_deg = np.rad2deg(vk_angles_rad)
            print(f"  visual_kinematics: {vk_angles_deg[0]:7.2f}°, {vk_angles_deg[1]:7.2f}°, {vk_angles_deg[2]:7.2f}°")
            
            # Difference
            diff = [abs(angles_deg[i] - vk_angles_deg[i]) for i in range(3)]
            max_diff = max(diff)
            status = "✓ MATCH" if max_diff < 0.01 else f"⚠ DIFF: {max_diff:.4f}°"
            print(f"  Status: {status}")
    
    # HIGH PRECISION TEST
    if HAS_VK:
        print("\n" + "=" * 70)
        print("HIGH PRECISION COMPARISON (Full 15 decimal places)")
        print("=" * 70)
        
        pt = (0.05, 0.05, -0.25)
        angles_rad = ik.inverse(*pt)
        f = Frame.from_euler_3(np.array([0.,0.,0.]), np.array([[pt[0]],[pt[1]],[pt[2]]]))
        vk_angles_rad = vk_robot.inverse(f).flatten()
        
        print(f"\nTest Point: {pt}")
        for i in range(3):
            diff = abs(angles_rad[i] - vk_angles_rad[i])
            print(f"  Theta{i+1}:")
            print(f"    delta_ik:          {angles_rad[i]:.15f} rad")
            print(f"    visual_kinematics: {vk_angles_rad[i]:.15f} rad")
            print(f"    Difference:        {diff:.2e} rad ({math.degrees(diff):.2e} deg)")
    
    print("\n" + "=" * 70)
    print("CONCLUSION:")
    print("  - Python float = 64-bit double = ~15 significant digits")
    print("  - Your servo resolution = ~0.1° (PWM limited)")
    print("  - More decimal places won't help - hardware is the limit!")
    print("=" * 70)

if __name__ == "__main__":
    main()

