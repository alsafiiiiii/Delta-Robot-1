#!/usr/bin/env python3
import math
import numpy as np
from delta_dynamics import DeltaDynamics

def test_static_hover():
    print("\n--- Test 1: Static Hover (Gravity Compensation) ---")
    
    # Initialize with user parameters
    bot = DeltaDynamics(
        mass_upper_arm=0.4,
        mass_forearm=0.04,
        mass_effector=0.4
    )
    
    # Scenario: Holding steady at geometric center, slightly down
    # With q=[0,0,0] (arms horizontal), Z should be roughly -sqrt(L2^2 - (ra-rb-L1)^2)
    # L2=0.205, L1=0.105, ra=0.104, rb=0.04. rdif=0.064.
    # Horizontal proj of rod = L2? No.
    # If q=0, Elbow is at radius ra - L1 (inwards?) No, L1 projects out.
    # Elbow R_e = ra + L1. 
    # Effector R_p = rb.
    # Horizontal dist for rod = (ra + L1) - rb = 0.104 + 0.105 - 0.04 = 0.169
    # Wait, if H_dist = 0.169, and Rod = 0.205...
    # Vertical dist Z = -sqrt(0.205^2 - 0.169^2) = -sqrt(0.0420 - 0.0285) = -0.116m
    
    z_hover = -0.116
    pos = [0, 0, z_hover]
    acc = [0, 0, 0]
    q = [0, 0, 0] # Radians
    q_acc = [0, 0, 0]
    
    torques = bot.compute_torques(pos, acc, q, q_acc)
    print(f"Position: {pos}")
    print(f"Torques: {torques} Nm")
    
    # Check Symmetry
    if np.allclose(torques[0], torques[1]) and np.allclose(torques[1], torques[2]):
        print("PASS: Torques are symmetric.")
    else:
        print("FAIL: Asymmetry detected at center!")

def test_dynamic_movement():
    print("\n--- Test 2: Dynamic Movement (Vertical Acceleration) ---")
    bot = DeltaDynamics()
    
    # Accelerating UPWARDS (+Z) at 1G (9.81 m/s^2)
    # Should require MORE torque to overcome gravity + inertia
    pos = [0, 0, -0.2]
    # val_q = math.asin((0.2) / 0.105) # REMOVED due to domain error
    q = [0.0, 0.0, 0.0] # Arms horizontal (approx Z=-0.116m as calcd above)
    pos = [0, 0, -0.116] # Consistent with q=0
    
    acc_static = [0, 0, 0]
    acc_up = [0, 0, 9.81] # 1G up
    
    # Mock q_acc for upward move (arms must push down/up? depends on config)
    q_acc = [0, 0, 0] 
    
    tau_static = bot.compute_torques(pos, acc_static, q, q_acc)
    tau_dynamic = bot.compute_torques(pos, acc_up, q, q_acc)
    
    print(f"Static Torque: {tau_static[0]:.4f} Nm")
    print(f"Dynamic Torque (1G up): {tau_dynamic[0]:.4f} Nm")
    
    diff = tau_dynamic[0] - tau_static[0]
    print(f"Difference: {diff:.4f} Nm")
    
    # If accelerating up (against gravity), we expect different torque.
    # Logic check: To accelerate up, we need net force UP.
    # Gravity pulls down.
    # So Force_needed = m(g + a).
    # Torque should Magnitude Increase? or Decrease?
    # Depends on sign of torque. 
    # If Static is -0.5, Dynamic should be like -1.0 (more effort).
    
if __name__ == "__main__":
    test_static_hover()
    test_dynamic_movement()
