#!/usr/bin/env python3
import numpy as np
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame
import math

# --- C IMPLEMENTATION PORT ---
# This mimics the C code exactly to test the logic
R_BASE = 0.104
R_END = 0.040
L_UPPER = 0.105
L_LOWER = 0.205

def c_algo_calc_angle_yz(x0, y0, z0):
    y1 = -R_BASE
    y0 -= R_END
    
    a = (x0*x0 + y0*y0 + z0*z0 + L_UPPER*L_UPPER - L_LOWER*L_LOWER - y1*y1) / (2.0 * z0)
    b = (y1 - y0) / z0
    
    d = -(a + b*y1)*(a + b*y1) + L_UPPER*(b*b*L_UPPER + L_UPPER)
    if d < 0:
        return None
        
    yj = (y1 - a*b - math.sqrt(d)) / (b*b + 1)
    zj = a + b*yj
    
    return 180.0 * math.atan2(-zj, (y1-yj)) / math.pi

def c_algo_solve_ik(x, y, z):
    # This mimics the Geometric Method used in the C code
    # Params
    Sb = R_BASE * math.sqrt(3.0)
    Sp = R_END * math.sqrt(3.0)
    L = L_UPPER
    l = L_LOWER
    
    Wb = (math.sqrt(3.0)/6.0) * Sb
    Ub = (math.sqrt(3.0)/3.0) * Sb
    Wp = (math.sqrt(3.0)/6.0) * Sp
    Up = (math.sqrt(3.0)/3.0) * Sp
    
    A = Wb - Up
    B = (Sp*0.5) - ((math.sqrt(3.0)*0.5) * Wb)
    C = Wp - (0.5 * Wb)
    
    # Pivot 1
    E1 = 2.0 * L * (y + A)
    F1 = 2.0 * z * L
    G1 = x**2 + y**2 + z**2 + A**2 + L**2 + 2.0*y*A - l**2
    
    # Pivot 2
    E2 = -L * ((math.sqrt(3.0)*(x + B)) + y + C)
    F2 = 2.0 * z * L
    G2 = x**2 + y**2 + z**2 + B**2 + C**2 + L**2 + 2.0*((x*B) + (y*C)) - l**2
    
    # Pivot 3
    E3 = L * ((math.sqrt(3.0)*(x - B)) - y - C)
    F3 = 2.0 * z * L
    G3 = x**2 + y**2 + z**2 + B**2 + C**2 + L**2 + 2.0*(-(x*B) + (y*C)) - l**2
    
    def solve_t(E, F, G):
        disc = E*E + F*F - G*G
        if disc < 0: return None
        sq = math.sqrt(disc)
        t1 = (-F - sq) / (G - E)
        return 2.0 * math.atan(t1) * 180.0 / math.pi
    
    t1 = solve_t(E1, F1, G1)
    t2 = solve_t(E2, F2, G2)
    t3 = solve_t(E3, F3, G3)
    
    return t1, t2, t3

# --- COMPARISON ---

def main():
    print("--- Verifying On-Chip IK Math ---")
    
    # 1. Setup Visual Kinematics (Ground Truth)
    robot = RobotDelta(np.array([R_BASE, R_END, L_UPPER, L_LOWER]))
    
    test_points = [
        (0.0, 0.0, -0.22), # Home
        (0.05, 0.05, -0.25),
        (-0.05, 0.02, -0.20),
        (0.0, 0.08, -0.18)
    ]
    
    for pt in test_points:
        print(f"\nTesting Point: {pt}")
        
        # Ground Truth
        f = Frame.from_euler_3(np.array([0.,0.,0.]), np.array([[pt[0]],[pt[1]],[pt[2]]]))
        vk_angles = np.rad2deg(robot.inverse(f).flatten()) # Convert to Degrees
        print(f"Visual Kinematics (Deg): {vk_angles[0]:.2f}, {vk_angles[1]:.2f}, {vk_angles[2]:.2f}")
        
        # C Algorithm (Standard)
        c_angles = c_algo_solve_ik(pt[0], pt[1], pt[2])
        print(f"Prop Algo (Standard) : {c_angles[0]:.2f}")
        
        # Permutation 1: Invert Z
        c_angles_invZ = c_algo_solve_ik(pt[0], pt[1], -pt[2])
        if c_angles_invZ[0]: print(f"Prop Algo (Inv Z)    : {c_angles_invZ[0]:.2f}")
        
        # Permutation 2: 90 - Angle
        print(f"Prop Algo (90 - Ang) : {90 - c_angles[0]:.2f}")

        # Permutation 3: Angle - 90
        print(f"Prop Algo (Ang - 90) : {c_angles[0] - 90:.2f}")
        
        # Permutation 5: Swap L/l (In case VK uses different order or I swapped them)
        # Hack global logic by swapping for one call
        
        # Define a swapped algo function
        def c_algo_swapped(x, y, z):
             # Swap L_UPPER and L_LOWER
             L = L_LOWER
             l = L_UPPER
             # Copy-Paste Logic... or just modify globals?
             # Globals are read-only-ish.
             # Let's duplicate logic or pass params.
             # For speed, I'll just change the math below:
             
             R_BASE_L = 0.104
             R_END_L = 0.040
             # SWAP HERE
             L_UPPER_L = 0.205
             L_LOWER_L = 0.105
             
             # ... Re-implement core logic locally ...
             Sb = R_BASE_L * math.sqrt(3.0)
             Sp = R_END_L * math.sqrt(3.0)
             L = L_UPPER_L
             l = L_LOWER_L
             
             Wb = (math.sqrt(3.0)/6.0) * Sb
             Ub = (math.sqrt(3.0)/3.0) * Sb
             Wp = (math.sqrt(3.0)/6.0) * Sp
             Up = (math.sqrt(3.0)/3.0) * Sp
             
             A = Wb - Up
             B = (Sp*0.5) - ((math.sqrt(3.0)*0.5) * Wb)
             C = Wp - (0.5 * Wb)
             
             # Pivot 1
             E1 = 2.0 * L * (y + A)
             F1 = 2.0 * z * L
             G1 = x**2 + y**2 + z**2 + A**2 + L**2 + 2.0*y*A - l**2
             
             def solve_t_loc(E, F, G):
                 disc = E*E + F*F - G*G
                 if disc < 0: return None
                 sq = math.sqrt(disc)
                 t1 = (-F - sq) / (G - E)
                 return 2.0 * math.atan(t1) * 180.0 / math.pi
                 
             return solve_t_loc(E1, F1, G1)

        # Permutation 6: VK Exact Port
        def c_algo_vk_exact(x, y, z):
            # Constants from Dump
            l1 = 0.105
            l2 = 0.205
            r1 = 0.104
            # r2 is implicit in ap
            
            # AP Columns form Dump
            # Col 0: [-0.04, 0, 0]
            # Col 1: [0.02, -0.03464102, 0]
            # Col 2: [0.02, 0.03464102, 0]
            ap = [
                [-0.04, 0.02, 0.02],
                [0.0, -0.03464102, 0.03464102],
                [0.0, 0.0, 0.0]
            ]
            
            # Phi
            phi = [0.0, 2.0943951, 4.1887902]
            
            theta = []
            
            for i in range(3):
                # op is target (x,y,z)
                op = [x, y, z]
                
                # oa = op - ap_i
                oa_0 = op[0] - ap[0][i]
                oa_1 = op[1] - ap[1][i]
                oa_2 = op[2] - ap[2][i] # z component usually 0 for ap
                
                norm_oa_sq = oa_0**2 + oa_1**2 + oa_2**2
                
                # a = 2*l1*z (op[2])
                a = 2 * l1 * op[2]
                
                # b = 2*l1 * (cos(phi)*(r1*cos(phi) - oaX) + sin(phi)*(r1*sin(phi) - oaY))
                cp = math.cos(phi[i])
                sp = math.sin(phi[i])
                
                term1 = (r1 * cp) - oa_0
                term2 = (r1 * sp) - oa_1
                
                b = 2 * l1 * (cp * term1 + sp * term2)
                
                # c = l2^2 - l1^2 - norm_oa^2 - r1^2 + 2*r1*(cos*oaX + sin*oaY)
                c = l2**2 - l1**2 - norm_oa_sq - r1**2 + 2*r1*(cp*oa_0 + sp*oa_1)
                
                # Solve a*sin + b*cos = c
                # theta = atan2(c, -sqrt(a*a+b*b-c*c)) - atan2(b, a)
                # Note: VK uses -sqrt (elbow down/up selection)
                
                disc = a*a + b*b - c*c
                if disc < 0:
                    theta.append(None)
                else:
                    val = math.atan2(c, -math.sqrt(disc)) - math.atan2(b, a)
                    theta.append(simplify_angle(val))
                    
            return theta

        def simplify_angle(angle):
            # VK helper
            while angle <= -math.pi: angle += 2*math.pi
            while angle > math.pi: angle -= 2*math.pi
            return angle * 180.0 / math.pi # Return deg
            
        vk_exact = c_algo_vk_exact(pt[0], pt[1], pt[2])
        if vk_exact[0] is not None:
             print(f"VK Exact Port        : {vk_exact[0]:.2f}, {vk_exact[1]:.2f}, {vk_exact[2]:.2f}")

    print("\n" + "="*40)
    print(" INTERACTIVE MODE ")
    print(" Enter coordinates from ESP32 Log to verify.")
    print(" Format: x y z (e.g. '0.0 0.0 -0.22')")
    print(" Ctrl+C to exit.")
    print("="*40)
    
    while True:
        try:
            inp = input("\nEnter X Y Z: ")
            if not inp.strip(): continue
            parts = inp.replace(',', ' ').split()
            if len(parts) != 3:
                print("Error: Please enter 3 numbers.")
                continue
                
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            
            angles = c_algo_vk_exact(x, y, z)
            if angles[0] is None:
                print("Result: Unreachable (None)")
            else:
                print(f"EXPECTED ANGLES: {angles[0]:.2f}, {angles[1]:.2f}, {angles[2]:.2f}")
                
        except ValueError:
            print("Invalid number format.")
        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    main()
