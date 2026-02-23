#!/usr/bin/env python3
import csv
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

def load_data(filename):
    times = []
    inputs = []
    outputs = []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        header = next(reader) # Skip header
        for row in reader:
            if not row: continue
            try:
                t = float(row[0])
                u = float(row[1]) 
                y = float(row[2])
                times.append(t)
                inputs.append(u)
                outputs.append(y)
            except ValueError:
                continue
    return np.array(times), np.array(inputs), np.array(outputs)

def identify_segments(times, inputs):
    # Find segments where input is roughly constant and non-zero
    # This helps isolate specific "moves"
    segments = []
    
    current_start = 0
    in_segment = False
    
    # Threshold for considering input "active"
    THRESHOLD = 0.02
    
    for i in range(1, len(inputs)):
        # Check for discontinuities in time
        if times[i] - times[i-1] > 0.2:
            if in_segment:
                segments.append((current_start, i-1))
            in_segment = False
            continue
            
        is_active = abs(inputs[i]) > THRESHOLD
        
        if is_active and not in_segment:
            current_start = i
            in_segment = True
        elif not is_active and in_segment:
            segments.append((current_start, i-1))
            in_segment = False
            
    if in_segment:
        segments.append((current_start, len(inputs)-1))
        
    return segments

def integrator_model(t, K, theta, y0, t_start):
    # Model: y(t) = y0 + K * input_mag * (t - t_start - theta) * heaviside
    # Just fitting the slope part
    # Assumes input U is provided globally or passed in context
    return y0 + (t > (t_start + theta)) * K * (t - (t_start + theta))

def tune_segment(t_seg, u_seg, y_seg):
    # 1. Estimate Mean Input U
    U_mean = np.mean(u_seg)
    if abs(U_mean) < 0.01: return None
    
    # 2. Fit Line to Output to find K (Velocity Gain)
    # y = mx + c
    # slope m = K * U_mean  => K = m / U_mean
    
    # Using curve_fit for robust line fitting on the main part of the move
    # Simple Linear Regression:
    slope, intercept = np.polyfit(t_seg, y_seg, 1)
    
    K_estimated = slope / U_mean
    
    # 3. Estimate Rough Delay (Theta)
    # This is harder with noise. We'll look for simple lag or assume a safe minimum.
    # Camera ~30fps + Comms -> ~0.05s min
    theta_estimated = 0.05 
    
    return abs(K_estimated), theta_estimated, U_mean

def main():
    filename = 'pid_data.csv'
    t, u, y = load_data(filename)
    
    print(f"Loaded {len(t)} samples.")
    
    segments = identify_segments(t, u)
    print(f"Found {len(segments)} activation segments.")
    
    results_k = []
    
    for start, end in segments:
        length = end - start
        if length < 10: continue # Too short
        
        # Only take the middle 80% to avoid transient startups impacting slope
        s_idx = start + int(length * 0.2)
        e_idx = end
        
        frame_t = t[s_idx:e_idx]
        frame_u = u[s_idx:e_idx]
        frame_y = y[s_idx:e_idx]
        
        res = tune_segment(frame_t, frame_u, frame_y)
        if res:
            k, theta, u_mag = res
            results_k.append(k)
            print(f"  Segment [{t[start]:.2f}s - {t[end]:.2f}s]: Input={u_mag:.3f}, Calc K={k:.3f}")

    if not results_k:
        print("No valid segments found to tune.")
        return

    # Average K
    K_avg = np.median(results_k)
    print(f"\n--- SYSTEM IDENTIFICATION ---")
    print(f"System Type: Integrator (Velocity -> Position)")
    print(f"Estimated Gain (K): {K_avg:.4f} (m/s / cmd_vel)")
    
    # --- TUNING RULES (SIMC / Skogestad) ---
    # For Integrating Process G(s) = K / s * e^(-theta*s)
    # PI Settings:
    #   Kc = 1 / (K * (Tc + theta))
    #   Ti = 4 * (Tc + theta)
    #   Td = 0 (Usually)
    
    # We choose Target Response Tc.
    # Aggressive: Tc = theta
    # Smoother: Tc = 3 * theta
    
    theta = 0.06 # Assuming 60ms total system lag (camera + comms) as safe baseline
    
    print(f"\n--- PID TUNING (SIMC Rules) ---")
    print(f"Assumed System Lag (theta): {theta:.3f}s")
    
    for aggressive, mode in [(1.0, "Aggressive"), (2.0, "Moderate"), (4.0, "Smooth")]:
        Tc = aggressive * theta
        
        Kc = 1.0 / (K_avg * (Tc + theta))
        Ti = 4.0 * (Tc + theta)
        Ki = Kc / Ti # Standard PID form: u = Kc*e + Ki*Int + ...
        
        # Adding a small Kd for damping (optional in pure SIMC but good for servo)
        # Kd usually theta/2 * Kc if used
        Kd = 0.5 * theta * Kc 
        
        print(f"\n[{mode} Tuning] (Tc={Tc:.3f}s)")
        print(f"  Kp: {Kc:.4f}")
        print(f"  Ki: {Ki:.4f}") 
        print(f"  Kd: {Kd:.4f}")

if __name__ == "__main__":
    main()
