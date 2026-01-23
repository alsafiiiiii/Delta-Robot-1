#!/usr/bin/env python3
"""
G-Code Interpolated Driver
Parses G-code files and sends "T:deg D:ms" commands to the new interpolating ESP32 firmware.
Calculates duration based on F (Feedrate in mm/min) and 3D distance.
"""
import serial
import time
import numpy as np
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame
import math
import sys
import os

# Configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
DEFAULT_FEEDRATE = 500 # mm/min

# Robot Setup
robot = RobotDelta(np.array([0.104, 0.040, 0.105, 0.205]))

def compute_ik(x, y, z):
    """Compute servo angles (degrees) for Cartesian position (meters)"""
    try:
        frame = Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[x], [y], [z]]))
        angles = robot.inverse(frame).flatten()
        return [math.degrees(a) for a in angles]
    except Exception as e:
        print(f"IK Error: {e}")
        return None

def send_move(ser, angles, duration_ms):
    """Send target commands to all 3 servos"""
    if duration_ms < 20: duration_ms = 20 # Minimum safety
    
    for idx, angle in enumerate(angles):
        cmd = f"T{idx}:{angle:.2f} D:{int(duration_ms)}\n"
        ser.write(cmd.encode())
    
    # Wait for completion
    time.sleep(duration_ms / 1000.0)

def parse_gcode_line(line, current_pos, current_feedrate):
    """Parse G-code line and update state"""
    parts = line.strip().split(';') # Remove comments
    clean_line = parts[0].strip().upper()
    if not clean_line:
        return current_pos, current_feedrate, None

    # Parse G0/G1
    if 'G0' in clean_line or 'G1' in clean_line:
        new_pos = list(current_pos)
        
        # Parse X, Y, Z
        tokens = clean_line.split()
        for token in tokens:
            if token.startswith('X'): new_pos[0] = float(token[1:])
            if token.startswith('Y'): new_pos[1] = float(token[1:])
            if token.startswith('Z'): new_pos[2] = float(token[1:])
            if token.startswith('F'): current_feedrate = float(token[1:])
        
        return tuple(new_pos), current_feedrate, "MOVE"
    
    return current_pos, current_feedrate, None

def run_gcode_file(filename, ser):
    print(f"Running {filename}...")
    
    current_pos = (0.0, 0.0, -0.22) # Default home
    current_feedrate = DEFAULT_FEEDRATE
    
    # Initial move to home
    print("Homing...")
    angles = compute_ik(*current_pos)
    send_move(ser, angles, 500) # Fast homing (0.5s)
    
    with open(filename, 'r') as f:
        for line in f:
            next_pos, next_feedrate, action = parse_gcode_line(line, current_pos, current_feedrate)
            
            if action == "MOVE":
                # Calculate Distance (meters)
                dist = math.sqrt((next_pos[0]-current_pos[0])**2 + 
                                 (next_pos[1]-current_pos[1])**2 + 
                                 (next_pos[2]-current_pos[2])**2)
                
                # Check for zero distance
                if dist < 0.0001:
                    continue
                
                # Calculate Duration
                # SCALING FIX: User wants faster moves without huge F values.
                # Standard G-code: F is mm/min.
                # NEW LOGIC: Treat F as mm/s (60x faster) OR just scale up.
                # Let's keep F as mm/min but add a global speed multiplier of 60x 
                # effectively making F = mm/s for convenience.
                
                speed_mm_s = next_feedrate  # Treat F as mm/s directly
                speed_m_s = speed_mm_s / 1000.0
                
                if speed_m_s <= 0: speed_m_s = 0.001 # Safety
                
                duration_s = dist / speed_m_s
                duration_ms = int(duration_s * 1000)
                
                print(f"Move: {current_pos} -> {next_pos} | Dist: {dist*1000:.1f}mm | F{next_feedrate} | Time: {duration_ms}ms")
                
                angles = compute_ik(*next_pos)
                if angles:
                    send_move(ser, angles, duration_ms)
                
                current_pos = next_pos
                current_feedrate = next_feedrate

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 gcode_interpolated_driver.py <file.gcode>")
        sys.exit(1)
        
    filename = sys.argv[1]
    if not os.path.exists(filename):
        print(f"File not found: {filename}")
        sys.exit(1)

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}")
        time.sleep(2)  # Wait for ESP32 reset
        
        while True:
            run_gcode_file(filename, ser)
            print("Finished file. Looping in 2s...")
            time.sleep(2)
                
    except serial.SerialException as e:
        print(f"Serial Error: {e}")
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
