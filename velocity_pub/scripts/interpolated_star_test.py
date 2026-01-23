#!/usr/bin/env python3
"""
Interpolated Star Test Driver
Sends "T:deg D:ms" commands to the new interpolating ESP32 firmware.
"""
import serial
import time
import numpy as np
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame
import math

# Configuration
SERIAL_PORT = '/dev/ttyUSB0'  # Adjust as needed (e.g., /dev/ttyACM0)
BAUD_RATE = 115200
MOVE_TIME_MS = 1500  # Time per segment

# Robot Setup
robot = RobotDelta(np.array([0.104, 0.040, 0.105, 0.205]))

# Star Waypoints (X, Y, Z)
star_waypoints = [
    (0.0, 0.0, -0.22),       # Start center
    (0.0, 0.05, -0.22),      # Point 1 (Top)
    (0.0, 0.05, -0.228),     # Drop down
    (0.029, -0.040, -0.228), # Point 3
    (-0.047, 0.015, -0.228), # Point 5
    (0.047, 0.015, -0.228),  # Point 2
    (-0.029, -0.040, -0.228),# Point 4
    (0.0, 0.05, -0.228),     # Back to Point 1
    (0.0, 0.05, -0.22),      # Lift
    (0.0, 0.0, -0.22),       # Center
]

def compute_ik(x, y, z):
    frame = Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[x], [y], [z]]))
    angles = robot.inverse(frame).flatten()
    return [math.degrees(a) for a in angles]

def send_command(ser, servo_idx, angle, duration):
    cmd = f"T{servo_idx}:{angle:.2f} D:{duration}\n"
    ser.write(cmd.encode())
    # print(f"Sent: {cmd.strip()}")

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}")
        time.sleep(2)  # Wait for ESP32 reset
        
        while True:
            print(f"Starting Star Pattern...")
            
            for i, (x, y, z) in enumerate(star_waypoints):
                print(f"Moving to Point {i}: ({x:.3f}, {y:.3f}, {z:.3f})")
                
                angles = compute_ik(x, y, z)
                
                # Send commands for all 3 servos
                for idx in range(3):
                    send_command(ser, idx, angles[idx], MOVE_TIME_MS)
                
                # Wait for move to complete + small buffer
                time.sleep(MOVE_TIME_MS / 1000.0 + 0.1)
                
    except serial.SerialException as e:
        print(f"Serial Error: {e}")
    except KeyboardInterrupt:
        print("\nTest Stopped")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
