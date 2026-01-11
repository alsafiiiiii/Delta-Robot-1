import socket
import struct
import time
import math
import numpy as np
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame

# --- CONFIGURATION ---
ESP_IP = "10.52.82.11"  # Set to your ESP32 IP
ESP_PORT = 3333
FREQUENCY = 50.0       # 50Hz Update Rate
DT = 1.0 / FREQUENCY

# --- ROBOT GEOMETRY ---
# (Matches your delta_ik_controller.py / physical robot)
R_BASE = 0.07582127019
R_EE = 0.035
L1 = 0.075
L2 = 0.2639602098

# --- STAR PARAMETERS ---
STAR_RADIUS = 0.05   # Meters (5cm)
DRAW_Z = -0.23       # Drawing Height
SAFE_Z = -0.20       # Travel Height
SPEED = 0.05         # m/s

def get_star_points(radius, z_height):
    """
    Generates vertices of a pentagram (5-pointed star).
    Order: Top -> BottomRight -> TopLeft -> TopRight -> BottomLeft -> Top
    """
    points = []
    # Angles for the 5 outer points (Top starts at 90 deg)
    # 90, 90+144=234, 234+144=378(18), 18+144=162, 162+144=306
    # Correct order for continuous drawing:
    # 1. 90 (Top)
    # 2. 306 (-54) (Bottom Right)
    # 3. 162 (Top Left)
    # 4. 18 (Top Right)
    # 5. 234 (-126) (Bottom Left)
    # 6. 90 (Close)
    
    angles = [90, 306, 162, 18, 234, 90]
    
    for deg in angles:
        rad = math.radians(deg)
        x = radius * math.cos(rad)
        y = radius * math.sin(rad)
        points.append(np.array([x, y, z_height]))
        
    return points

def setup_robot():
    return RobotDelta(np.array([R_BASE, R_EE, L1, L2]))

def ik_solve(robot, x, y, z):
    """
    Returns joint angles in degrees for a given position.
    """
    # Create target frame (Position only, no rotation needed for basic IK)
    # visual_kinematics expects a specific frame format usually
    # Frame.from_euler_3(rot_arr, pos_arr)
    t = np.array([[x], [y], [z]])
    f = Frame.from_euler_3(np.array([0., 0., 0.]), t)
    
    # Solve
    # Returns [theta1, theta2, theta3] in RADIANS
    rads = robot.inverse(f).flatten()
    degs = np.rad2deg(rads)
    return degs

def main():
    print("--- Delta Robot Star Drawer (No ROS) ---")
    print(f"Target: {ESP_IP}:{ESP_PORT}")
    
    # 1. Setup UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    esp_addr = (ESP_IP, ESP_PORT)
    
    # 2. Setup Robot
    robot = setup_robot()
    
    # 3. Generate Path
    print("Generating Path...")
    vertices = get_star_points(STAR_RADIUS, DRAW_Z)
    
    # Start at Safe Z above first point
    start_pt = vertices[0].copy()
    start_pt[2] = SAFE_Z
    
    # Sequence of points to visit
    # Home -> Start(Safe) -> Start(Draw) -> [Vertices] -> End(Draw) -> End(Safe) -> Home
    path_sequence = [
        np.array([0.0, 0.0, SAFE_Z]), # Home
        start_pt,                     # Move to above Point 1
        vertices[0]                   # Lower to Point 1
    ] + vertices[1:] + [              # Draw Star
        vertices[0],                  # Ensure closed
        start_pt,                     # Lift
        np.array([0.0, 0.0, SAFE_Z])  # Home
    ]

    print("Sending Trajectory...")
    
    try:
        current_pos = np.array([0.0, 0.0, SAFE_Z])
        
        for target_pos in path_sequence:
            print(f"Target: {target_pos}")
            
            # Linear Interpolation (Move to Target)
            dist = np.linalg.norm(target_pos - current_pos)
            if dist < 0.0001: continue
            
            duration = dist / SPEED
            steps = int(duration * FREQUENCY)
            if steps < 1: steps = 1

            for step in range(steps):
                # Interpolate
                t = float(step) / steps
                interp_pos = current_pos + (target_pos - current_pos) * t
                
                # IK
                angles = ik_solve(robot, interp_pos[0], interp_pos[1], interp_pos[2])
                
                # Prepare Packet: <fffff (5 floats)
                # We only have 3 arm angles. 
                # Packet struct requires 5 floats? Checking robot_control.py...
                # Yes: struct.pack('<fffff', *angles_deg) usually with 0s for rest
                # Ensure angles are 0-180 (Clip)
                angles = np.clip(angles, 0, 180)
                
                # Payload: [J1, J2, J3, 0, 0]
                payload = struct.pack('<fffff', 
                                    angles[0], angles[1], angles[2], 
                                    0.0, 0.0)
                
                sock.sendto(payload, esp_addr)
                
                # Precision Timing
                time.sleep(DT)
            
            # Update current position
            current_pos = target_pos
            
    except KeyboardInterrupt:
        print("\nAborted.")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        sock.close()
        print("Done.")

if __name__ == "__main__":
    main()
