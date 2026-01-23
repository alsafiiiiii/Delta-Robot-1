import numpy as np
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame
import math

robot = RobotDelta(np.array([0.104, 0.040, 0.105, 0.205]))

# Star points from gcode (X, Y, Z in meters)
star_points = [
    (0.0, 0.0, -0.22),      # Start center
    (0.0, 0.05, -0.22),     # Point 1 (Top)
    (0.0, 0.05, -0.228),    # Drop down
    (0.029, -0.040, -0.228), # Point 3
    (-0.047, 0.015, -0.228), # Point 5
    (0.047, 0.015, -0.228),  # Point 2
    (-0.029, -0.040, -0.228),# Point 4
    (0.0, 0.05, -0.228),     # Back to Point 1
    (0.0, 0.05, -0.22),      # Lift
    (0.0, 0.0, -0.22),       # Center
]

print('// Precomputed star trajectory - Joint angles in DEGREES')
print('// Format: {servo1_deg, servo2_deg, servo3_deg}')
print('const float star_trajectory[][3] = {')

for i, (x, y, z) in enumerate(star_points):
    try:
        frame = Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[x], [y], [z]]))
        angles = robot.inverse(frame).flatten()
        # Convert to degrees
        deg1 = math.degrees(angles[0])
        deg2 = math.degrees(angles[1])
        deg3 = math.degrees(angles[2])
        print(f'    {{{deg1:.2f}f, {deg2:.2f}f, {deg3:.2f}f}}, // Point {i}: X={x:.3f} Y={y:.3f} Z={z:.3f}')
    except Exception as e:
        print(f'    // Error at point {i}: {e}')

print('};')
print(f'const int STAR_POINTS = {len(star_points)};')