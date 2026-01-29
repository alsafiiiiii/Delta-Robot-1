#!/usr/bin/env python3
"""
Workspace Visualization for Delta Robot
Run: python3 visualize_workspace.py
"""
import numpy as np
from math import pi
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame

# --- ROBOT GEOMETRY ---
robot = RobotDelta(np.array([0.104, 0.040, 0.105, 0.205]))
robot.ws_lim = np.array([[-pi/12, pi/2]]*3)
robot.ws_division = 10

# --- WORKSPACE ANALYSIS ---
print('=== Workspace Analysis ===')
print(f'Geometry: Base={0.104}m, EE={0.040}m, Bicep={0.105}m, Forearm={0.205}m')
print(f'Joint Limits: {-15}° to {90}°')

# Test some positions
test_angles = [
    [0, 0, 0],           # Top
    [pi/4, pi/4, pi/4],  # Mid
    [pi/2, pi/2, pi/2],  # Bottom
]

print('\n--- Z-Range Analysis ---')
for angles in test_angles:
    frame = robot.forward(np.array(angles))
    pos = frame.t_3_1.flatten()
    print(f'Angles {[round(a*180/pi) for a in angles]}° -> Z={pos[2]:.4f}m')

# --- SHOW GUI ---
print('\n--- Opening 3D Workspace Visualization ---')
robot.show(ws=True)
