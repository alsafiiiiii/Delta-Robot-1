#!/usr/bin/env python3
"""
Generate dense precomputed trajectory for ESP32 standalone test.
Uses Quintic interpolation between star points with linear Cartesian paths.
Outputs C code with all intermediate points.
"""
import numpy as np
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame
import math

robot = RobotDelta(np.array([0.104, 0.040, 0.105, 0.205]))

# Star waypoints from gcode (X, Y, Z in meters)
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

# Settings
POINTS_PER_SEGMENT = 30  # Intermediate points between waypoints
MOVE_TIME_MS = 1500      # Time per segment in ms

def quintic_scale(t):
    """Returns 0-1 position along quintic curve for t in [0,1]"""
    return 10*t**3 - 15*t**4 + 6*t**5

def interpolate_segment(start, end, n_points):
    """Generate n_points along a straight line from start to end with quintic timing."""
    points = []
    for i in range(n_points):
        t = i / (n_points - 1) if n_points > 1 else 1.0
        s = quintic_scale(t)  # Quintic position
        x = start[0] + (end[0] - start[0]) * s
        y = start[1] + (end[1] - start[1]) * s
        z = start[2] + (end[2] - start[2]) * s
        points.append((x, y, z))
    return points

def compute_ik(x, y, z):
    """Compute joint angles in degrees for Cartesian position."""
    try:
        frame = Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[x], [y], [z]]))
        angles = robot.inverse(frame).flatten()
        return [math.degrees(a) for a in angles]
    except Exception as e:
        print(f"// IK Error at ({x:.3f}, {y:.3f}, {z:.3f}): {e}")
        return None

# Generate all trajectory points
all_points = []
for i in range(len(star_waypoints) - 1):
    segment = interpolate_segment(star_waypoints[i], star_waypoints[i+1], POINTS_PER_SEGMENT)
    all_points.extend(segment[:-1])  # Exclude last to avoid duplicates
all_points.append(star_waypoints[-1])  # Add final point

# Compute IK for all points
trajectory = []
for x, y, z in all_points:
    angles = compute_ik(x, y, z)
    if angles:
        trajectory.append(angles)

# Output C code
print(f"""/**
 * Precomputed Star Trajectory with Linear Cartesian Paths
 * Generated with Quintic velocity profile
 * {len(trajectory)} points, ~{MOVE_TIME_MS // POINTS_PER_SEGMENT}ms per point
 */

const float trajectory[][3] = {{""")

for i, angles in enumerate(trajectory):
    print(f"    {{{angles[0]:.2f}f, {angles[1]:.2f}f, {angles[2]:.2f}f}},")

print(f"""}};
#define TRAJECTORY_POINTS {len(trajectory)}
#define POINT_DELAY_MS {MOVE_TIME_MS // POINTS_PER_SEGMENT}  // Time between each point
""")

print(f"// Total trajectory: {len(trajectory)} points")
print(f"// Estimated cycle time: {len(trajectory) * MOVE_TIME_MS // POINTS_PER_SEGMENT / 1000:.1f} seconds")
