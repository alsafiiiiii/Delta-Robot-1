#!/usr/bin/env python3
"""
Delta Robot Utilities
Shared constants and helper functions for Delta Robot controllers.
"""

import math
import numpy as np

# ==================== WORKSPACE LIMITS ====================
# These define the operational bounds of the robot

WS_X_MIN = -0.15
WS_X_MAX = 0.15
WS_Y_MIN = -0.15
WS_Y_MAX = 0.15
WS_Z_MIN = -0.32
WS_Z_MAX = -0.15

# ==================== HELPER FUNCTIONS ====================

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion (x, y, z, w).
    
    Args:
        roll: Rotation around X axis (radians)
        pitch: Rotation around Y axis (radians)
        yaw: Rotation around Z axis (radians)
    
    Returns:
        Tuple of (qx, qy, qz, qw)
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    return (qx, qy, qz, qw)


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp value between min and max."""
    return max(min_val, min(max_val, value))


def clamp_workspace(x: float, y: float, z: float) -> tuple:
    """Clamp XYZ position to workspace limits."""
    return (
        clamp(x, WS_X_MIN, WS_X_MAX),
        clamp(y, WS_Y_MIN, WS_Y_MAX),
        clamp(z, WS_Z_MIN, WS_Z_MAX)
    )


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def in_workspace(x: float, y: float, z: float = None) -> bool:
    """Check if position is within workspace limits."""
    if x < WS_X_MIN or x > WS_X_MAX:
        return False
    if y < WS_Y_MIN or y > WS_Y_MAX:
        return False
    if z is not None and (z < WS_Z_MIN or z > WS_Z_MAX):
        return False
    return True
