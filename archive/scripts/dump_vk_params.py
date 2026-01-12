#!/usr/bin/env python3
import numpy as np
from visual_kinematics.RobotDelta import RobotDelta

def main():
    # Params from delta_3dof_controller.py
    # np.array([0.104, 0.040, 0.105, 0.205])
    # [f, e, rf, re]
    params = np.array([0.104, 0.040, 0.105, 0.205])
    robot = RobotDelta(params)
    
    print("--- RobotDelta Internal Params ---")
    try:
        print(f"l1 (rf): {robot.l1}")
    except: pass
    try:
        print(f"l2 (re): {robot.l2}")
    except: pass
    try:
        print(f"r1 (f): {robot.r1}")
    except: pass
    try:
        print(f"r2 (e): {robot.r2}")
    except: pass
    
    try:
        print(f"ap (Attachment Points):\n{robot.ap}")
    except: pass
    
    try:
        print(f"phi (Angles):\n{robot.phi}")
    except: pass
    
if __name__ == "__main__":
    main()
