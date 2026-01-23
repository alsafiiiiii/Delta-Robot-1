#!/usr/bin/env python3
import numpy as np
from quintic_trajectory import QuinticGenerator

def test_straight_line():
    print("Testing Quintic Trajectory...")
    start = [0, 0, 0]
    end = [1, 1, 1]
    
    # 1.414m distance. Speed 1.0 m/s -> Duration 1.73s
    gen = QuinticGenerator(start, end, average_speed=1.0)
    
    print(f"Duration: {gen.duration:.2f}s")
    
    # Check Start (t=0)
    p, v, a = gen.get_state(0.0)
    print(f"Start Pos: {p} (Expected {start})")
    print(f"Start Vel: {v} (Expected [0,0,0])")
    
    # Check Midpoint (t=duration/2)
    # Quintic passes 0.5 at t=0.5 normalized
    p, v, a = gen.get_state(gen.duration / 2.0)
    print(f"Mid Pos: {p} (Expected [0.5, 0.5, 0.5])")
    print(f"Mid Vel: {np.linalg.norm(v):.2f} m/s (Peak should be ~1.875)")
    
    # Check End
    # Advance to end
    p, v, a = gen.get_state(gen.duration)
    print(f"End Pos: {p} (Expected {end})")
    print(f"End Vel: {v} (Expected [0,0,0])")
    
    if np.allclose(p, end) and np.allclose(v, [0,0,0]):
        print("PASS: Trajectory reached target with 0 velocity.")
    else:
        print("FAIL: Did not reach target or nonzero velocity.")

if __name__ == "__main__":
    test_straight_line()
