#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from quintic_trajectory import QuinticGenerator

def visualize_trajectory():
    print("Generating Trajectory Visualization...")
    
    # Define a move
    start_pos = [0, 0, 0]
    end_pos = [0.1, 0.05, -0.05] # Move 10cm in X, 5cm in Y, -5cm in Z
    avg_speed = 0.05 # 5 cm/s
    
    # Create Generator
    gen = QuinticGenerator(start_pos, end_pos, average_speed=avg_speed)
    print(f"Movement Duration: {gen.duration:.2f} s")
    
    # Simulate Loop
    dt = 0.01 # 100Hz simulation
    times = []
    positions = []
    velocities = []
    accelerations = []
    
    t = 0
    curr_pos = start_pos
    
    while not gen.finished:
        p, v, a = gen.get_state(dt)
        times.append(gen.time_elapsed)
        positions.append(p)
        velocities.append(np.linalg.norm(v)) # Speed Magnitude
        accelerations.append(np.linalg.norm(a)) # Accel Magnitude
    
    # Plotting
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(8, 10))
    
    # Position
    pos_np = np.array(positions)
    ax1.plot(times, pos_np[:, 0], label='X')
    ax1.plot(times, pos_np[:, 1], label='Y')
    ax1.plot(times, pos_np[:, 2], label='Z')
    ax1.set_ylabel('Position (m)')
    ax1.set_title('Quintic Trajectory Profile')
    ax1.legend()
    ax1.grid(True)
    
    # Velocity
    ax2.plot(times, velocities, color='orange')
    ax2.set_ylabel('Speed (m/s)')
    ax2.set_title('Velocity Profile (Bell Curve)')
    ax2.grid(True)
    
    # Acceleration
    ax3.plot(times, accelerations, color='green')
    ax3.set_ylabel('Accel (m/s^2)')
    ax3.set_xlabel('Time (s)')
    ax3.set_title('Acceleration Profile (Smooth Start/Stop)')
    ax3.grid(True)
    
    output_file = 'trajectory_plot.png'
    plt.savefig(output_file)
    print(f"Plot saved to {output_file}")
    plt.show()

if __name__ == "__main__":
    visualize_trajectory()
