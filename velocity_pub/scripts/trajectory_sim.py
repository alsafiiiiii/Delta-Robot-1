import matplotlib.pyplot as plt
import numpy as np
import math

# --- CONFIGURATION MATCHING FIRMWARE ---
TRAJ_MAX_VEL = 500.0   # deg/s
TRAJ_ACCEL   = 2000.0  # deg/s^2
TRAJ_DECEL   = 20000.0 # deg/s^2 (Instant Stop)
LOOP_RATE    = 250.0   # Hz
DT           = 1.0 / LOOP_RATE

class TrajectorySim:
    def __init__(self, max_vel, acc, dec, thresh=0.1):
        self.target = 0.0
        self.cur_pos = 90.0
        self.cur_vel = 0.0
        
        self.max_vel = max_vel
        self.acc = acc
        self.dec = dec
        self.thresh = thresh
        self.vel_goal = max_vel

    def set_target(self, target):
        self.target = target
        self.vel_goal = self.max_vel

    def update(self):
        # Position Control Logic (Exact port from C)
        pos_error = self.target - self.cur_pos
        
        if abs(pos_error) > self.thresh:
            direction = 1.0 if pos_error > 0 else -1.0
            
            # Determine accel vs decel
            # Decel distance needed = v^2 / 2a
            decel_dist = (self.cur_vel**2) / (2 * self.dec)
            
            if decel_dist >= abs(pos_error):
                acceleration = -self.dec # Brake!
            else:
                acceleration = self.acc  # Accelerate
            
            # Update Velocity
            if direction > 0:
                self.cur_vel += acceleration * DT
            else:
                self.cur_vel -= acceleration * DT
                
            # Clamp Velocity
            if self.cur_vel > self.vel_goal: self.cur_vel = self.vel_goal
            if self.cur_vel < -self.vel_goal: self.cur_vel = -self.vel_goal
            
            # Update Position
            dp = self.cur_vel * DT
            
            # Anti-overshoot
            if abs(dp) < abs(pos_error):
                self.cur_pos += dp
            else:
                self.cur_pos = self.target
                self.cur_vel = 0 # Stop
                
        else:
            self.cur_pos = self.target
            self.cur_vel = 0
            
        return self.cur_pos, self.cur_vel

# --- TEST SCENARIO ---
sim = TrajectorySim(TRAJ_MAX_VEL, TRAJ_ACCEL, TRAJ_DECEL)

times = []
positions = []
velocities = []
targets = []

# 1. 0.0s -> 0.5s: Move from 90 to 120 (Accelerate)
sim.set_target(120.0)
for i in range(int(0.5 * LOOP_RATE)):
    pos, vel = sim.update()
    times.append(i * DT)
    positions.append(pos)
    velocities.append(vel)
    targets.append(120.0)

# 2. 0.5s -> 1.0s: Let go of stick (Target becomes Current Pos -> Instant Stop?)
# In Joystick mode, when user lets go, target goes to 90 (Home) usually?
# User said "letting go means stop" -> No, usually it means recenter?
# Or if it's throttle control: "deviation from center controls speed". 
# If they let go, deviation is 0, so speed is 0. 
# BUT we are sending POSITIONS. 
# If the joystick script stops sending, the robot holds last position.
# Let's simulate "Change Target to 90" (Snap back)
sim.set_target(90.0)
start_t = len(times) * DT
for i in range(int(0.5 * LOOP_RATE)):
    pos, vel = sim.update()
    times.append(start_t + i * DT)
    positions.append(pos)
    velocities.append(vel)
    targets.append(90.0)

# --- PLOTTING ---
try:
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    
    ax1.plot(times, targets, 'r--', label='Target')
    ax1.plot(times, positions, 'b-', label='Position (Calculated)')
    ax1.set_ylabel('Degrees')
    ax1.legend()
    ax1.grid(True)
    ax1.set_title(f'Trajectory Simulation (Decel={TRAJ_DECEL})')
    
    ax2.plot(times, velocities, 'g-', label='Velocity')
    ax2.set_ylabel('Deg/s')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True)
    
    output_file = 'trajectory_test.png'
    plt.savefig(output_file)
    print(f"Simulation complete. Plot saved to {output_file}")
    
except Exception as e:
    print(f"Could not plot: {e}")
