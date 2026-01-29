#!/usr/bin/env python3
import numpy as np

class QuinticGenerator:
    """
    Generates a generic 5th-order polynomial trajectory (Minimum Jerk Trajectory).
    Equation: p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    
    Simplified for Start/End Velocity=0 and Acceleration=0:
    Normalized Time tau = t / TotalDuration
    Scaling Function s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
    Position = Start + (End - Start) * s(tau)
    Velocity = (End - Start) * (30*tau^2 - 60*tau^3 + 30*tau^4) / Duration
    Acceleration = (End - Start) * (60*tau - 180*tau^2 + 120*tau^3) / Duration^2
    """
    
    def __init__(self, start_pos, end_pos, average_speed=0.0):
        """
        Args:
            start_pos: np.array([x, y, z])
            end_pos: np.array([x, y, z])
            average_speed: float (m/s). Use this OR duration. 
        """
        self.start = np.array(start_pos, dtype=float)
        self.end = np.array(end_pos, dtype=float)
        
        # Calculate Duration based on Distance and Speed
        distance = np.linalg.norm(self.end - self.start)
        
        # Avoid division by zero
        if distance < 1e-6:
            self.duration = 0.05 # Tiny duration for practically 0 move
        elif average_speed <= 0:
             self.duration = 1.0 # Default fallback
        else:
            # Quintic peak velocity is approx 1.875 * average_velocity
            # If we want to maintain a specific 'average' cruise, T = dist / speed
            self.duration = distance / average_speed
            
        self.time_elapsed = 0.0
        self.finished = False

    def get_state(self, dt):
        """
        Advance time by dt and return current state.
        Returns:
            (position, velocity, acceleration) as np.arrays
        """
        self.time_elapsed += dt
        
        # Check completion
        if self.time_elapsed >= self.duration:
            self.finished = True
            return self.end, np.zeros(3), np.zeros(3)
        
        # Normalized time 0.0 to 1.0
        tau = self.time_elapsed / self.duration
        tau2 = tau * tau
        tau3 = tau2 * tau
        tau4 = tau3 * tau
        tau5 = tau4 * tau
        
        # Polynomial Scaling factors
        # s(tau) for position
        scale_pos = 10*tau3 - 15*tau4 + 6*tau5
        
        # s'(tau) for velocity (Derivative wrt tau)
        # d/dtau = 30t^2 - 60t^3 + 30t^4
        scale_vel = 30*tau2 - 60*tau3 + 30*tau4
        
        # s''(tau) for acceleration
        # d2/dtau2 = 60t - 180t^2 + 120t^3
        scale_acc = 60*tau - 180*tau2 + 120*tau3
        
        # Vectors
        diff = self.end - self.start
        
        # Position
        pos = self.start + diff * scale_pos
        
        # Velocity (Chain rule: d/dt = d/dtau * dtau/dt = d/dtau * 1/T)
        vel = diff * scale_vel / self.duration
        
        # Acceleration (Chain rule: d2/dt2 = d2/dtau2 * 1/T^2)
        acc = diff * scale_acc / (self.duration ** 2)
        
        return pos, vel, acc
