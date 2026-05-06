import numpy as np
from ruckig import Ruckig, InputParameter, OutputParameter

class VectorRuckig:
    """
    Wraps Ruckig to provide 3D Vector Smoothing.
    Instead of smoothing X, Y, Z independently (which causes curves),
    this smooths the *distance* traveled along the line connecting Start->Target.
    
    Result: Perfectly straight 3D lines with Ruckig's S-Curve profiles.
    """
    def __init__(self, dt):
        # We only need 1 DOF (Distance)
        self.otg = Ruckig(1, dt)
        self.inp = InputParameter(1)
        self.out = OutputParameter(1)
        
        self.dt = dt
        
        # State
        self.start_pos = np.array([0.0, 0.0, 0.0])
        self.target_pos = np.array([0.0, 0.0, 0.0])
        self.unit_vector = np.array([0.0, 0.0, 0.0])
        self.total_distance = 0.0
        
        # We track 's' (scalar distance)
        self.current_s = 0.0
        
        # Limits (Scalar)
        self.max_vel = 1.0
        self.max_acc = 1.0
        self.max_jerk = 1.0

    def set_limits(self, max_v, max_a, max_j):
        """Set kinematic limits for the path."""
        self.max_vel = max_v
        self.max_acc = max_a
        self.max_jerk = max_j
        
        self.inp.max_velocity = [max_v]
        self.inp.max_acceleration = [max_a]
        self.inp.max_jerk = [max_j]

    def set_target(self, current, target):
        """
        Set a new 3D target.
        This resets the 1D progress 's' to 0 and defines a new line.
        
        Args:
            current (list): [x, y, z] current position
            target (list): [x, y, z] target position
        """
        # If we are already close, do nothing or just update target?
        # A full reset is safer for straight lines to ensure we strictly follow the new vector.
        
        self.start_pos = np.array(current)
        self.target_pos = np.array(target)
        
        diff = self.target_pos - self.start_pos
        dist = np.linalg.norm(diff)
        
        if dist < 0.00001:
            self.unit_vector = np.array([0.0, 0.0, 0.0])
            self.total_distance = 0.0
        else:
            self.unit_vector = diff / dist
            self.total_distance = dist
            
        # Reset 1D Planner
        # We are at '0' distance along this new vector
        self.current_s = 0.0
        self.inp.current_position = [0.0]
        self.inp.current_velocity = [0.0]     # Assume stop at node change? Or project velocity?
        self.inp.current_acceleration = [0.0]
        
        self.inp.target_position = [self.total_distance]
        self.inp.target_velocity = [0.0]
        self.inp.target_acceleration = [0.0]

    def update(self):
        """
        Run Ruckig for this step.
        Returns:
            (x, y, z) : The new smoothed 3D position
            done (bool): True if target reached
        """
        # Run OTG
        result = self.otg.update(self.inp, self.out)
        
        # Update State
        self.current_s = self.out.new_position[0]
        
        # Feedback for next loop
        self.inp.current_position = self.out.new_position
        self.inp.current_velocity = self.out.new_velocity
        self.inp.current_acceleration = self.out.new_acceleration
        
        # Calculate 3D Position
        # Pos = Start + UnitVec * s
        new_pos = self.start_pos + (self.unit_vector * self.current_s)
        
        is_done = (result == 0) # Result 0 is 'Finished'
        
        return new_pos, is_done
