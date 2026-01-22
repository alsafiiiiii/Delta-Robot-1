#!/usr/bin/env python3
import math
import numpy as np
from numpy.linalg import inv

class DeltaDynamics:
    """
    Dynamics model for a Delta Parallel Robot.
    Based on 'Method A' (Lagrange Multipliers) from the provided reference thesis.
    """
    def __init__(self, 
                 radius_base=0.104,    # Meters (ra)
                 radius_effector=0.040, # Meters (rb)
                 len_upper_arm=0.105,   # Meters (l1)
                 len_forearm=0.205,     # Meters (l2)
                 mass_upper_arm=0.400,  # kg (m1)
                 mass_forearm=0.040,    # kg (m2 - estimated)
                 mass_effector=0.400,   # kg (mp - moving platform + gripper + payload)
                 gravity=9.81):         # m/s^2
        
        # --- Robot Geometry ---
        self.ra = radius_base
        self.rb = radius_effector
        self.l1 = len_upper_arm
        self.l2 = len_forearm
        
        # Derived parameter: The effective horizontal distance difference between base and effector joints
        # when joints are at 0 (horizontal).
        self.rdif = self.ra - self.rb
        
        # --- Robot Mass Properties ---
        self.m1 = mass_upper_arm
        self.m2 = mass_forearm
        self.mp = mass_effector
        self.g  = gravity
        
        # Precompute constants
        self.sqrt3 = math.sqrt(3.0)
        self.pi = math.pi
        self.sin120 = self.sqrt3 / 2.0
        self.cos120 = -0.5
        self.dtr = math.pi / 180.0
        
        # Friction Parameters (Simple Coulomb + Viscous model placeholders)
        self.friction_coulomb = 0.0 # Static friction torque (Nm)
        self.friction_viscous = 0.0 # Viscous friction coeff (Nm / (rad/s))

        print(f"DeltaDynamics Initialized: L1={self.l1}, L2={self.l2}, m_total_payload={self.mp}")

    def compute_torques(self, position, acceleration_cartesian, joint_angles, joint_accelerations, payload_mass=0.0):
        """
        Computes the required motor torques (Inverse Dynamics).
        
        Args:
            position: [x, y, z] (meters) - Position of end effector
            acceleration_cartesian: [ax, ay, az] (m/s^2) - Acceleration of end effector
            joint_angles: [theta1, theta2, theta3] (deg or rad? Assuming Need to standardize. Reference used Deg, but standard is Rad. I WILL USE RADIANS)
            joint_accelerations: [alpha1, alpha2, alpha3] (rad/s^2)
            payload_mass: Additional mass added to effector (kg)

        Returns:
            [tau1, tau2, tau3] (Nm) - Torque for each motor
        """
        
        # Unpack inputs
        x, y, z = position
        ax, ay, az = acceleration_cartesian
        t1, t2, t3 = joint_angles # Assumed Radians
        a1, a2, a3 = joint_accelerations # Assumed Rad/s^2
        
        # Total moving mass at effector
        m_total = self.mp + payload_mass
        
        # External forces (Gravity + Inertial force of payload)
        # Note: The reference code puts Gravity IN the B matrix acting on mass.
        # F = ma. We are solving for necessary Force/Torque.
        # Implied external force Fp = [0, 0, 0] if only moving itself.
        # If we want to simulate holding a weight, that's in m_total.
        # The equation from reference: 
        # m_B[2] = ((mnt + 3*m2) * (zp_pp - g)) - fpz
        # So we pass ax, ay, az as 'xp_pp', 'yp_pp'...
        
        fp = [0.0, 0.0, 0.0] # No external pushing force, just gravity handling internally
        
        # Setup specific arm angles for the Delta structure (120 deg separation)
        # Phi angles for the 3 arms: 0, 120, 240 degrees
        phi = [0.0, 2.0*self.pi/3.0, 4.0*self.pi/3.0]
        
        # Joint angles array
        theta = [t1, t2, t3]
        theta_acc = [a1, a2, a3]
        
        # 1. Calculate Lagrange Multipliers (Constraint Forces)
        # Represents forces transmitted through the connecting rods
        lambdas = self._solve_lagrange(phi, theta, [x, y, z], fp, [ax, ay, az], m_total)
        
        # 2. Calculate Torque for each arm
        torques = []
        for i in range(3):
            tau = self._calculate_single_arm_torque(
                phi[i], 
                theta_acc[i], 
                theta[i], 
                x, y, z, 
                lambdas[i, 0]
            )
            torques.append(tau)
            
        return np.array(torques)

    def _solve_lagrange(self, phi, theta, pos, force_ext, acc_cartesian, m_payload):
        """
        Solves the system AX = B for Lagrange multipliers.
        """
        x, y, z = pos
        ax, ay, az = acc_cartesian
        fx, fy, fz = force_ext
        
        m_A = np.zeros((3, 3))
        
        # Build Matrix A
        # The reference uses helper functions maxx, mayy, mazz.
        # Corresponds to geometry constraints derivatives.
        for i in range(3):
            # Row 0 (X constraints)
            # 2 * (x - (rdif * cos(phi)) - (l1 * cos(theta) * cos(phi)))
            m_A[0, i] = 2.0 * (x - (self.rdif * math.cos(phi[i])) - 
                               (self.l1 * math.cos(theta[i]) * math.cos(phi[i])))
            
            # Row 1 (Y constraints)
            # 2 * (y - (rdif * sin(phi)) - (l1 * cos(theta) * sin(phi)))
            m_A[1, i] = 2.0 * (y - (self.rdif * math.sin(phi[i])) - 
                               (self.l1 * math.cos(theta[i]) * math.sin(phi[i])))
            
            # Row 2 (Z constraints)
            # 2 * (z - (l1 * sin(theta)))
            m_A[2, i] = 2.0 * (z - (self.l1 * math.sin(theta[i])))
            
        # Build Matrix B
        # Represents sum of forces/accelerations on the platform
        # Note: The reference code includes "3.0 * m2" in the platform mass calculation for B matrix?
        # In reference: m_B[0] = ((mnt + 3*m2) * xp_pp) - fpx
        # This implies it lumps part of the forearm mass onto the platform (common simplification: mass splitting)
        # Ideally mass of rod is split: 1/2 at elbow, 1/2 at effector.
        # If m2 is mass of ONE rod, and there are 2 rods per arm, total 6 rods.
        # Reference 'm2' seems to be "mass of one rod of forearm".
        # But 'mnt' is 'mp + payload'.
        # Let's stick to the reference logic: (Mass_Platform_Total + 3 * M_Rod)
        # This suggests 3 rods' worth of mass is effectively moving with the platform. 
        # (Since 6 rods total, 1/2 of each = 3 full masses). Correct.
        
        mass_term = (m_payload + 3.0 * self.m2)
        
        m_B = np.zeros((3, 1))
        m_B[0, 0] = (mass_term * ax) - fx
        m_B[1, 0] = (mass_term * ay) - fy
        m_B[2, 0] = (mass_term * (az + self.g)) - fz  # Gravity acts on Z
        
        # Solve for Lambda
        # Check singularity?
        try:
            lambdas = inv(m_A).dot(m_B)
        except np.linalg.LinAlgError:
            print("Warning: Singular Matrix in Dynamics! Returning zeros.")
            lambdas = np.zeros((3, 1))
            
        return lambdas

    def _calculate_single_arm_torque(self, phi, theta_acc, theta_val, x, y, z, lambda_val):
        """
        Calculates torque for a single motor joint using the Lagrangian result.
        """
        # Term A: Inertia of the Upper Arm
        # Reference: A = ((m1/3.0) + m2) * l1^2 * theta_pp
        # This assumes Inertia of rod about endpoint is m*L^2/3. 
        # Plus it adds m2 (full mass of ONE rod?). 
        # Mass splitting again: 1/2 m2 at elbow. The reference says " + m2" though.
        # If it adds pure m2, it treats the forearm mass at the elbow as a point mass?
        # Let's follow reference: ((m1 / 3.0) + (m2))
        
        term_inertia = ((self.m1 / 3.0) + self.m2) * (self.l1**2) * theta_acc
        
        # Term B: Gravity on the Upper Arm
        # Reference: B = ((m1/2.0) + m2) * g * l1 * cos(theta)
        # Gravity moment. Center of mass of arm at L1/2. Mass at elbow is m2.
        term_gravity = ((self.m1 / 2.0) + self.m2) * self.g * self.l1 * math.cos(theta_val)
        
        # Term C: Force from the coupling rod (Lagrange Multiplier)
        # This projects the force form the rod (lambda) onto the motor tangent.
        # C1 = ((x * cos(phi) + y * sin(phi) - rdif) * sin(theta))
        # C2 = z * cos(theta)
        # C = C1 - C2
        
        # Project Cartesian Position onto Arm Plane
        # Horizontal dist from base joint to point
        # x_local = x*cos + y*sin
        x_proj = x * math.cos(phi) + y * math.sin(phi)
        
        c1 = (x_proj - self.rdif) * math.sin(theta_val)
        c2 = z * math.cos(theta_val)
        term_geometry = c1 - c2
        
        # Torque from constraint: -2 * lambda * l1 * C
        # (The 2 comes from the derivative of the constraint equation usually)
        term_constraint = -1.0 * 2.0 * lambda_val * self.l1 * term_geometry
        
        # Total Torque
        # Reference: (-1 * 2 * lambda * ...) + Inertia - Gravity (Note signs!)
        # Reference code: torq_3 = (-B). torq = t1*1 + t2*1 + t3*1.
        # So: Constraint + Inertia - Gravity.
        # My term_gravity is positive value. Gravity usually opposes lifting.
        # Angle definition: 0 is horizontal? + is down?
        # Reference Z is usually negative (down).
        # Let's check gravity sign.
        # If arm is horizontal (theta=0), cos(0)=1. Torque gravity = mass*g*L. Positive.
        # If we need to HOLD it there, we apply torque upwards.
        # If theta defined positive UP, gravity tries to lower it (negative torque). Motor must apply Positive.
        # Reference code: torq_3 = -B. 
        # If B is positive, torq_3 is negative.
        # This implies the calculated torque is the "Dynamic Torque" (Sum of forces).
        # Motor Torque = Inertia + Gravity + Friction + Load.
        # If Equation is I*alpha + G = Tau_motor - Tau_load...
        # Let's assume the output is "Required Motor Torque".
        
        torque = term_constraint + term_inertia - term_gravity
        
        # Friction add-on (User request)
        # Viscous: proportional to velocity (not passed yet, assume 0 for now or estimate)
        # Coulomb: constant opposing motion
        # torque += self.friction_viscous * theta_vel + self.friction_coulomb * sign(theta_vel)
        
        return torque

if __name__ == "__main__":
    # Quick sanity check
    bot = DeltaDynamics()
    # Test holding at center
    p = [0, 0, -0.25]
    a_cart = [0, 0, 0] # Static
    q = [0, 0, 0] # Horizontal arms
    q_acc = [0, 0, 0] 
    
    # We need valid IK angles for (0,0,-0.25) to test properly, 
    # but for pure math check:
    tau = bot.compute_torques(p, a_cart, q, q_acc)
    print(f"Test Torque at {p}: {tau}")
