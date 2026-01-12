#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
import numpy as np
import math
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame

class TrapezoidalProfile:
    def __init__(self, max_vel=0.5, accel=1.0, decel=1.0):
        self.max_vel = max_vel
        self.accel = accel
        self.decel = decel
        self.current_pos = 0.0
        self.current_vel = 0.0
        self.target_pos = 0.0
        self.threshold = 0.0001
        
    def set_pid(self, max_vel, accel, decel=None):
        self.max_vel = max_vel
        self.accel = accel
        self.decel = decel if decel else accel # Default to symmetric if not provided
        
    def set_target(self, target):
        self.target_pos = target
        
    def update(self, dt):
        pos_error = self.target_pos - self.current_pos
        
        if abs(pos_error) < self.threshold:
            self.current_vel = 0.0
            self.current_pos = self.target_pos
            return self.current_pos, True # Done
            
        # Determine direction
        direction = 1.0 if pos_error > 0 else -1.0
        
        # Distance needed to stop from current velocity
        stop_dist = (self.current_vel**2) / (2.0 * self.decel)
        
        # Decide whether to Accelerate or Decelerate
        target_vel = 0.0
        
        # If we are close to target and need to stop
        if abs(pos_error) <= stop_dist:
            # We must decelerate
            target_vel = 0.0
        else:
            # Cruise at max velocity
            target_vel = self.max_vel * direction
            
        # Apply Acceleration/Deceleration limits
        vel_diff = target_vel - self.current_vel
        max_vel_change = self.accel * dt
        
        if abs(vel_diff) > max_vel_change:
            self.current_vel += math.copysign(max_vel_change, vel_diff)
        else:
            self.current_vel = target_vel
            
        # Deadlock Prevention: If velocity is 0 but not at target (and close enough), force finish
        # Reduced tolerance to avoid visible snaps (10x -> 2x = 0.2mm)
        if abs(self.current_vel) < 0.0001 and abs(pos_error) < (self.threshold * 2.0):
             self.current_pos = self.target_pos
             self.current_vel = 0.0
             return self.current_pos, True
            
        # Update Position
        self.current_pos += self.current_vel * dt
        
        return self.current_pos, False # Not Done

class SmoothDeltaController(Node):
    def __init__(self):
        super().__init__('smooth_delta_controller')
        
        # --- Config ---
        self.loop_rate = 50.0  # Hz
        self.dt = 1.0 / self.loop_rate
        
        # Default Kinematics (set later based on param)
        self.robot = None 
        
        # Motion Profile (Using one profile for Linear Distance)
        self.profile = TrapezoidalProfile(0.1, 0.5)
        
        # Linear Interpolation State
        self.start_pos = np.array([0.0, 0.0, 0.0])
        self.target_pos = np.array([0.0, 0.0, 0.0])
        self.move_vector = np.array([0.0, 0.0, 0.0])
        self.move_distance = 0.0
        
        # State
        self.is_moving = False
        self.real_angles = None # Feedback
        
        # Pub/Sub
        self.joint_pub = self.create_publisher(JointTrajectory, '/model/delta_robot/joint_trajectory', 10)
        self.done_pub = self.create_publisher(Bool, '/delta/movement_done', 10)
        
        self.pose_sub = self.create_subscription(Pose, '/delta/target_pose', self.new_target_callback, 10)
        self.speed_sub = self.create_subscription(Twist, '/delta/speed_params', self.speed_callback, 10)
        
        # Feedback Subscription
        self.declare_parameter('use_sim', False)
        self.use_sim = self.get_parameter('use_sim').value

        # Robot Parameters [SB, WB, UP, WP] => [R_Base, R_EE, L1, L2]
        # Simulation Parameters (Provided by User)
        #self.robot = RobotDelta(np.array([0.0758, 0.035, 0.075, 0.2639]))
        # Hardware Parameters (Existing)
        self.robot = RobotDelta(np.array([0.104, 0.040, 0.105, 0.205])) 

        if self.use_sim:
            # Gazebo Feedback
            self.fb_sub = self.create_subscription(JointState, '/joint_states', self.feedback_callback, 10)
            self.get_logger().info("Mode: SIMULATION (Listening to /joint_states)")
        else:
            # Hardware Feedback
            self.fb_sub = self.create_subscription(JointState, '/delta/real_joints', self.feedback_callback, 10)
            self.get_logger().info("Mode: HARDWARE (Listening to /delta/real_joints)")
        
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # Initial Homing
        self.set_home(-0.22)
        
        self.get_logger().info('Smooth 3DOF Controller Ready (Trapezoidal Mode)')

    def set_home(self, z_height):
        # Force position to home
        self.start_pos = np.array([0.0, 0.0, z_height])
        self.target_pos = np.array([0.0, 0.0, z_height])
        self.profile.current_pos = 0.0
        self.profile.target_pos = 0.0
        self.move_distance = 0.0
        self.solve_and_publish(0.0, 0.0, z_height)

    def speed_callback(self, msg):
        v_lin = max(0.001, msg.linear.x) # Linear Velocity
        
        # Allow dynamic tuning from Twist
        # x = Vel, y = Accel, z = Decel
        acc_lin = msg.linear.y if msg.linear.y > 0.001 else 0.5 
        dec_lin = msg.linear.z if msg.linear.z > 0.001 else 0.2 
        
        self.profile.set_pid(v_lin, acc_lin, dec_lin)
        
        self.get_logger().info(f"Speed Updated: V={v_lin:.2f}, A={acc_lin:.2f}, D={dec_lin:.2f}")

    def feedback_callback(self, msg):
        # Robustly map joint names to angles
        # We need 'jbf1', 'jbf2', 'jbf3' for the 3 main axes
        try:
            if 'jbf1' in msg.name and 'jbf2' in msg.name and 'jbf3' in msg.name:
                idx1 = msg.name.index('jbf1')
                idx2 = msg.name.index('jbf2')
                idx3 = msg.name.index('jbf3')
                
                # Store in fixed order [Angle1, Angle2, Angle3]
                # Sim might return full state, we just want the arms
                self.real_angles = [msg.position[idx1], msg.position[idx2], msg.position[idx3]]
            else:
                # Fallback if names missing (shouldn't happen with our bridge/gazebo)
                self.real_angles = msg.position[:3]
        except ValueError:
            pass

    def new_target_callback(self, msg):
        # 1. Determine Start Position
        # If interrupting a move, use current calculated pos
        if self.is_moving and self.move_distance > 0.000001:
             s = self.profile.current_pos
             ratio = s / self.move_distance
             if ratio > 1.0: ratio = 1.0
             self.start_pos = self.start_pos + self.move_vector * ratio
        else:
             self.start_pos = np.array(self.target_pos) 
             
        # 2. Set new Target
        self.target_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        
        # 3. Calculate Vector
        self.move_vector = self.target_pos - self.start_pos
        self.move_distance = np.linalg.norm(self.move_vector)
        
        # --- Teleop Optimization ---
        # If the move is small (e.g. < 10cm) and we have continuous updates, 
        # assume it's a stream of updates from Joystick.
        # Bypass profile to avoid "stop-start" lag at 50Hz.
        if self.move_distance < 0.1: 
             # Instant Move
             self.start_pos = self.target_pos
             self.profile.current_pos = 0.0
             self.profile.target_pos = 0.0
             self.is_moving = False # Don't run profile loop
             
             # Publish immediately
             self.solve_and_publish(self.target_pos[0], self.target_pos[1], self.target_pos[2])
             
             # Signal Done (so GCode doesn't hang on small moves)
             self.done_pub.publish(Bool(data=True))
             return

        # 4. Reset/Setup Profile (For Standard Moves)
        self.profile.current_pos = 0.0
        self.profile.set_target(self.move_distance)
        self.profile.current_vel = 0.0 
        
        self.is_moving = True
        self.get_logger().info(f"New Move: {self.move_distance:.4f}m")

    def control_loop(self):
        if not self.is_moving:
            return

        # 1. Update Distance Profile
        s, done = self.profile.update(self.dt)
        
        # 2. Calculate Cartesian Position (Linear Interpolation)
        if self.move_distance > 0.000001:
            ratio = s / self.move_distance
            # Clamp ratio
            if ratio > 1.0: ratio = 1.0
            
            p = self.start_pos + self.move_vector * ratio
            x, y, z = p[0], p[1], p[2]
        else:
            # Zero move
            x, y, z = self.target_pos
            done = True

        self.solve_and_publish(x, y, z)
        
        if done:
            self.is_moving = False
            self.done_pub.publish(Bool(data=True)) # Signal GCode to proceed
            self.get_logger().info("Target Reached")
            # Ensure final pos is exact
            self.start_pos = self.target_pos 
        
        # DEBUG
        # self.get_logger().info(f"Prog: {s:.3f}/{self.move_distance:.3f}", throttle_duration_sec=0.5)

    def solve_and_publish(self, x=None, y=None, z=None):
        if x is None: 
            # Use current state
            x, y, z = self.start_pos
            if self.is_moving:
                 # Should not happen if called correctly
                 pass

        try:
            # 3DOF Inverse Kinematics
            # Wrist frame is just X,Y,Z translation (Identity rotation)
            wrist_pos = np.array([x, y, z])
            wrist_frame = Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[wrist_pos[0]], [wrist_pos[1]], [wrist_pos[2]]]))
            
            joint_angles = self.robot.inverse(wrist_frame).flatten()
            t1, t2, t3 = joint_angles
            
            # Publish
            msg = JointTrajectory()
            msg.joint_names = ['jbf1', 'jbf2', 'jbf3', 'Bevelj1', 'Bevelj2', 'Tj1', 'BeveljEE']
            point = JointTrajectoryPoint()
            
            # Send computed angles for first 3 joints, zeros for others
            point.positions = [float(t1), float(t2), float(t3), 0.0, 0.0, 0.0, 0.0]
            point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9)) 
            msg.points.append(point)
            
            self.joint_pub.publish(msg)
            
        except Exception as e:
            # Workspace error
            pass

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SmoothDeltaController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()