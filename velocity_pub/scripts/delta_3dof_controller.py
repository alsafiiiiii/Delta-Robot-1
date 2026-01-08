#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame
import math
from varspeed import Vspeed  # Import user-provided reference library

def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z

class SmoothDeltaController(Node):
    def __init__(self):
        super().__init__('smooth_delta_controller')
        
        # --- CONFIGURATION ---
        self.loop_rate = 50.0   # [FIX] Increased to 50Hz to catch small G-code segments
        self.dt = 1.0 / self.loop_rate
        self.linear_speed = 0.05  
        self.angular_speed = 1.0  
        
        # --- SAFETY SETTINGS ---
        self.min_z_limit = -0.28 

        # Geometry
        self.robot = RobotDelta(np.array([0.104, 0.040, 0.105, 0.205]))
        
        # State
        self.current_pos = np.array([0.0, 0.0, -0.22]) 
        self.start_pos = np.array([0.0, 0.0, -0.22]) # Start of current move
        self.target_pos = np.array([0.0, 0.0, -0.22]) # End of current move
        
        self.current_tilt = 0.0
        self.current_spin = 0.0
        self.target_tilt = 0.0
        self.target_spin = 0.0
        self.is_moving = False

        # --- MOTION GENERATOR ---
        # Vspeed will generate a scalar 0.0 -> 1.0 over time
        self.motion_gen = Vspeed(init_position=0.0, result="float")

        self.joint_pub = self.create_publisher(JointTrajectory, '/model/delta_robot/joint_trajectory', 10)
        self.pose_sub = self.create_subscription(Pose, '/delta/target_pose', self.new_target_callback, 10)
        self.speed_sub = self.create_subscription(Twist, '/delta/speed_params', self.speed_callback, 10)
        
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Smooth Controller Started (Velocity-Based S-Curve @ 50Hz).')

    def new_target_callback(self, msg):
        # Apply Safety Limit to Z
        safe_z = max(msg.position.z, self.min_z_limit)
        
        new_target = np.array([msg.position.x, msg.position.y, safe_z])
        
        # Calculate Distance
        dist = np.linalg.norm(new_target - self.current_pos)
        
        if dist < 0.0001: 
            return # Ignore tiny moves
            
        # VELOCITY-BASED TIMING: T = D / V
        # [FIX] Removed 0.1s clamp. Now allows moves as fast as 1 tick (0.02s).
        duration = dist / self.linear_speed
        
        # Ensure at least 1 tick
        if duration < self.dt: 
            duration = self.dt

        steps = int(duration * self.loop_rate)
        if steps < 1: steps = 1

        # Setup the Move
        self.start_pos = np.copy(self.current_pos)
        self.target_pos = new_target
        
        # Reset Vspeed to go from 0.0 to 1.0
        self.motion_gen.set_position(0.0) 
        
        # [CRITICAL FIX] Force Vspeed to re-initialize!
        # Since new_position is always 1.0, Vspeed thinks we are continuing the old move.
        # We must manually flag it as "Not Started" to trigger the reset of steps/timers.
        self.motion_gen.started = False 

        # [FIX] Smart Easing Selection
        # If already moving (Streaming/Blending), use Linear to maintain momentum.
        # If starting from stop, use Quad for smooth acceleration.
        easing_type = "LinearInOut" if self.is_moving else "QuadEaseInOut"
        
        # PERSIST ARGUMENTS
        self.current_move_args = {
            'new_position': 1.0, 
            'time_secs': duration, 
            'steps': steps, 
            'easing': easing_type
        }
        
        self.motion_gen.move(**self.current_move_args)
        
        self.is_moving = True
        # self.get_logger().info(f"Move: {dist*1000:.1f}mm in {duration:.3f}s ({easing_type})")

    def speed_callback(self, msg):
        self.linear_speed = msg.linear.x
        self.angular_speed = msg.angular.z

    def control_loop(self):
        if not self.is_moving:
            self.solve_and_publish()
            return
        
        # Get next scalar (0.0 -> 1.0)
        # [FIX] Must pass the SAME arguments to continue the move!
        alpha, running, changed = self.motion_gen.move(**self.current_move_args)
        
        # INTERPOLATE
        self.current_pos = self.start_pos + (self.target_pos - self.start_pos) * alpha

        if not running:
            self.is_moving = False
            self.current_pos = self.target_pos # Ensure exact finish

        # (Orientation ignored for now as per previous logic)
        self.solve_and_publish()

    def solve_and_publish(self):
        try:
            # DIRECT MAPPING: Target = Wrist Position
            wrist_pos = self.current_pos

            wrist_frame = Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[wrist_pos[0]], [wrist_pos[1]], [wrist_pos[2]]]))
            
            joint_angles = self.robot.inverse(wrist_frame).flatten()
            t1, t2, t3 = joint_angles
            b1 = self.current_tilt + 2 * self.current_spin
            b2 = 2 * self.current_spin - self.current_tilt
            
            msg = JointTrajectory()
            msg.joint_names = ['jbf1', 'jbf2', 'jbf3', 'Bevelj1', 'Bevelj2', 'Tj1', 'BeveljEE']
            point = JointTrajectoryPoint()
            point.positions = [float(t1), float(t2), float(t3), float(b1), float(b2), float(self.current_tilt), float(self.current_spin)]
            point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9)) 
            msg.points.append(point)
            self.joint_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"IK Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SmoothDeltaController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()