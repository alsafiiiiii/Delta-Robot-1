#!/usr/bin/env python3
"""
Delta 5-DOF Smooth Trajectory Controller
interpolates between points to simulate velocity control on position servos.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame
import math
import time

# --- Message Definition for Custom Command ---
# Since standard Pose doesn't have "Speed", we usually use a custom msg or 
# hack the Pose msg. For simplicity, I will subscribe to a custom topic 
# where we use the standard Pose, but I'll hardcode a default speed variable 
# that you can change dynamically.

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
        
        # --- Configuration ---
        self.loop_rate = 50.0  # Hz (50 times per second)
        self.dt = 1.0 / self.loop_rate
        
        # Speed Settings
        self.linear_speed = 0.05  # m/s (5 cm per second)
        self.angular_speed = 1.0  # rad/s
        
        # Robot Geometry
        self.r_base = 0.0758
        self.r_ee = 0.035
        self.l1 = 0.075
        self.l2 = 0.2639
        self.tool_offset = 0.033
        
        self.robot = RobotDelta(np.array([self.r_base, self.r_ee, self.l1, self.l2]))
        
        # --- State Variables ---
        # Where the robot IS currently (simulated current state)
        self.current_pos = np.array([0.0, 0.0, -0.25]) 
        self.current_tilt = 0.0
        self.current_spin = 0.0
        
        # Where the robot WANTS to go (final destination)
        self.target_pos = np.array([0.0, 0.0, -0.25])
        self.target_tilt = 0.0
        self.target_spin = 0.0
        
        self.is_moving = False

        # --- ROS Setup ---
        self.joint_pub = self.create_publisher(
            JointTrajectory, '/model/delta_robot/joint_trajectory', 10)
        
        # Input: Standard Pose message
        self.pose_sub = self.create_subscription(
            Pose, '/delta/target_pose', self.new_target_callback, 10)
        
        # Input: Speed Parameters
        self.speed_sub = self.create_subscription(
            Twist, '/delta/speed_params', self.speed_callback, 10)
        
        # Timer for the control loop (The Heartbeat)
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info(f'Smooth Controller Started. Freq: {self.loop_rate}Hz, Speed: {self.linear_speed}m/s')

    def new_target_callback(self, msg):
        """Receives a new final destination."""
        # Extract Position
        self.target_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        
        # Extract Orientation
        r, p, y = quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.target_tilt = r
        self.target_spin = y
        
        self.is_moving = True
        self.get_logger().info(f"New Target Received: {self.target_pos}")

    def speed_callback(self, msg):
        """Receives new speed settings."""
        self.linear_speed = msg.linear.x
        self.angular_speed = msg.angular.z
        self.get_logger().info(f"Speed Updated: Linear={self.linear_speed:.3f}, Angular={self.angular_speed:.3f}")

    def control_loop(self):
        """
        Runs at 50Hz.
        Calculates the NEXT TINY STEP to take towards the target.
        """
        if not self.is_moving:
            # Even if not moving, we publish current state to hold position torque
            self.solve_and_publish()
            return

        # 1. Calculate Errors (Distance to target)
        pos_error = self.target_pos - self.current_pos
        dist = np.linalg.norm(pos_error)
        
        tilt_error = self.target_tilt - self.current_tilt
        spin_error = self.target_spin - self.current_spin
        
        # 2. Determine Step Size based on Speed
        # Max step allowed per loop cycle = speed * dt
        step_dist = self.linear_speed * self.dt
        step_ang = self.angular_speed * self.dt
        
        # 3. Interpolate Position
        if dist > step_dist:
            # Move towards target by step_dist
            direction = pos_error / dist
            self.current_pos += direction * step_dist
        else:
            # We are close enough, snap to target
            self.current_pos = np.copy(self.target_pos)
            
        # 4. Interpolate Angles
        # Simple logic: if error positive, add step; if negative, sub step
        if abs(tilt_error) > step_ang:
            self.current_tilt += math.copysign(step_ang, tilt_error)
        else:
            self.current_tilt = self.target_tilt
            
        if abs(spin_error) > step_ang:
            self.current_spin += math.copysign(step_ang, spin_error)
        else:
            self.current_spin = self.target_spin
            
        # Check if we have arrived
        if dist <= step_dist and abs(tilt_error) <= step_ang and abs(spin_error) <= step_ang:
            self.is_moving = False
            
        # 5. Calculate IK and Publish
        self.solve_and_publish()

    def solve_and_publish(self):
        """Calculates IK for current_pos/tilt/spin and publishes"""
        try:
            # Apply Tool Offset Logic (Same as before)
            offset_vec = np.array([
                0.0, 
                self.tool_offset * np.sin(self.current_tilt), 
                -self.tool_offset * np.cos(self.current_tilt)
            ])
            
            # Calculate Wrist Center
            wrist_pos = self.current_pos - offset_vec
            
            # Delta IK
            wrist_frame = Frame.from_euler_3(
                np.array([0., 0., 0.]), 
                np.array([[wrist_pos[0]], [wrist_pos[1]], [wrist_pos[2]]])
            )
            
            joint_angles = self.robot.inverse(wrist_frame).flatten()
            t1, t2, t3 = joint_angles
            
            # Differential Wrist IK
            b1 = self.current_tilt + 2 * self.current_spin
            b2 = 2 * self.current_spin - self.current_tilt
            
            self.publish_joints(t1, t2, t3, b1, b2)
            
        except Exception as e:
            self.get_logger().warn(f"IK Error: {e}")

    def publish_joints(self, t1, t2, t3, b1, b2):
        msg = JointTrajectory()
        msg.joint_names = ['jbf1', 'jbf2', 'jbf3', 'Bevelj1', 'Bevelj2', 'Tj1', 'BeveljEE']
        point = JointTrajectoryPoint()
        # Use Simulated Passive joints for visualization
        point.positions = [float(t1), float(t2), float(t3), float(b1), float(b2), float(self.current_tilt), float(self.current_spin)]
        point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9)) 
        msg.points.append(point)
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SmoothDeltaController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()