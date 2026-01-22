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
from quintic_trajectory import QuinticGenerator

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
        self.loop_rate = 100.0  # Increased to 100Hz for smoother trajectory sampling
        self.dt = 1.0 / self.loop_rate
        self.linear_speed = 0.05  
        self.angular_speed = 1.0  
        
        # --- SAFETY SETTINGS ---

        # Geometry
        self.robot = RobotDelta(np.array([0.104, 0.040, 0.105, 0.205]))
        
        # State
        self.current_pos = np.array([0.0, 0.0, -0.22]) 
        self.current_tilt = 0.0
        self.current_spin = 0.0
        self.target_pos = np.array([0.0, 0.0, -0.22])
        self.target_tilt = 0.0
        self.target_spin = 0.0
        self.target_spin = 0.0
        self.is_moving = False
        
        # Trajectory Generator
        self.traj_generator = None

        self.joint_pub = self.create_publisher(JointTrajectory, '/model/delta_robot/joint_trajectory', 10)
        self.pose_sub = self.create_subscription(Pose, '/delta/target_pose', self.new_target_callback, 10)
        self.speed_sub = self.create_subscription(Twist, '/delta/speed_params', self.speed_callback, 10)
        
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Smooth Controller Started (Wrist Only Mode).')

    def new_target_callback(self, msg):
        
        # The 3DOF controller does not have tool offset compensation logic.
        self.target_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        
        # Enforce vertical tool orientation
        self.target_tilt = 0.0
        self.target_spin = 0.0
        # r, p, y = quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # self.target_tilt = r
        # self.target_spin = y
        # Start new trajectory from current actual position to target
        self.traj_generator = QuinticGenerator(
            start_pos=self.current_pos,
            end_pos=self.target_pos,
            average_speed=self.linear_speed
        )
        self.is_moving = True

    def speed_callback(self, msg):
        self.linear_speed = msg.linear.x
        self.angular_speed = msg.angular.z

    def control_loop(self):
        if self.traj_generator:
            # Get next point in smooth trajectory
            pos, vel, acc = self.traj_generator.get_state(self.dt)
            self.current_pos = pos
            
            if self.traj_generator.finished:
                self.traj_generator = None
                self.is_moving = False
            
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