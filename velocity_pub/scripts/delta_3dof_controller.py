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
        
        # --- SAFETY SETTINGS ---
        # Geometry
        self.robot = RobotDelta(np.array([0.104, 0.040, 0.105, 0.205]))
        
        # Publishers / Subscribers
        self.joint_pub = self.create_publisher(JointTrajectory, '/model/delta_robot/joint_trajectory', 10)
        
        # UPDATED: Subscribe to Cartesian Trajectory (Target + Duration)
        self.cart_sub = self.create_subscription(
            JointTrajectory, 
            '/delta/cartesian_trajectory', 
            self.trajectory_callback, 
            10
        )
        
        self.get_logger().info('Smooth Controller Started (Event-Driven Mode).')

    def trajectory_callback(self, msg):
        if not msg.points: return
        
        # 1. Extract Target (Cartesian)
        point = msg.points[0]
        x, y, z = point.positions[0:3]
        duration = point.time_from_start
        
        try:
            # 2. Compute IK (Joint Angles)
            wrist_frame = Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[x], [y], [z]]))
            joint_angles = self.robot.inverse(wrist_frame).flatten()
            t1, t2, t3 = joint_angles
            
            # Simple assumption for wrist orientation (Face down)
            tilt = 0.0
            spin = 0.0
            b1 = tilt + 2 * spin
            b2 = 2 * spin - tilt
            
            # 3. Construct Output Message (Joints + Duration)
            out_msg = JointTrajectory()
            out_msg.joint_names = ['jbf1', 'jbf2', 'jbf3', 'Bevelj1', 'Bevelj2', 'Tj1', 'BeveljEE']
            
            out_point = JointTrajectoryPoint()
            out_point.positions = [float(t1), float(t2), float(t3), float(b1), float(b2), float(tilt), float(spin)]
            out_point.time_from_start = duration  # Pass-through duration
            
            out_msg.points.append(out_point)
            
            # 4. Forward immediately
            self.joint_pub.publish(out_msg)
            
            sec = duration.sec + duration.nanosec * 1e-9
            self.get_logger().info(f"Forwarding Move: [{t1:.2f}, {t2:.2f}, {t3:.2f}] T={sec:.3f}s")
            
        except Exception as e:
            self.get_logger().warn(f"IK Error for ({x},{y},{z}): {e}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SmoothDeltaController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()