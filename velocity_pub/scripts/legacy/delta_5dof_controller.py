#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
import numpy as np
import math
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame

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
    
    return roll_x, pitch_y, yaw_z # Roll=Tilt, Yaw=Spin (approx for this robot)

class TrapezoidalProfile:
    def __init__(self, max_vel=0.5, accel=1.0, decel=1.0):
        self.max_vel = max_vel
        self.accel = accel
        self.decel = decel
        self.current_pos = 0.0
        self.current_vel = 0.0
        self.target_pos = 0.0
        self.threshold = 0.0001
        
    def set_pid(self, max_vel, accel):
        self.max_vel = max_vel
        self.accel = accel
        self.decel = accel # Symmetric
        
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
            
        # Update Position
        self.current_pos += self.current_vel * dt
        
        return self.current_pos, False # Not Done

class SmoothDeltaController(Node):
    def __init__(self):
        super().__init__('smooth_delta_controller')
        
        # --- Config ---
        self.loop_rate = 50.0  # Hz (Matches ROS standard, Bridge handles interpolation if needed)
        self.dt = 1.0 / self.loop_rate
        
        # Default Kinematics
        self.robot = RobotDelta(np.array([0.104, 0.040, 0.105, 0.205])) 
        self.tool_offset = 0.033
        
        # Motion Profiles per Axis (or Combined)
        # We treat XYZ as a coupled vector for speed, but compute individually here for simplicity
        # Ideally: Interpolate along the 3D Line, not independent axes.
        self.traj_x = TrapezoidalProfile(0.1, 0.5)
        self.traj_y = TrapezoidalProfile(0.1, 0.5)
        self.traj_z = TrapezoidalProfile(0.1, 0.5)
        self.traj_tilt = TrapezoidalProfile(2.0, 5.0) # Rad/s
        self.traj_spin = TrapezoidalProfile(2.0, 5.0)
        
        # State
        self.is_moving = False
        
        # Pub/Sub
        self.joint_pub = self.create_publisher(JointTrajectory, '/model/delta_robot/joint_trajectory', 10)
        self.done_pub = self.create_publisher(Bool, '/delta/movement_done', 10)
        
        self.pose_sub = self.create_subscription(Pose, '/delta/target_pose', self.new_target_callback, 10)
        self.speed_sub = self.create_subscription(Twist, '/delta/speed_params', self.speed_callback, 10)
        
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # Initial Homing
        self.set_home(-0.25)
        
        self.get_logger().info('Smooth Delta Controller Ready (Trapezoidal Mode)')

    def set_home(self, z_height):
        self.traj_x.current_pos = 0.0
        self.traj_x.target_pos = 0.0
        self.traj_y.current_pos = 0.0
        self.traj_y.target_pos = 0.0
        self.traj_z.current_pos = z_height
        self.traj_z.target_pos = z_height
        self.solve_and_publish()

    def speed_callback(self, msg):
        # Allow dynamic speed updates from GCode
        v_lin = max(0.001, msg.linear.x)
        acc_lin = 0.5 # Default fixed accel for now, could be dynamic
        
        self.traj_x.set_pid(v_lin, acc_lin)
        self.traj_y.set_pid(v_lin, acc_lin)
        self.traj_z.set_pid(v_lin, acc_lin)
        
        self.get_logger().info(f"Speed updated: {v_lin:.3f} m/s")

    def new_target_callback(self, msg):
        self.traj_x.set_target(msg.position.x)
        self.traj_y.set_target(msg.position.y)
        self.traj_z.set_target(msg.position.z)
        
        r, p, y = quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.traj_tilt.set_target(r)
        self.traj_spin.set_target(y)
        
        self.is_moving = True
        # self.get_logger().info("New Target Received")

    def control_loop(self):
        if not self.is_moving:
            # Still publish to keep system alive/holding position
            # self.solve_and_publish() 
            return

        # Update all axes
        x, done_x = self.traj_x.update(self.dt)
        y, done_y = self.traj_y.update(self.dt)
        z, done_z = self.traj_z.update(self.dt)
        t, done_t = self.traj_tilt.update(self.dt)
        s, done_s = self.traj_spin.update(self.dt)
        
        self.solve_and_publish(x, y, z, t, s)
        
        if done_x and done_y and done_z and done_t and done_s:
            self.is_moving = False
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info("Target Reached")

    def solve_and_publish(self, x=None, y=None, z=None, tilt=None, spin=None):
        # If args missing, use current profile state
        if x is None: x = self.traj_x.current_pos
        if y is None: y = self.traj_y.current_pos
        if z is None: z = self.traj_z.current_pos
        if tilt is None: tilt = self.traj_tilt.current_pos
        if spin is None: spin = self.traj_spin.current_pos

        try:
            # 1. Delta IK
            # Offset wrist based on tilt? (Simplified model here)
            # In simple delta, tool_offset is just handled by shifting Z or circle intersections
            # For 5DOF, we assume wrist is at (x,y,z) and end-effector is offset by tool length + tilt
            
            # Note: The visual_kinematics library assumes inputs are meant for the wrist frame center
            # effectively.
            
            offset_vec = np.array([0.0, self.tool_offset * np.sin(tilt), -self.tool_offset * np.cos(tilt)])
            wrist_pos = np.array([x, y, z]) - offset_vec
            
            wrist_frame = Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[wrist_pos[0]], [wrist_pos[1]], [wrist_pos[2]]]))
            
            joint_angles = self.robot.inverse(wrist_frame).flatten()
            t1, t2, t3 = joint_angles
            
            # 2. Wrist Aux Joints
            # Assuming simple differential bevel gear or similar mechanism
            b1 = tilt + 2 * spin
            b2 = 2 * spin - tilt
            
            # Publish
            msg = JointTrajectory()
            msg.joint_names = ['jbf1', 'jbf2', 'jbf3', 'Bevelj1', 'Bevelj2', 'Tj1', 'BeveljEE']
            point = JointTrajectoryPoint()
            # We send 7 values to match simulation, Bridge picks first 5
            point.positions = [float(t1), float(t2), float(t3), float(b1), float(b2), float(tilt), float(spin)]
            point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9)) 
            msg.points.append(point)
            
            self.joint_pub.publish(msg)
            
        except Exception as e:
            # Kinematic error (out of workspace)
            # self.get_logger().warn(f"IK Error: {e}")
            pass

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SmoothDeltaController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()