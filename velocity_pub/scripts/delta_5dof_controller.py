#!/usr/bin/env python3
"""
5DOF Delta Robot Controller

Handles 5 degrees of freedom:
- 3DOF: X, Y, Z position (Delta parallel linkage)
- 2DOF: Tilt and Spin (End-effector orientation via bevel gears)

Uses pure Python IK (delta_ik.py) and publishes to /delta/joint_commands.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
from delta_ik import DeltaIK

def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to roll, pitch, yaw."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = max(-1.0, min(+1.0, t2))
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z

class SmoothDeltaController5DOF(Node):
    def __init__(self):
        super().__init__('smooth_delta_controller_5dof')
        
        # --- CONTROL PARAMETERS ---
        self.loop_rate = 50.0  # Hz (match sim_control)
        self.dt = 1.0 / self.loop_rate
        self.linear_speed = 0.10  # m/s
        self.angular_speed = 2.0  # rad/s
        
        # --- GEOMETRY ---
        # Tool offset for 5DOF (distance from wrist to end-effector tip)
        self.tool_offset = 0.033  # 33mm
        
        # IK solver with SIMULATION geometry
        # Sim: r_base=0.0758, r_ee=0.035, l1=0.075, l2=0.2639
        # Real: 0.104, 0.040, 0.105, 0.205 (default in delta_ik.py)
        self.ik = DeltaIK()
        
        # --- STATE ---
        self.current_pos = [0.0, 0.0, -0.25]
        self.current_tilt = 0.0
        self.current_spin = 0.0
        
        self.target_pos = [0.0, 0.0, -0.25]
        self.target_tilt = 0.0
        self.target_spin = 0.0
        
        self.is_moving = False
        
        # --- ROS INTERFACE ---
        # Output: joint commands (goes to sim_control or robot_control)
        self.joint_pub = self.create_publisher(
            JointTrajectory, 
            '/delta/joint_commands', 
            10
        )
        
        # Input: target pose (from joystick, GUI, camera_pnp)
        self.pose_sub = self.create_subscription(
            Pose, 
            '/delta/target_pose', 
            self.target_callback, 
            10
        )
        
        # Input: cartesian trajectory (from G-code interpreter)
        self.cart_sub = self.create_subscription(
            JointTrajectory,
            '/delta/cartesian_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Input: speed parameters
        self.speed_sub = self.create_subscription(
            Twist, 
            '/delta/speed_params', 
            self.speed_callback, 
            10
        )
        
        # Control loop timer
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('5DOF Delta Controller Started @ 50Hz')

    def target_callback(self, msg):
        """Direct pose command (realtime control)."""
        self.target_pos = [msg.position.x, msg.position.y, msg.position.z]
        
        # Extract orientation
        roll, pitch, yaw = quaternion_to_euler(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        )
        self.target_tilt = roll  # Tilt around X axis
        self.target_spin = yaw   # Spin around Z axis
        # self.get_logger().info(f"Target Pose: {self.target_pos}, Tilt: {self.target_tilt:.3f}, Spin: {self.target_spin:.3f}")
        if abs(yaw) > 0.1: # Only log if there is significant yaw
             self.get_logger().info(f"Received Yaw: {yaw:.3f}")
        self.is_moving = True

    def trajectory_callback(self, msg):
        """Time-encoded trajectory (from G-code)."""
        if not msg.points:
            return
        
        # For now, just take the last point as target
        # (Full trajectory following could be added later)
        point = msg.points[-1]
        
        if len(point.positions) >= 3:
            self.target_pos = list(point.positions[:3])
        
        # If 5 values provided, use tilt and spin
        if len(point.positions) >= 5:
            self.target_tilt = point.positions[3]
            self.target_spin = point.positions[4]
        
        self.is_moving = True

    def speed_callback(self, msg):
        """Update speed parameters."""
        if msg.linear.x > 0:
            self.linear_speed = msg.linear.x
        if msg.angular.z > 0:
            self.angular_speed = msg.angular.z

    def control_loop(self):
        """Main control loop - interpolate towards target."""
        if self.is_moving:
            # Position interpolation
            dx = self.target_pos[0] - self.current_pos[0]
            dy = self.target_pos[1] - self.current_pos[1]
            dz = self.target_pos[2] - self.current_pos[2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            step_dist = self.linear_speed * self.dt
            step_ang = self.angular_speed * self.dt
            
            if dist > step_dist:
                scale = step_dist / dist
                self.current_pos[0] += dx * scale
                self.current_pos[1] += dy * scale
                self.current_pos[2] += dz * scale
            else:
                self.current_pos = list(self.target_pos)
            
            # Orientation interpolation
            tilt_err = self.target_tilt - self.current_tilt
            spin_err = self.target_spin - self.current_spin
            
            if abs(tilt_err) > step_ang:
                self.current_tilt += math.copysign(step_ang, tilt_err)
            else:
                self.current_tilt = self.target_tilt
            
            if abs(spin_err) > step_ang:
                self.current_spin += math.copysign(step_ang, spin_err)
            else:
                self.current_spin = self.target_spin
            
            # Check if motion complete
            if (dist <= step_dist and 
                abs(tilt_err) <= step_ang and 
                abs(spin_err) <= step_ang):
                self.is_moving = False
        
        # Always publish current state
        self.solve_and_publish()

    def solve_and_publish(self):
        """Compute IK and publish joint commands."""
        try:
            # 5DOF: Compute wrist position accounting for tool offset
            # The tool tip is offset from the wrist by tool_offset in the Z direction
            # When tilted, this offset rotates
            offset_y = self.tool_offset * math.sin(self.current_tilt)
            offset_z = self.tool_offset * math.cos(self.current_tilt)
            
            wrist_x = self.current_pos[0]
            wrist_y = self.current_pos[1] - offset_y
            wrist_z = self.current_pos[2] + offset_z - self.tool_offset  # Subtract base offset
            
            # Solve IK for wrist position
            t1, t2, t3 = self.ik.inverse(wrist_x, wrist_y, wrist_z)
            
            # Bevel gear kinematics (for tilt and spin)
            # b1 and b2 are the two bevel gear joint angles
            b1 = self.current_tilt + 2 * self.current_spin
            b2 = 2 * self.current_spin - self.current_tilt
            
            # Build joint trajectory message
            msg = JointTrajectory()
            msg.joint_names = ['jbf1', 'jbf2', 'jbf3', 'Bevelj1', 'Bevelj2', 'Tj1', 'BeveljEE']
            
            point = JointTrajectoryPoint()
            point.positions = [
                float(t1), float(t2), float(t3),  # Delta arm joints (radians)
                float(b1), float(b2),              # Bevel gear joints
                float(self.current_tilt),          # Tilt joint
                float(self.current_spin)           # Spin joint
            ]
            point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))
            msg.points.append(point)
            
            self.joint_pub.publish(msg)
            
        except ValueError as e:
            self.get_logger().warn(f"IK Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SmoothDeltaController5DOF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()