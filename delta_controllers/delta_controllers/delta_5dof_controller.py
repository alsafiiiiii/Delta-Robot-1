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
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
import math
from delta_kinematics.delta_ik import DeltaIK

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
        self.get_logger().info('5-DOF Controller v1.0 (L-R-K-V)')
        
        # --- CONTROL PARAMETERS ---
        self.loop_rate = 100.0  # Hz 
        self.dt = 1.0 / self.loop_rate
        self.linear_speed = 100.0  # m/s
        self.angular_speed = 100.0  # rad/s
        
        # --- GEOMETRY ---
        self.tool_offset = 0.0  
        self.object_offset = 0.0 
        
        # IK solver
        self.ik = DeltaIK()
        
        # --- STATE ---
        self.current_pos = [0.0, 0.0, -0.15]
        self.current_tilt = 0.0
        self.current_spin = 0.0
        
        self.target_pos = [0.0, 0.0, -0.15]
        self.target_tilt = 0.0
        self.target_spin = 0.0
        
        self.is_moving = False
        
        # --- ROS PARAMETERS (Offsets & Compensation) ---
        self.declare_parameter('ee_to_tilt_axis_offset_m', -0.057625)
        self.declare_parameter('tilt_axis_to_tool_tip_offset_m', -0.028385)
        self.declare_parameter('tool_tip_to_object_center_offset_m', -0.0200)
        
        # Added the boolean flag from your YAML config
        self.declare_parameter('enable_tilt_axis_compensation', True)

        # --- ROS INTERFACE ---
        self.joint_pub_real = self.create_publisher(JointTrajectory, '/delta/joint_commands', 10)
        self.joint_pub_sim = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        self.pose_sub = self.create_subscription(Pose, '/delta/target_pose', self.target_callback, 10)
        self.cart_sub = self.create_subscription(JointTrajectory, '/delta/cartesian_trajectory', self.trajectory_callback, 10)
        self.speed_sub = self.create_subscription(Twist, '/delta/speed_params', self.speed_callback, 10)
        self.offset_sub = self.create_subscription(Float64MultiArray, '/delta/offsets', self.offset_callback, 10)
        
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('5DOF TCP Controller Started @ 100Hz (Tilt Compensation Ready)')

    def target_callback(self, msg):
        self.target_pos = [msg.position.x, msg.position.y, msg.position.z]
        roll, pitch, yaw = quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.target_tilt = roll
        self.target_spin = yaw
        self.is_moving = True

    def trajectory_callback(self, msg):
        if not msg.points: return
        point = msg.points[-1]
        if len(point.positions) >= 3:
            self.target_pos = list(point.positions[:3])
        if len(point.positions) >= 5:
            self.target_tilt = point.positions[3]
            self.target_spin = point.positions[4]
        self.is_moving = True

    def speed_callback(self, msg):
        if msg.linear.x > 0: self.linear_speed = msg.linear.x
        if msg.angular.z > 0: self.angular_speed = msg.angular.z

    def offset_callback(self, msg):
        if len(msg.data) >= 2:
            self.tool_offset = msg.data[0]
            self.object_offset = msg.data[1]

    def control_loop(self):
        if self.is_moving:
            dx, dy, dz = self.target_pos[0] - self.current_pos[0], self.target_pos[1] - self.current_pos[1], self.target_pos[2] - self.current_pos[2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            step_dist, step_ang = self.linear_speed * self.dt, self.angular_speed * self.dt
            
            if dist > step_dist:
                scale = step_dist / dist
                self.current_pos[0] += dx * scale
                self.current_pos[1] += dy * scale
                self.current_pos[2] += dz * scale
            else:
                self.current_pos = list(self.target_pos)
            
            tilt_err, spin_err = self.target_tilt - self.current_tilt, self.target_spin - self.current_spin
            
            if abs(tilt_err) > step_ang: self.current_tilt += math.copysign(step_ang, tilt_err)
            else: self.current_tilt = self.target_tilt
            
            if abs(spin_err) > step_ang: self.current_spin += math.copysign(step_ang, spin_err)
            else: self.current_spin = self.target_spin
            
            if dist <= step_dist and abs(tilt_err) <= step_ang and abs(spin_err) <= step_ang:
                self.is_moving = False
        
        self.solve_and_publish()

    def solve_and_publish(self):
        try:
            # 1. Fetch live parameters
            tilt_comp = self.get_parameter('enable_tilt_axis_compensation').value
            
            # The static vertical drop from the end-effector wrist to the bevel gear joint
            static_drop = self.get_parameter('ee_to_tilt_axis_offset_m').value
            
            # The length of the tool that actively swings when tilted.
            # (Note: YAML params are negative. GUI offsets are subtracted to make the tool longer/more negative)
            rotating_length = (
                self.get_parameter('tilt_axis_to_tool_tip_offset_m').value +
                self.get_parameter('tool_tip_to_object_center_offset_m').value -
                self.tool_offset - 
                self.object_offset
            )

            # 2. Calculate Required Wrist Position
            wrist_x = self.current_pos[0]
            
            if tilt_comp:
                # To keep the TCP exactly at (X,Y,Z), the wrist must physically move backwards/upwards
                # to counter-act the arc made by the swinging tool length.
                wrist_y = self.current_pos[1] + (rotating_length * math.sin(self.current_tilt))
                wrist_z = self.current_pos[2] - static_drop - (rotating_length * math.cos(self.current_tilt))
            else:
                # Standard behavior: The wrist sits directly above the (X,Y) target.
                # If you tilt, the object will physically swing away from the target coordinates.
                wrist_y = self.current_pos[1]
                wrist_z = self.current_pos[2] - static_drop - rotating_length

            # 3. Solve IK for the newly calculated wrist coordinates
            t1, t2, t3 = self.ik.inverse(wrist_x, wrist_y, wrist_z)

            # 4. Bevel gear kinematics (for tilt and spin)
            b1 = self.current_tilt + 2 * self.current_spin
            b2 = 2 * self.current_spin - self.current_tilt

            # 5. Build and Publish Message
            msg = JointTrajectory()
            msg.joint_names = [
                'motor_joint_1', 'motor_joint_2', 'motor_joint_3',
                'differential_pinion_joint_1', 'differential_pinion_joint_2',
                'differential_T_joint', 'differential_EE_joint'
            ]
            point = JointTrajectoryPoint()
            point.positions = [float(t1), float(t2), float(t3), float(b1), float(b2), float(self.current_tilt), float(self.current_spin)]
            point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))
            msg.points.append(point)

            self.joint_pub_real.publish(msg)
            self.joint_pub_sim.publish(msg)

        except ValueError as e:
            pass # Keep terminal clean, or use self.get_logger().warn(f"IK Error: {e}") if debugging

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