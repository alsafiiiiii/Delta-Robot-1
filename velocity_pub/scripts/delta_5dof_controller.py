#!/usr/bin/env python3
"""
Delta Robot 5-DOF Controller
Combines Delta Inverse Kinematics with Differential Wrist Control.
Compensates for tool offset to maintain tool tip position during rotation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame
import math

def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
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

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Calculates Rotation Matrix given euler angles.
    R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    cx = np.cos(roll)
    sx = np.sin(roll)
    cy = np.cos(pitch)
    sy = np.sin(pitch)
    cz = np.cos(yaw)
    sz = np.sin(yaw)

    R = np.array([
        [cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
        [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
        [-sy,   cy*sx,            cy*cx]
    ])
    return R

class Delta5DOFController(Node):
    def __init__(self):
        super().__init__('delta_5dof_controller')
        
        # --- Robot Parameters ---
        # Delta Geometry
        self.r_base = 0.07582127019  # r1
        self.r_ee = 0.035            # r2
        self.l1 = 0.075              # upper arm
        self.l2 = 0.2639602098       # forearm
        
        # Tool Offset (Distance from Wrist Center/EEBase to Tool Tip)
        # Based on SDF: EEBase z = -0.237, EE z = -0.270 -> diff = 0.033
        self.tool_offset = 0.033 
        
        # Initialize Delta Robot Kinematics
        self.robot = RobotDelta(np.array([self.r_base, self.r_ee, self.l1, self.l2]))
        
        # --- ROS Interfaces ---
        # Publisher for all joints
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/model/delta_robot/joint_trajectory',
            10
        )
        
        # Subscriber for target pose
        self.pose_sub = self.create_subscription(
            Pose,
            '/delta/target_pose',
            self.pose_callback,
            10
        )
        
        self.get_logger().info('Delta 5-DOF Controller initialized')
        self.get_logger().info(f'Tool Offset: {self.tool_offset}m')
        self.get_logger().info('Listening on /delta/target_pose')

    def pose_callback(self, msg):
        try:
            # 1. Extract Target Position and Orientation
            target_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
            
            # Convert Quaternion to Euler (Roll, Pitch, Yaw)
            # Note: The mechanism controls Pitch (T-link, around X) and Roll (EE, around Z local)
            # We map the standard Euler angles to our mechanism.
            # Assuming standard XYZ convention: Roll=X, Pitch=Y, Yaw=Z.
            # But our mechanism's "Pitch" is around X axis of the T-link?
            # Let's look at the SDF: Tj1 axis is 1 0 0 (X). So Tj1 controls rotation around X.
            # BeveljEE axis is 0 0 1 (Z). So BeveljEE controls rotation around Z.
            # So we map:
            #   Mechanism Pitch (Tj1) <-> Euler Roll (X-axis rotation)
            #   Mechanism Roll (BeveljEE) <-> Euler Yaw (Z-axis rotation)?
            #   Wait, usually "Roll" is around the tool axis.
            #   If the tool points down (-Z), rotation around Z is indeed "Roll" of the tool.
            #   Rotation around X is "Pitch" (tilting forward/back).
            #   Rotation around Y is "Yaw" (tilting left/right)?
            #   The differential mechanism usually allows 2 degrees of freedom.
            #   Tj1 (X-axis) allows tilting in one plane.
            #   BeveljEE (Z-axis) allows spinning the tool.
            #   This means we can tilt in X-axis and spin. We CANNOT tilt in Y-axis?
            #   If so, this is a 4-DOF robot (X,Y,Z, TiltX) + Spin?
            #   Or maybe the whole wrist can rotate? No, the wrist is mounted on the parallel arms.
            #   The platform (EEBase) always stays parallel to the ground (3-DOF Delta property).
            #   So the wrist base is fixed in orientation (flat).
            #   Then Tj1 rotates around X (global X, since base is flat).
            #   Then EE rotates around Z (local Z of T-link).
            #   So we can only tilt around X and spin around Z.
            #   We cannot tilt around Y.
            #   So if the user asks for Pitch (Y-rotation), we can't do it?
            #   Or maybe I am misinterpreting "Pitch" and "Roll" names in the user request.
            #   User said: "do the rotation and offset... 3dof delta robot into 5dof".
            #   Usually 5DOF implies X,Y,Z + 2 rotations.
            #   If the mechanism is X-axis tilt + Z-axis spin, that is 2 rotations.
            #   So I will map:
            #     Target Euler X (Roll) -> Tj1 (Tilt)
            #     Target Euler Z (Yaw) -> BeveljEE (Spin)
            #     Target Euler Y (Pitch) -> Ignored/Impossible?
            
            # Let's assume the user wants to control the available DOFs.
            # I will extract Roll (X) and Yaw (Z) from the target quaternion.
            # But wait, "Pitch" in `ee_controller` context might be the T-link angle.
            # Let's stick to the axis definitions:
            # Tj1: X-axis.
            # BeveljEE: Z-axis.
            
            roll, pitch, yaw = quaternion_to_euler(
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            )
            
            # Mapping to mechanism variables
            # theta_tilt (Tj1) = roll (rotation about X)
            # theta_spin (BeveljEE) = yaw (rotation about Z)
            # Note: This assumes the target orientation is composed of X-rot then Z-rot.
            
            theta_tilt = roll
            theta_spin = yaw # Or pitch? Let's assume Yaw (Z-rot) is the spin.
            
            # However, usually "Pitch" and "Roll" refer to the two tilt axes in a 5-DOF wrist (like a universal joint).
            # But here we have a specific differential gear.
            # Let's look at `ee_controller.py` again.
            # `target_angle_ee = 0.25 * (b1 + b2)` -> Output 1
            # `target_angle_tlink = 0.5 * (b1 - b2)` -> Output 2
            # If I set b1=1, b2=1 -> ee=0.5, tlink=0. Spin only.
            # If I set b1=1, b2=-1 -> ee=0, tlink=1. Tilt only.
            
            # So we have independent control of Tilt (X) and Spin (Z).
            # I will use the extracted Euler Roll (X) for Tilt, and Euler Yaw (Z) for Spin.
            # (Assuming the user sends a quaternion that respects this constraint).
            
            # 2. Calculate Wrist Center Position (Inverse Displacement)
            # We want the Tool Tip to be at `target_pos`.
            # The Tool Tip is offset from the Wrist Center (EEBase) by `tool_offset` along the tool Z-axis.
            # Vector from Wrist to Tip in Wrist Frame: V_local = [0, 0, -tool_offset] (pointing down)
            # Actually, EEBase is at -0.237, EE at -0.270. So EE is below EEBase.
            # So in the T-link frame (which tilts), the EE is along the -Z axis?
            # SDF: T pose 0 0 -0.26. EE pose 0 0 -0.27 relative to world? No, absolute.
            # Relative to T?
            # BeveljEE connects T to EE.
            # So EE is attached to T.
            # If T tilts, EE tilts.
            # So the vector is fixed in T frame (or EE frame).
            # Let's assume the vector from Wrist Center to Tool Tip is rotated by the Tilt (Tj1).
            # The Spin (BeveljEE) doesn't change the position of the tip if it's on the axis.
            # So we only care about Tilt (theta_tilt).
            
            # Rotation Matrix for Tilt (around X):
            # [1  0        0     ]
            # [0  cos(t)  -sin(t)]
            # [0  sin(t)   cos(t)]
            
            # Offset vector in un-rotated frame (pointing down): [0, 0, -L]
            # Rotated offset: R_x(theta_tilt) * [0, 0, -L]
            # = [0, -sin(t)*(-L), cos(t)*(-L)]
            # = [0, L*sin(t), -L*cos(t)]
            
            # So WristPos = TargetPos - RotatedOffset
            # WristPos = TargetPos - [0, L*sin(t), -L*cos(t)]
            
            L = self.tool_offset
            t = theta_tilt
            
            offset_vec = np.array([0.0, L * np.sin(t), -L * np.cos(t)])
            
            # Wait, if I want the tip at TargetPos, and the tip is at Wrist + Offset.
            # TargetPos = Wrist + Offset
            # Wrist = TargetPos - Offset
            
            wrist_pos = target_pos - offset_vec
            
            # 3. Compute Delta Inverse Kinematics for Wrist Position
            # Create a frame for the wrist center (orientation doesn't matter for Delta IK, only pos)
            wrist_frame = Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[wrist_pos[0]], [wrist_pos[1]], [wrist_pos[2]]]))
            
            joint_angles_rad = self.robot.inverse(wrist_frame)
            theta1, theta2, theta3 = joint_angles_rad.flatten()
            
            # 4. Compute Differential Inverse Kinematics
            # We have theta_tilt (t) and theta_spin (s).
            # t = 0.5 * (b1 - b2)
            # s = 0.25 * (b1 + b2)
            # Solve for b1, b2:
            # 2t = b1 - b2
            # 4s = b1 + b2
            # Adding: 2t + 4s = 2*b1 -> b1 = t + 2s
            # Subtr: 4s - 2t = 2*b2 -> b2 = 2s - t
            
            b1 = theta_tilt + 2 * theta_spin
            b2 = 2 * theta_spin - theta_tilt
            
            # 5. Compute Passive Joint Angles (for simulation visualization)
            # Tj1 = theta_tilt
            # BeveljEE = theta_spin
            tj1_val = theta_tilt
            bevel_ee_val = theta_spin
            
            # 6. Publish Trajectory
            self.publish_trajectory(theta1, theta2, theta3, b1, b2, tj1_val, bevel_ee_val)
            
            self.get_logger().info(f'Moved to: {target_pos}, Tilt: {theta_tilt:.2f}, Spin: {theta_spin:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Control failed: {str(e)}')

    def publish_trajectory(self, t1, t2, t3, b1, b2, tj1, bee):
        msg = JointTrajectory()
        msg.joint_names = [
            'jbf1', 'jbf2', 'jbf3',  # Delta Arms
            'Bevelj1', 'Bevelj2',    # Wrist Motors
            'Tj1', 'BeveljEE'        # Passive Joints (Simulated)
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [
            float(t1), float(t2), float(t3),
            float(b1), float(b2),
            float(tj1), float(bee)
        ]
        point.time_from_start = Duration(sec=0, nanosec=500000000) # 0.5s duration
        
        msg.points.append(point)
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Delta5DOFController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
