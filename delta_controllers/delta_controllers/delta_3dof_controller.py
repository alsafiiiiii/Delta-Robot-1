#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
from delta_kinematics.delta_ik import DeltaIK

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
        # Geometry (using pure Python IK)
        self.ik = DeltaIK()
        
        # Direct mode response time (instant)
        self.DIRECT_DURATION_MS = 20  # 20ms for immediate response
        
        # --- CONFIGURATION ---
        self.declare_parameter('use_sim', False)
        self.use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value
        
        # Topic selection based on mode
        if self.use_sim:
            # ROS 2 Control (Simulation)
            topic = '/joint_trajectory_controller/joint_trajectory'
            self.get_logger().info(f"Mode: SIMULATION. Publishing to {topic}")
        else:
            # Custom Serial Bridge (Real Robot)
            topic = '/delta/joint_commands'
            self.get_logger().info(f"Mode: REAL ROBOT. Publishing to {topic}")

        # Publishers / Subscribers
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.joint_pub = self.create_publisher(JointTrajectory, topic, qos_profile)
        
        # Time-encoded trajectory (for G-code)
        self.cart_sub = self.create_subscription(
            JointTrajectory, 
            '/delta/cartesian_trajectory', 
            self.trajectory_callback, 
            10
        )
        
        # DIRECT MODE: Subscribe to target_pose for GUI/joystick (bypasses time encoding)
        self.pose_sub = self.create_subscription(
            Pose,
            '/delta/target_pose',
            self.direct_pose_callback,
            10
        )
        
        self.get_logger().info('Smooth Controller Started (Dual Mode: Trajectory + Direct).')

    def direct_pose_callback(self, msg):
        """Direct mode: Immediately forward pose to joints (no time encoding)."""
        try:
            x = msg.position.x
            y = msg.position.y
            z = msg.position.z
            
            # Extract orientation (for tilt/spin if needed)
            roll, pitch, yaw = quaternion_to_euler(
                msg.orientation.x, msg.orientation.y, 
                msg.orientation.z, msg.orientation.w
            )
            
            # Compute IK
            t1, t2, t3 = self.ik.inverse(x, y, z)
            
            # Orientation mapping
            tilt = roll
            spin = yaw
            b1 = tilt + 2 * spin
            b2 = 2 * spin - tilt
            
            # Create instant trajectory message
            out_msg = JointTrajectory()
            out_msg.joint_names = ['jbf1', 'jbf2', 'jbf3', 'Bevelj1', 'Bevelj2', 'Tj1', 'BeveljEE']
            
            out_point = JointTrajectoryPoint()
            out_point.positions = [float(t1), float(t2), float(t3), float(b1), float(b2), float(tilt), float(spin)]
            out_point.time_from_start = Duration(sec=0, nanosec=self.DIRECT_DURATION_MS * 1_000_000)
            out_msg.points.append(out_point)
            
            self.joint_pub.publish(out_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Direct IK Error: {e}")

    def trajectory_callback(self, msg):
        """Time-encoded mode: Process full trajectory with durations (for G-code)."""
        if not msg.points: return
        
        out_msg = JointTrajectory()
        out_msg.joint_names = ['jbf1', 'jbf2', 'jbf3', 'Bevelj1', 'Bevelj2', 'Tj1', 'BeveljEE']
        
        try:
            for point in msg.points:
                # 1. Extract Cartesian
                x, y, z = point.positions[0:3]
                duration = point.time_from_start
                
                # 2. Compute IK
                t1, t2, t3 = self.ik.inverse(x, y, z)
                
                # Orientation (Face down)
                tilt = 0.0
                spin = 0.0
                b1 = tilt + 2 * spin
                b2 = 2 * spin - tilt
                
                # 3. Append Joint Point
                out_point = JointTrajectoryPoint()
                out_point.positions = [float(t1), float(t2), float(t3), float(b1), float(b2), float(tilt), float(spin)]
                out_point.time_from_start = duration
                out_msg.points.append(out_point)

            # 4. Publish Full Trajectory
            self.joint_pub.publish(out_msg)
            
            # Log summary
            if out_msg.points:
                last_t = out_msg.points[-1].time_from_start
                sec = last_t.sec + last_t.nanosec * 1e-9
                self.get_logger().info(f"Forwarding Path: {len(out_msg.points)} pts, Total T={sec:.3f}s")
                
        except Exception as e:
            self.get_logger().warn(f"IK Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SmoothDeltaController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()