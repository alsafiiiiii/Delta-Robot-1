#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import math
import sys
import os

# Try to import the library, warn if missing
try:
    from gcodeparser import GcodeParser
except ImportError:
    print("Error: 'gcodeparser' library is missing.")
    print("Please install it using: pip install gcodeparser")
    sys.exit(1)

def get_quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler (rad) to Quaternion (x, y, z, w)"""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class GCodeVirtualMachine(Node):
    def __init__(self, filename):
        super().__init__('gcode_sender')
        # UPDATED: Publish Cartesian Trajectory instead of Pose
        self.traj_pub_ = self.create_publisher(JointTrajectory, '/delta/cartesian_trajectory', 10)
        
        # --- Robot Settings ---
        self.robot_speed = 0.5  # m/s
        
        # --- Virtual Machine State ---
        self.mode = 'ABS'         # 'ABS' (G90) or 'REL' (G91)
        self.unit_scale = 1.0     # Default to Meters (1.0)
        
        # Current Position (Absolute Meters, Radians)
        self.pos = {
            'X': 0.0, 'Y': 0.0, 'Z': -0.22, 'A': 0.0, 'C': 0.0
        }
        
        self.filename = filename
        
        # Wait for ROS connection
        self.get_logger().info("Connecting to ROS...")
        time.sleep(2.0)
        
        self.run_gcode()

    def run_gcode(self):
        try:
            while rclpy.ok():
                self.get_logger().info(f"Parsing file: {self.filename}")
                self.pos = {'X': 0.0, 'Y': 0.0, 'Z': -0.22, 'A': 0.0, 'C': 0.0}
                
                if not os.path.exists(self.filename):
                    self.get_logger().error(f"File not found: {self.filename}")
                    time.sleep(1.0)
                    continue

                with open(self.filename, 'r') as f:
                    gcode_text = f.read()

                parsed_lines = GcodeParser(gcode_text).lines

                for line in parsed_lines:
                    if not rclpy.ok(): break
                    self.execute_command(line)
                
                self.get_logger().info("Job Complete. Restarting in 1s...")
                time.sleep(1.0)
        except KeyboardInterrupt:
            self.get_logger().info("Stopping G-Code Looper...")

    def execute_command(self, line):
        cmd = line.command_str
        params = line.params
        
        # --- Speed Control (F) ---
        if 'F' in params:
            f_val = float(params['F'])
            # NEW LOGIC: F is mm/s (per user instruction) or scaled up mm/min
            # 1.0 = 1m/s = 1000mm/s
            self.robot_speed = f_val / 1000.0 
            if self.robot_speed < 0.001: self.robot_speed = 0.001
            
            self.get_logger().info(f"Speed Update: F{f_val} -> {self.robot_speed:.4f} m/s")

        # --- State Commands ---
        if cmd == 'G90':
            self.mode = 'ABS'
        elif cmd == 'G91':
            self.mode = 'REL'
        elif cmd == 'G20':
            self.unit_scale = 0.0254
            self.get_logger().info("Units: Inches")
        elif cmd == 'G21':
            self.unit_scale = 0.001
            self.get_logger().info("Units: mm")
        elif cmd == 'G28':
            self.get_logger().info("Homing...")
            self.move_robot(0.0, 0.0, -0.22, 0.0, 0.0)

        # --- Motion ---
        elif cmd in ['G0', 'G1']:
            target = self.calculate_target(params)
            self.move_robot(*target)

    def calculate_target(self, params):
        next_pos = self.pos.copy()
        axis_map = {'X': 'X', 'Y': 'Y', 'Z': 'Z', 'A': 'A', 'C': 'C'}
        
        for g_key, internal_key in axis_map.items():
            if g_key in params:
                val = float(params[g_key])
                if internal_key in ['X', 'Y', 'Z']:
                    val_meters = val * self.unit_scale
                else:
                    val_meters = val
                
                if self.mode == 'REL':
                    next_pos[internal_key] += val_meters
                else:
                    next_pos[internal_key] = val_meters

        return (next_pos['X'], next_pos['Y'], next_pos['Z'], next_pos['A'], next_pos['C'])

    def move_robot(self, x, y, z, a, c):
        start_x, start_y, start_z = self.pos['X'], self.pos['Y'], self.pos['Z']
        
        dx = x - start_x
        dy = y - start_y
        dz = z - start_z
        total_dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # --- TRAJECTORY GENERATION ---
        # Constants
        ACCEL = 0.25 # m/s^2 (Moderate acceleration)
        DT = 0.05    # 50ms (Matches ESP32 Control Loop)
        
        if total_dist < 0.001:
            # Too small to profile, just jump
            duration = 0.1
            points = [(x, y, z, duration)]
        else:
            # 1. Calculate Profile
            v_cruise = self.robot_speed
            
            # Time to accelerate to cruise speed
            t_accel = v_cruise / ACCEL
            d_accel = 0.5 * ACCEL * t_accel**2
            
            if 2 * d_accel > total_dist:
                # Triangular Profile (Cannot reach v_cruise)
                d_accel = total_dist / 2.0
                t_accel = math.sqrt(2 * d_accel / ACCEL)
                t_cruise = 0.0
                d_cruise = 0.0
            else:
                # Trapezoidal Profile
                d_cruise = total_dist - 2 * d_accel
                t_cruise = d_cruise / v_cruise
                
            t_total = 2 * t_accel + t_cruise
            
            # 2. Sample Points
            points = []
            t = DT
            while t < t_total:
                # Calculate Distance at t
                if t <= t_accel:
                    # Accelerating
                    d = 0.5 * ACCEL * t**2
                elif t <= (t_accel + t_cruise):
                    # Cruising
                    d = d_accel + v_cruise * (t - t_accel)
                else:
                    # Decelerating
                    t_decel = t - (t_accel + t_cruise)
                    d = d_accel + d_cruise + (v_cruise * t_decel) - (0.5 * ACCEL * t_decel**2)
                
                # Interpolate Position
                ratio = d / total_dist
                px = start_x + dx * ratio
                py = start_y + dy * ratio
                pz = start_z + dz * ratio
                
                points.append((px, py, pz, t))
                t += DT
            
            # Ensure final point is exact
            points.append((x, y, z, t_total))

        # --- PUBLISH MESSAGE ---
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['x', 'y', 'z']
        
        for px, py, pz, t_abs in points:
            p = JointTrajectoryPoint()
            p.positions = [float(px), float(py), float(pz)]
            
            # time_from_start uses ABSOLUTE TIME from start of move
            sec = int(t_abs)
            nsec = int((t_abs % 1) * 1e9)
            p.time_from_start = Duration(sec=sec, nanosec=nsec)
            
            msg.points.append(p)
            
        self.traj_pub_.publish(msg)
        
        final_time = points[-1][3]
        self.get_logger().info(f"Move: {total_dist:.3f}m in {final_time:.2f}s (Pts: {len(points)})")
        
        # Update State
        self.pos['X'] = x
        self.pos['Y'] = y
        self.pos['Z'] = z
        self.pos['A'] = a
        self.pos['C'] = c
        
        # Block until move completes
        time.sleep(final_time)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: python3 delta_gcode_interpreter.py <file.gcode>")
        sys.exit(1)
    filename = sys.argv[1]
    node = GCodeVirtualMachine(filename)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()