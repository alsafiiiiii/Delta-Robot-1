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
        dx = x - self.pos['X']
        dy = y - self.pos['Y']
        dz = z - self.pos['Z']
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        # Calculate duration
        if dist > 0.0001:
            duration_sec = dist / self.robot_speed
        else:
            duration_sec = 0.250 # Minimum move time for safety/homing

        # Create Cartesian Trajectory Message (5DOF)
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['x', 'y', 'z', 'a', 'c'] # 5DOF axes

        point = JointTrajectoryPoint()
        point.positions = [float(x), float(y), float(z), float(a), float(c)]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1)*1e9))
        msg.points.append(point)

        self.traj_pub_.publish(msg)

        self.get_logger().info(f"Move: ({x:.3f}, {y:.3f}, {z:.3f}, {a:.3f}, {c:.3f}) T={duration_sec:.3f}s")

        # Update State
        self.pos['X'] = x
        self.pos['Y'] = y
        self.pos['Z'] = z
        self.pos['A'] = a
        self.pos['C'] = c

        # Wait for move to complete (VM blocking)
        time.sleep(duration_sec)

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