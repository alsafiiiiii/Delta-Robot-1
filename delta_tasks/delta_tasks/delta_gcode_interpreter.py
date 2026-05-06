#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node
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

class GCodeVirtualMachine(Node):
    def __init__(self, filename, loop_enabled):
        super().__init__('gcode_sender')
        self.traj_pub_ = self.create_publisher(JointTrajectory, '/delta/cartesian_trajectory', 10)
        self.loop_enabled = loop_enabled
        
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
                
                if not self.loop_enabled:
                    break
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
                    # Scale linear units to meters
                    val_converted = val * self.unit_scale
                else:
                    # G-Code standard uses degrees for A and C.
                    # Convert to Radians for the controller node.
                    val_converted = math.radians(val)
                
                if self.mode == 'REL':
                    next_pos[internal_key] += val_converted
                else:
                    next_pos[internal_key] = val_converted

        return (next_pos['X'], next_pos['Y'], next_pos['Z'], next_pos['A'], next_pos['C'])

    def move_robot(self, x, y, z, a, c):
        dx = x - self.pos['X']
        dy = y - self.pos['Y']
        dz = z - self.pos['Z']
        da = a - self.pos['A']
        dc = c - self.pos['C']
        
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        angular_dist = math.sqrt(da**2 + dc**2)

        # Calculate duration based on distance, or angular rotation if stationary
        if dist > 0.0001:
            duration_sec = dist / self.robot_speed
        elif angular_dist > 0.001:
            # If the move is a pure rotation, estimate time based on 1 rad/sec rotation speed
            duration_sec = angular_dist / 1.0 
        else:
            duration_sec = 0.250 # Minimum move time for safety/homing

        # Interpolation parameters
        interp_rate = 100.0  # Hz
        steps = max(2, int(duration_sec * interp_rate))
        sleep_dt = duration_sec / steps

        for i in range(1, steps + 1):
            alpha = i / steps
            interp_x = self.pos['X'] + dx * alpha
            interp_y = self.pos['Y'] + dy * alpha
            interp_z = self.pos['Z'] + dz * alpha
            interp_a = self.pos['A'] + da * alpha
            interp_c = self.pos['C'] + dc * alpha

            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = ['x', 'y', 'z', 'tilt', 'spin']

            point = JointTrajectoryPoint()
            # NOW PUBLISHING ALL 5 POSITIONS
            point.positions = [
                float(interp_x), float(interp_y), float(interp_z), 
                float(interp_a), float(interp_c)
            ]
            point.time_from_start = Duration(sec=0, nanosec=int(sleep_dt * 1e9))
            msg.points.append(point)

            self.traj_pub_.publish(msg)
            time.sleep(sleep_dt)

        self.get_logger().info(f"Move: ({x:.3f}, {y:.3f}, {z:.3f}, A:{math.degrees(a):.1f}°, C:{math.degrees(c):.1f}°) T={duration_sec:.3f}s")

        # Update State
        self.pos['X'] = x
        self.pos['Y'] = y
        self.pos['Z'] = z
        self.pos['A'] = a
        self.pos['C'] = c

def main(args=None):
    parser = argparse.ArgumentParser(description="Delta G-code interpreter")
    parser.add_argument("filename", help="Path to a G-code file")
    parser.add_argument(
        "--once",
        action="store_true",
        help="Run the file once and exit (default is to loop)",
    )
    parsed = parser.parse_args()

    rclpy.init(args=args)
    node = GCodeVirtualMachine(parsed.filename, loop_enabled=not parsed.once)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()