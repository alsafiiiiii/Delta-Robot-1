#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
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
        self.publisher_ = self.create_publisher(Pose, '/delta/target_pose', 10)
        self.speed_pub_ = self.create_publisher(Twist, '/delta/speed_params', 10)
        
        # --- Robot Settings ---
        self.robot_speed = 0.05  # m/s (Default)
        
        # --- Virtual Machine State ---
        self.mode = 'ABS'         # 'ABS' (G90) or 'REL' (G91)
        self.unit_scale = 1.0     # Default to Meters (1.0)
        
        # Current Position (Absolute Meters, Radians)
        self.pos = {
            'X': 0.0, 'Y': 0.0, 'Z': -0.25, 'A': 0.0, 'C': 0.0
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
            
            # Heuristic: If F > 100 with unit_scale=1.0, user probably meant mm/min
            # but we respect strict G-code if possible.
            # Standard: F is units/min.
            
            speed_per_min = f_val
            if self.unit_scale != 1.0: # If mm (0.001) or inch (0.0254)
                # Convert active units to meters
                speed_meters_min = speed_per_min * self.unit_scale
            else:
                # Active units are Meters
                speed_meters_min = speed_per_min
                
            self.robot_speed = speed_meters_min / 60.0
            self.robot_speed = max(0.002, self.robot_speed) # Safety Floor

            # Publish Update
            s_msg = Twist()
            s_msg.linear.x = float(self.robot_speed)
            s_msg.angular.z = 1.0 
            self.speed_pub_.publish(s_msg)
            
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
            self.move_robot(0.0, 0.0, -0.25, 0.0, 0.0)

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
                
                # Apply Unit Scale to Linear Axes only
                if internal_key in ['X', 'Y', 'Z']:
                    val_meters = val * self.unit_scale
                else:
                    val_meters = val # Angles are radians

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
        
        # Use simple Factor to allow smooth continuous motion
        # If we sleep EXACTLY the time, rounding errors might cause stutter
        # 0.8 means we send the next command when 80% of the way there, creating a blend
        BLEND_FACTOR = 1.0
        
        if dist > 0.0001:
            wait_time = (dist / self.robot_speed) * BLEND_FACTOR
        else:
            wait_time = 0.05
            
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        
        q = get_quaternion_from_euler(a, 0.0, c)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        
        self.publisher_.publish(msg)
        self.get_logger().info(f"Move: X{x:.3f} Y{y:.3f} Z{z:.3f} (T={wait_time:.2f}s, V={self.robot_speed:.3f})")
        
        self.pos['X'] = x
        self.pos['Y'] = y
        self.pos['Z'] = z
        self.pos['A'] = a
        self.pos['C'] = c
        
        time.sleep(wait_time)

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