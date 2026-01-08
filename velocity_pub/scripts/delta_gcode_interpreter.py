#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
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
        
        # --- Robot Settings ---
        self.robot_speed = 0.05  # m/s (Average execution speed)
        
        # --- Virtual Machine State ---
        self.mode = 'ABS'         # 'ABS' (G90) or 'REL' (G91)
        
        # CHANGED: Default to 1.0 (Meters) instead of 0.001. 
        # If the file contains G21, it will switch to 0.001 automatically.
        self.unit_scale = 1.0   
        
        # Current Position (Always stored in Absolute Meters, Radians)
        self.pos = {
            'X': 0.0,
            'Y': 0.0,
            'Z': -0.25, # Default Start Height
            'A': 0.0,   # Tilt
            'C': 0.0    # Spin
        }
        
        self.filename = filename
        
        # Wait for ROS connection
        self.get_logger().info("Connecting to ROS...")
        time.sleep(2.0)
        
        self.run_gcode()

    def run_gcode(self):
        self.get_logger().info(f"Parsing file: {self.filename}")
        
        if not os.path.exists(self.filename):
            self.get_logger().error(f"File not found: {self.filename}")
            return

        # 1. Read File
        with open(self.filename, 'r') as f:
            gcode_text = f.read()

        # 2. Parse using Library
        parsed_lines = GcodeParser(gcode_text).lines

        # 3. Execute Line by Line
        for line in parsed_lines:
            if not rclpy.ok(): break
            self.execute_command(line)
            
        self.get_logger().info("Job Complete.")

    def execute_command(self, line):
        cmd = line.command_str  # e.g., 'G1', 'G90'
        params = line.params    # e.g., {'X': 10.0}
        
        # --- State Control Commands ---
        if cmd == 'G90':
            self.mode = 'ABS'
            self.get_logger().info("Mode: Absolute (G90)")
            return
        elif cmd == 'G91':
            self.mode = 'REL'
            self.get_logger().info("Mode: Incremental (G91)")
            return
        elif cmd == 'G20':
            self.unit_scale = 0.0254 # Inch -> Meter
            self.get_logger().info("Units: Inches (G20) -> Scaling by 0.0254")
            return
        elif cmd == 'G21':
            self.unit_scale = 0.001  # mm -> Meter
            self.get_logger().info("Units: Millimeters (G21) -> Scaling by 0.001")
            return
        elif cmd == 'G28':
            self.get_logger().info("Homing (G28)...")
            self.move_robot(0.0, 0.0, -0.25, 0.0, 0.0)
            return

        # --- Motion Commands (G0, G1) ---
        if cmd in ['G0', 'G1']:
            target = self.calculate_target(params)
            self.move_robot(*target)

    def calculate_target(self, params):
        # Create a temp copy of current pos
        next_pos = self.pos.copy()
        
        axis_map = {'X': 'X', 'Y': 'Y', 'Z': 'Z', 'A': 'A', 'C': 'C'}
        
        for g_key, internal_key in axis_map.items():
            if g_key in params:
                raw_val = params[g_key]
                
                # Apply Unit Scale only to XYZ, not Angles
                if internal_key in ['X', 'Y', 'Z']:
                    val_meters = raw_val * self.unit_scale
                else:
                    val_meters = raw_val # Assumes Radians in G-code

                if self.mode == 'REL':
                    next_pos[internal_key] += val_meters
                else:
                    next_pos[internal_key] = val_meters

        return (next_pos['X'], next_pos['Y'], next_pos['Z'], next_pos['A'], next_pos['C'])

    def move_robot(self, x, y, z, a, c):
        # 1. Calculate travel distance
        dx = x - self.pos['X']
        dy = y - self.pos['Y']
        dz = z - self.pos['Z']
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # 2. Calculate Wait Time (CORNER BLENDING FACTOR)
        # 1.0 = Stop exactly at corner (Risk of stutter)
        # 1.1 = Stop and wait (Bad for speed)
        # 0.9 = Send next command early (Continuous motion!)
        BLEND_FACTOR = 0.9 
        
        if dist > 0:
            # We calculate time based on the Controller's known speed
            wait_time = (dist / self.robot_speed) * BLEND_FACTOR
        else:
            wait_time = 0.1 # Minimal delay for rotation only
            
        # 3. Publish Pose
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        
        # (Assuming you use the quaternion function defined globally)
        q = get_quaternion_from_euler(a, 0.0, c)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        
        self.publisher_.publish(msg)
        self.get_logger().info(f"Moving -> X{x:.3f} Y{y:.3f} (Time: {wait_time:.2f}s)")
        
        # 4. Update Internal State
        self.pos['X'] = x
        self.pos['Y'] = y
        self.pos['Z'] = z
        self.pos['A'] = a
        self.pos['C'] = c
        
        # 5. Sleep just enough to keep the buffer full, but not enough to stop the robot
        time.sleep(wait_time)

def main(args=None):
    rclpy.init(args=args)
    
    # --- ARGUMENT CHECKING ---
    if len(sys.argv) < 2:
        print("\nERROR: Missing G-Code file path.")
        print("Usage: python3 gcode_sender.py <path_to_file.gcode>\n")
        print("Example: python3 gcode_sender.py square_test.gcode")
        sys.exit(1)
        
    filename = sys.argv[1]
    
    node = GCodeVirtualMachine(filename)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()