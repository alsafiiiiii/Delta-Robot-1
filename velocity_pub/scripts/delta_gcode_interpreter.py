#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool
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
        
        # --- Config ---
        self.filename = filename
        
        # --- Pub/Sub ---
        self.publisher_ = self.create_publisher(Pose, '/delta/target_pose', 10)
        self.speed_pub_ = self.create_publisher(Twist, '/delta/speed_params', 10)
        
        # Feedback from Controller
        self.create_subscription(Bool, '/delta/movement_done', self.movement_done_callback, 10)
        
        # --- State ---
        self.robot_speed = 0.05  # m/s (Default)
        self.mode = 'ABS'         # 'ABS' (G90) or 'REL' (G91)
        self.unit_scale = 1.0     # Default to Meters (1.0)
        self.pos = {'X': 0.0, 'Y': 0.0, 'Z': -0.25, 'A': 0.0, 'C': 0.0}
        
        self.lines = []
        self.current_line_idx = 0
        self.waiting_for_movement = False
        self.job_active = False
        
        # Load File
        self.load_gcode()
        
        # Execution Timer (Using timer instead of while loop to be non-blocking)
        self.timer = self.create_timer(0.01, self.execute_loop) # 100Hz check
        
        self.get_logger().info("G-Code VM Ready. Waiting for ROS2...")

    def load_gcode(self):
        if not os.path.exists(self.filename):
            self.get_logger().error(f"File not found: {self.filename}")
            sys.exit(1)

        with open(self.filename, 'r') as f:
            gcode_text = f.read()

        self.lines = GcodeParser(gcode_text).lines
        self.current_line_idx = 0
        self.job_active = True
        self.get_logger().info(f"Loaded {len(self.lines)} lines. Starting...")

    def movement_done_callback(self, msg):
        if self.waiting_for_movement and msg.data:
            self.waiting_for_movement = False
            self.get_logger().debug("Movement ACK received.")

    def execute_loop(self):
        if not self.job_active: return
        
        # If we are waiting for robot to finish a move, DO NOTHING
        if self.waiting_for_movement:
            return
            
        # If we are done with file
        if self.current_line_idx >= len(self.lines):
            self.get_logger().info("G-Code Job Complete!")
            self.job_active = False
            # Optional: Loop?
            # self.current_line_idx = 0 
            # self.job_active = True
            return

        # Execute Next Line
        line = self.lines[self.current_line_idx]
        self.current_line_idx += 1
        
        self.process_command(line)

    def process_command(self, line):
        cmd = line.command_str
        params = line.params
        
        # --- Speed (F) ---
        if 'F' in params:
            f_val = float(params['F'])
            # Standard G-Code: F is usually units/min
            # Heuristic: If value > 10, it's probably mm/min
            if f_val > 10.0:
                self.robot_speed = (f_val * self.unit_scale) / 60.0
            else:
                self.robot_speed = f_val # Assume m/s if very small?
                
            self.robot_speed = max(0.001, self.robot_speed) # Safety
            
            # Publish Speed Update
            msg = Twist()
            msg.linear.x = float(self.robot_speed)
            msg.angular.z = 1.0 
            self.speed_pub_.publish(msg)

        # --- G Codes ---
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
        # Update Internal State
        self.pos = {'X': x, 'Y': y, 'Z': z, 'A': a, 'C': c}
        
        # Create Pose Msg
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
        self.get_logger().info(f"Target Sent: {x:.3f}, {y:.3f}, {z:.3f}")
        
        # BLOCK NEXT COMMAND until we get confirmation
        self.waiting_for_movement = True

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: python3 delta_gcode_interpreter.py <file.gcode>")
        sys.exit(1)
        
    node = GCodeVirtualMachine(sys.argv[1])
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()