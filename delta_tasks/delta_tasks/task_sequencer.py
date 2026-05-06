#!/usr/bin/env python3
"""
Task Sequencer for Delta Robot

Python-based alternative to G-code for task automation.
Uses JSON task files for human-readable sequence definitions.

Example task file:
[
    {"action": "move", "x": 0.05, "y": 0, "z": -0.22},
    {"action": "tilt", "angle": 30},
    {"action": "suction", "on": true},
    {"action": "wait", "seconds": 0.5},
    {"action": "home"}
]
"""

import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import json
import math
import time
import sys

class TaskSequencer(Node):
    def __init__(self, task_file=None, loop_enabled=False):
        super().__init__('task_sequencer')
        
        # Publishers
        self.pose_pub = self.create_publisher(Pose, '/delta/target_pose', 10)
        self.suction_pub = self.create_publisher(Bool, '/suction/command', 10)
        
        # State
        self.current_pos = [0.0, 0.0, -0.27]
        self.current_tilt = 0.0
        self.current_spin = 0.0
        
        # Default speeds
        self.default_duration = 1.0  # seconds per move
        
        # Home position
        self.HOME = {"x": 0.0, "y": 0.0, "z": -0.27, "tilt": 0.0, "spin": 0.0}
        
        self.get_logger().info("Task Sequencer Ready")
        self.loop_enabled = loop_enabled
        
        if task_file:
            self.load_and_run(task_file)
    
    def load_and_run(self, filepath):
        """Load and execute a task file."""
        try:
            with open(filepath, 'r') as f:
                tasks = json.load(f)
            
            self.get_logger().info(f"Loaded {len(tasks)} tasks from {filepath}")
            while rclpy.ok():
                self.execute_tasks(tasks)
                if not self.loop_enabled:
                    break
                self.get_logger().info("Task loop complete. Restarting in 1s...")
                time.sleep(1.0)
            
        except FileNotFoundError:
            self.get_logger().error(f"Task file not found: {filepath}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON: {e}")
    
    def execute_tasks(self, tasks):
        """Execute a list of task dictionaries."""
        for i, task in enumerate(tasks):
            action = task.get("action", "").lower()
            self.get_logger().info(f"[{i+1}/{len(tasks)}] Executing: {action}")
            
            if action == "move":
                self.do_move(task)
            elif action == "tilt":
                self.do_tilt(task)
            elif action == "spin":
                self.do_spin(task)
            elif action == "suction":
                self.do_suction(task)
            elif action == "wait":
                self.do_wait(task)
            elif action == "home":
                self.do_home(task)
            else:
                self.get_logger().warn(f"Unknown action: {action}")
        
        self.get_logger().info("Task sequence complete!")
    
    def do_move(self, task):
        """Move to a position."""
        x = task.get("x", self.current_pos[0])
        y = task.get("y", self.current_pos[1])
        z = task.get("z", self.current_pos[2])
        duration = task.get("duration", self.default_duration)
        
        self.current_pos = [x, y, z]
        self.publish_pose()
        time.sleep(duration)
    
    def do_tilt(self, task):
        """Tilt end-effector (degrees)."""
        angle_deg = task.get("angle", 0)
        self.current_tilt = math.radians(angle_deg)
        
        duration = task.get("duration", 0.5)
        self.publish_pose()
        time.sleep(duration)
    
    def do_spin(self, task):
        """Spin end-effector (degrees)."""
        angle_deg = task.get("angle", 0)
        self.current_spin = math.radians(angle_deg)
        
        duration = task.get("duration", 0.5)
        self.publish_pose()
        time.sleep(duration)
    
    def do_suction(self, task):
        """Control suction gripper."""
        on = task.get("on", False)
        
        msg = Bool()
        msg.data = on
        for _ in range(5):  # Send multiple times for reliability
            self.suction_pub.publish(msg)
            time.sleep(0.05)
        
        self.get_logger().info(f"Suction: {'ON' if on else 'OFF'}")
    
    def do_wait(self, task):
        """Wait for specified time."""
        seconds = task.get("seconds", 1.0)
        time.sleep(seconds)
    
    def do_home(self, task):
        """Return to home position."""
        duration = task.get("duration", self.default_duration)
        
        self.current_pos = [self.HOME["x"], self.HOME["y"], self.HOME["z"]]
        self.current_tilt = self.HOME["tilt"]
        self.current_spin = self.HOME["spin"]
        
        self.publish_pose()
        time.sleep(duration)
    
    def publish_pose(self):
        """Publish current pose to controller."""
        msg = Pose()
        msg.position.x = self.current_pos[0]
        msg.position.y = self.current_pos[1]
        msg.position.z = self.current_pos[2]
        
        # Convert tilt/spin to quaternion
        q = self.euler_to_quaternion(self.current_tilt, 0, self.current_spin)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        
        self.pose_pub.publish(msg)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert euler angles to quaternion."""
        cr, sr = math.cos(roll/2), math.sin(roll/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        
        return [qx, qy, qz, qw]


def main(args=None):
    parser = argparse.ArgumentParser(description="Delta task sequencer")
    parser.add_argument("task_file", nargs="?", help="Path to a JSON task file")
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Loop the task list continuously",
    )
    parsed = parser.parse_args()

    rclpy.init(args=args)

    node = TaskSequencer(parsed.task_file, loop_enabled=parsed.loop)

    if not parsed.task_file:
        rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
