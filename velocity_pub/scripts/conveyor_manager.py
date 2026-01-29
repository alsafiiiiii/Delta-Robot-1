#!/usr/bin/env python3
"""
Conveyor Manager for Delta Robot Box PnP
- Spawns 'box_conveyor' at Y=0.3
- Moves it towards Y=0.0
- Delete and Respawn on 'DROPPED' signal
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time
import os

class ConveyorManager(Node):
    def __init__(self):
        super().__init__('conveyor_manager')
        
        self.sub = self.create_subscription(String, '/pnp/status', self.callback, 10)
        self.world_name = "empty"
        self.box_name = "box_conveyor"
        self.sdf_path = "/home/rikisu/major_project_ws/src/delta_robot_description/models/box.sdf"
        
        self.current_y = 0.3
        self.moving = True
        
        self.get_logger().info("Conveyor Manager Started. Spawning first box...")
        self.spawn_box()
        
        # Timer for animation (10Hz)
        self.timer = self.create_timer(0.1, self.update_pose)

    def callback(self, msg):
        if msg.data == "DROPPED":
            self.get_logger().info("Box Dropped! Recycling...")
            self.recycle_box()

    def spawn_box(self):
        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-file", self.sdf_path,
            "-name", self.box_name,
            "-x", "0.0", "-y", "0.3", "-z", "0.05",
            "-allow_renaming", "false"
        ]
        # Run async, don't wait
        subprocess.Popen(cmd)
        self.current_y = 0.3
        self.moving = True

    def delete_box(self):
        req = f'entity: {{ name: "{self.box_name}" type: MODEL }}'
        cmd = [
            "gz", "service", "-s", f"/world/{self.world_name}/remove",
            "--reqtype", "gz.msgs.Boolean",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", req
        ]
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().error(f"Delete failed: {e}")

    def recycle_box(self):
        self.moving = False
        self.delete_box()
        # Small delay to ensure deletion
        time.sleep(0.5)
        self.spawn_box()

    def update_pose(self):
        if not self.moving:
            return
            
        if self.current_y > 0.0:
            self.current_y -= 0.005 # 5cm/s speed
            if self.current_y < 0.0:
                self.current_y = 0.0
            
            # Send Pose Update
            # Z=0.025 is center of 0.05 sim box
            req = f'name: "{self.box_name}" position {{ x: 0 y: {self.current_y:.3f} z: 0.025 }}'
            
            cmd = [
                "gz", "service", "-s", f"/world/{self.world_name}/set_pose",
                "--reqtype", "gz.msgs.Pose",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "100", # Short timeout
                "--req", req
            ]
            # Fire and forget to avoid blocking loop
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.delete_box() # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
