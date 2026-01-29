#!/usr/bin/env python3
"""
Conveyor Manager V3 - Simple Timed Spawning
- Spawns a box every 10 seconds
- Despawns each box after 2 minutes
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import subprocess
import random
import math

class ConveyorManagerV3(Node):
    def __init__(self):
        super().__init__('conveyor_manager')
        
        # Config
        self.world_name = "empty"
        self.box_name = "box_conveyor"
        self.sdf_path = "/home/rikisu/major_project_ws/src/delta_robot_description/models/box.sdf"
        
        # Spawn position
        self.spawn_x = 0.5
        self.spawn_y = 0.0
        self.spawn_z = 0.2
        
        # State
        self.box_counter = 0
        self.active_boxes = {}  # {box_name: ticks_alive}
        
        # Timing (10Hz timer)
        self.SPAWN_INTERVAL = 100   # 10 seconds (100 ticks @ 10Hz)
        self.BOX_LIFETIME = 300    # 2 minutes (1200 ticks @ 10Hz)
        self.spawn_timer = 0
        self.startup_delay = 30     # 3 seconds before first spawn
        
        self.get_logger().info("Conveyor Manager V3 Started (Spawn every 10s, Despawn after 2min)")
        
        # Main loop timer (10Hz)
        self.timer = self.create_timer(0.1, self.update)

    def update(self):
        # Startup delay
        if self.startup_delay > 0:
            self.startup_delay -= 1
            if self.startup_delay == 0:
                self.spawn_box()
            return
        
        # Increment spawn timer
        self.spawn_timer += 1
        if self.spawn_timer >= self.SPAWN_INTERVAL:
            # Limit to 3 active boxes
            if len(self.active_boxes) < 3:
                self.spawn_box()
            self.spawn_timer = 0
        
        # Update lifetimes and despawn old boxes
        boxes_to_remove = []
        for box_name, age in self.active_boxes.items():
            self.active_boxes[box_name] = age + 1
            if self.active_boxes[box_name] >= self.BOX_LIFETIME:
                boxes_to_remove.append(box_name)
        
        for box_name in boxes_to_remove:
            self.despawn_box(box_name)
            del self.active_boxes[box_name]

    def spawn_box(self):
        self.box_counter += 1
        box_name = f"{self.box_name}_{self.box_counter}"
        
        # Random Y and Yaw
        random_y = random.uniform(-0.075, 0.075)
        random_yaw = random.uniform(0, 2 * math.pi)
        
        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-file", self.sdf_path,
            "-name", box_name,
            "-x", str(self.spawn_x),
            "-y", str(random_y),
            "-z", str(self.spawn_z),
            "-Y", str(random_yaw),
            "-allow_renaming", "false"
        ]
        
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.active_boxes[box_name] = 0
            self.get_logger().info(f"Spawned {box_name}")
        except Exception as e:
            self.get_logger().error(f"Spawn failed: {e}")

    def despawn_box(self, box_name):
        cmd = [
            "gz", "service", "-s", f"/world/{self.world_name}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--req", f'name: "{box_name}" type: MODEL',
            "--timeout", "1000"
        ]
        
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f"Despawned {box_name} (aged out)")
        except Exception as e:
            self.get_logger().error(f"Despawn failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorManagerV3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()