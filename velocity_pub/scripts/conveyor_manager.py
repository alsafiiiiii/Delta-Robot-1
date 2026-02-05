#!/usr/bin/env python3
"""
Conveyor Manager V3 - Object Pooling (No Lag)
- Spawns 6 boxes at startup in a "Holding Area" (X=10.0)
- Teleports them to the belt when needed
- Zero runtime spawning/despawning overhead
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import subprocess
import random
import math
import time

class ConveyorManagerV3(Node):
    def __init__(self):
        super().__init__('conveyor_manager')
        
        # Config
        self.world_name = "empty"
        self.box_base_name = "box_pool"
        self.pool_size = 3
        self.sdf_path = "/home/rikisu/major_project_ws/src/delta_robot_description/models/box.sdf"
        
        # Locations
        # Locations
        self.HOLDING_AREA_X = 0.0 ; self.HOLDING_AREA_Y = 1.0 # Store in Y instead of X
        self.SPAWN_X = 0.5
        self.SPAWN_Z = 0.2
        
        # State
        self.current_box_index = 0
        self.active_boxes = {} # {box_name: age}
        
        # Timing
        self.SPAWN_INTERVAL = 25 # 5s
        self.spawn_timer = 0
        
        # Conveyor control
        self.conveyor_pub = self.create_publisher(Float64, '/conveyor/cmd_vel', 10)
        self.conveyor_sub = self.create_subscription(Float64, '/conveyor/cmd_vel', self.speed_cb, 10)
        self.conveyor_speed = -0.15
        
        # Initial Spawn (Blocking, but only once)
        self.spawn_pool()
        
        # Start first box immediately
        self.activate_next_box()

        # Main loop (10Hz)
        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info("Conveyor Manager: Object Pooling Active")

    def speed_cb(self, msg):
        self.conveyor_speed = msg.data

    def spawn_pool(self):
        self.get_logger().info(f"Initializing Pool of {self.pool_size} boxes...")
        for i in range(self.pool_size):
            box_name = f"{self.box_base_name}_{i}"
            box_name = f"{self.box_base_name}_{i}"
            # Spawn in Holding Area (Y offset)
            hold_y = self.HOLDING_AREA_Y + (i * 0.2)
            
            cmd = [
                "ros2", "run", "ros_gz_sim", "create",
                "-file", self.sdf_path,
                "-name", box_name,
                "-x", "0.0",
                "-y", str(hold_y),
                "-z", "0.1",
                "-allow_renaming", "false"
            ]
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.2) # Small stagger to be safe
        self.get_logger().info("Pool Ready.")

    def update(self):
        # 1. Publish Speed
        msg = Float64()
        msg.data = self.conveyor_speed
        self.conveyor_pub.publish(msg)

        # 2. Spawn Logic (Teleport from Pool)
        self.spawn_timer += 1
        if self.spawn_timer >= self.SPAWN_INTERVAL:
            self.activate_next_box()
            self.spawn_timer = 0
            
        # 3. Despawn Logic (Teleport to Holding)
        active_names = list(self.active_boxes.keys())
        for name in active_names:
            self.active_boxes[name] += 1
            if self.active_boxes[name] > 100: # 10s lifetime (Reduced from 30s to save physics)
                self.deactivate_box(name)

    def activate_next_box(self):
        box_name = f"{self.box_base_name}_{self.current_box_index}"
        
        # Cycle index
        self.current_box_index = (self.current_box_index + 1) % self.pool_size
        
        # Randomize start
        rand_y = random.uniform(-0.075, 0.075)
        rand_yaw = random.uniform(0, 3.14)
        
        self.teleport_box(box_name, self.SPAWN_X, rand_y, self.SPAWN_Z, rand_yaw)
        self.active_boxes[box_name] = 0
        self.get_logger().info(f"Activated {box_name}")

    def deactivate_box(self, box_name):
        if box_name in self.active_boxes:
            # Teleport back to holding
            # Teleport back to holding
            self.teleport_box(box_name, 0.0, self.HOLDING_AREA_Y, 0.1, 0.0)
            del self.active_boxes[box_name]
            self.get_logger().info(f"Recycled {box_name}")

    def teleport_box(self, name, x, y, z, yaw):
        # Construct Protobuf-ish string manually for gz service
        # GZ Pose cmd: "name: 'foo', position: {x: 1, y: 2, z: 3}, orientation: {x: 0, y: 0, z: 0, w: 1}"
        
        # Orientation (Yaw to Quat)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        qw = cy
        qz = sy
        
        req_str = (f'name: "{name}" '
                   f'position: {{x: {x}, y: {y}, z: {z}}} '
                   f'orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}')
        
        cmd = [
            "gz", "service", "-s", f"/world/{self.world_name}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--req", req_str,
            "--timeout", "100"
        ]
        
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

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