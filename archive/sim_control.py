#!/usr/bin/env python3
"""
Simulation Control Bridge

Mimics ESP32 firmware behavior for Gazebo simulation.
Subscribes to joint commands and publishes interpolated trajectories
at high rate to provide smooth motion matching real hardware.

This node does for simulation what ESP32 firmware does for real hardware:
- Receives target joint positions with duration
- Interpolates smoothly over time using hybrid linear/cubic easing
- Publishes high-rate position updates
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time

class SimControlBridge(Node):
    def __init__(self):
        super().__init__('sim_control_bridge')
        
        # --- CONFIGURATION (Matching ESP32 firmware) ---
        self.CONTROL_FREQ_HZ = 100  # Match ESP32 control loop and 5DOF controller (increased from 50Hz)
        self.HYBRID_FACTOR = 0.8   # 0.0=Linear, 1.0=Cubic (match firmware)
        
        # Current joint state (radians)
        self.current_positions = [0.0] * 7  # All 7 joints
        self.target_positions = [0.0] * 7
        self.start_positions = [0.0] * 7
        self.move_duration = 0.0
        self.move_elapsed = 0.0
        self.is_moving = False
        
        # Joint names (must match SDF)
        self.joint_names = ['jbf1', 'jbf2', 'jbf3', 'Bevelj1', 'Bevelj2', 'Tj1', 'BeveljEE']
        
        # Publishers / Subscribers
        self.pub = self.create_publisher(
            JointTrajectory, 
            '/model/delta_robot/joint_trajectory', 
            10
        )
        
        self.sub = self.create_subscription(
            JointTrajectory,
            '/delta/joint_commands',
            self.command_callback,
            10
        )
        
        # Control loop timer
        self.dt = 1.0 / self.CONTROL_FREQ_HZ
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info(f'Sim Control Bridge Started @ {self.CONTROL_FREQ_HZ}Hz (increased)')
    
    def command_callback(self, msg):
        """Receive new target trajectory from controller."""
        if not msg.points:
            return
        
        # Take the last point as the target (for multi-point, we use final destination)
        point = msg.points[-1]
        
        # Extract duration
        duration_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        if duration_sec < 0.02:
            duration_sec = 0.02  # Minimum 20ms (match ESP32)
        
        # Store start positions
        self.start_positions = list(self.current_positions)
        
        # Extract target positions (pad with zeros if fewer than 7)
        positions = list(point.positions)
        while len(positions) < 7:
            positions.append(self.current_positions[len(positions)])
        
        self.target_positions = positions
        self.move_duration = duration_sec
        self.move_elapsed = 0.0
        self.is_moving = True
        
        self.get_logger().debug(
            f'New target: [{positions[0]:.2f}, {positions[1]:.2f}, {positions[2]:.2f}] '
            f'in {duration_sec:.3f}s'
        )
    
    def control_loop(self):
        """High-rate control loop - interpolates and publishes."""
        if not self.is_moving:
            return
        
        self.move_elapsed += self.dt
        
        if self.move_elapsed >= self.move_duration:
            # Move complete - snap to target
            self.current_positions = list(self.target_positions)
            self.is_moving = False
        else:
            # Interpolate using hybrid easing (matching ESP32)
            t = self.move_elapsed / self.move_duration  # Normalized time [0, 1]
            
            # Cubic ease-in/ease-out: 3t² - 2t³
            cubic = (3 * t * t) - (2 * t * t * t)
            
            # Linear
            linear = t
            
            # Hybrid blend
            ease = (1.0 - self.HYBRID_FACTOR) * linear + self.HYBRID_FACTOR * cubic
            
            # Interpolate each joint
            for i in range(7):
                spread = self.target_positions[i] - self.start_positions[i]
                self.current_positions[i] = self.start_positions[i] + (spread * ease)
        
        # Publish to Gazebo
        self.publish_to_gazebo()
    
    def publish_to_gazebo(self):
        """Publish current interpolated position to Gazebo."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.current_positions
        # Small time_from_start for immediate execution
        point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 0.5 * 1e9))
        msg.points.append(point)
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimControlBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
