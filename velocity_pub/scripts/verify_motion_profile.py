#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import numpy as np
import time

class MotionVerifier(Node):
    def __init__(self):
        super().__init__('motion_verifier')
        self.points = []
        self.recording = False
        self.start_time = None
        
        self.sub = self.create_subscription(
            JointTrajectory, 
            '/model/delta_robot/joint_trajectory', 
            self.callback, 
            10)
        self.get_logger().info("Verifier Ready. Please trigger a robot move...")

    def callback(self, msg):
        if not msg.points: return
        
        # Extract Joint 1 Position (rad)
        pos = msg.points[0].positions[0]
        now = time.time()
        
        if not self.recording:
            # Start recording if we detect change (simple logic)
            if len(self.points) > 0 and abs(pos - self.points[-1][1]) > 0.0001:
                self.recording = True
                self.start_time = now
                self.get_logger().info("Motion Detected! Recording...")
        
        self.points.append((now, pos))
        
        # Stop recording if stable for 1 second after moving
        if self.recording and (now - self.start_time > 1.0):
            # Check stability
            last_30 = [p[1] for p in self.points[-30:]]
            if np.std(last_30) < 0.0001 and len(self.points) > 100:
                self.get_logger().info("Motion Stopped. Analyzing...")
                self.analyze()
                rclpy.shutdown()

    def analyze(self):
        # Convert to numpy
        data = np.array(self.points)
        t = data[:, 0]
        pos = data[:, 1]
        
        # Calculate Derivatives
        dt = np.diff(t)
        # Filter tiny dt to avoid huge spikes from jitter
        valid = dt > 0.001 
        t = t[:-1][valid]
        dt = dt[valid]
        dpos = np.diff(pos)[valid]
        
        vel = dpos / dt
        acc = np.diff(vel) / dt[:-1]
        
        # Statistics
        max_vel = np.max(np.abs(vel))
        max_acc = np.max(np.abs(acc))
        start_vel = np.mean(np.abs(vel[:5]))
        end_vel = np.mean(np.abs(vel[-5:]))
        
        print("\n" + "="*40)
        print("COMPUTATIONAL VERIFICATION REPORT")
        print("="*40)
        print(f"Total Points: {len(pos)}")
        print(f"Max Velocity: {max_vel:.4f} rad/s")
        print(f"Max Accel:    {max_acc:.4f} rad/s^2")
        print("-" * 20)
        print(f"Start Velocity (Avg first 5): {start_vel:.5f} rad/s")
        print(f"End Velocity   (Avg last 5):  {end_vel:.5f} rad/s")
        print("-" * 20)
        
        if start_vel < 0.01 and end_vel < 0.01:
            print("✅ SUCCESS: Motion starts and ends at rest.")
        else:
            print("❌ FAILURE: Non-zero start/end velocity detected!")
            
        print("="*40 + "\n")

def main():
    rclpy.init()
    node = MotionVerifier()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
