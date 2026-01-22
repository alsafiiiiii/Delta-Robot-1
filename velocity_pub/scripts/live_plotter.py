#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import math
import threading
import time

# --- CONFIG (Match robot_control.py) ---
MIN_US = 550
MAX_US = 2400
MIN_DEG = 0
MAX_DEG = 180
OFFSET_DEG = 0.0

class LivePlotter(Node):
    def __init__(self):
        super().__init__('live_plotter')
        
        self.maxlen = 100
        self.times = deque(maxlen=self.maxlen)
        self.pwm_data = deque(maxlen=self.maxlen)
        self.vel_data = deque(maxlen=self.maxlen)
        
        self.last_time = time.time()
        self.last_pos = 0.0
        self.start_time = time.time()
        
        self.sub_traj = self.create_subscription(
            JointTrajectory, 
            '/model/delta_robot/joint_trajectory', 
            self.traj_callback, 
            10)
            
        self.get_logger().info("Live Plotter Started")

    def map_deg_to_us(self, deg):
        deg += OFFSET_DEG
        deg = max(MIN_DEG, min(MAX_DEG, deg))
        us = (deg - MIN_DEG) * (MAX_US - MIN_US) / (MAX_DEG - MIN_DEG) + MIN_US
        return us

    def traj_callback(self, msg):
        if msg.points:
            now = time.time()
            dt = now - self.last_time
            
            # Joint 1 Data
            rad = msg.points[0].positions[0]
            deg = math.degrees(rad)
            us = self.map_deg_to_us(deg)
            
            # Calculate Velocity (deg/s)
            vel = 0.0
            if dt > 0.001: # Avoid div by zero
                vel = (deg - self.last_pos) / dt
            
            # Store
            self.times.append(now - self.start_time)
            self.pwm_data.append(us)
            self.vel_data.append(vel)
            
            self.last_time = now
            self.last_pos = deg

# Global Node
node = None

def update_plot(frame):
    if node and len(node.times) > 1:
        ax1.clear()
        ax2.clear()
        
        # PWM Plot
        ax1.plot(node.times, node.pwm_data, 'b-', label='PWM (us)')
        ax1.set_ylabel('Pulse Width (us)')
        ax1.set_title('Servo 1 Signal')
        ax1.legend(loc='upper left')
        ax1.grid(True)
        
        # Velocity Plot
        ax2.plot(node.times, node.vel_data, 'r-', label='Velocity (deg/s)')
        ax2.set_ylabel('Speed (deg/s)')
        ax2.set_xlabel('Time (s)')
        ax2.legend(loc='upper left')
        ax2.grid(True)

def main():
    global node, ax1, ax2
    rclpy.init()
    node = LivePlotter()
    
    # Run ROS
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    # Setup Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 8))
    ani = animation.FuncAnimation(fig, update_plot, interval=100)
    plt.show()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
