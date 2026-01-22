#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import math
import threading

# --- CONFIG ---
# Match these exactly to your bridge/ESP32
MIN_US = 500
MAX_US = 2400
MIN_DEG = -90.0
MAX_DEG = 90.0
OFFSET_DEG = 0.0 

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('debug_visualizer_d2')
        
        # Buffer for plotting (Last 100 points)
        self.maxlen = 100
        self.t_data = deque(maxlen=self.maxlen)
        self.y1_data = deque(maxlen=self.maxlen)
        
        self.counter = 0

        self.sub = self.create_subscription(
            JointTrajectory, 
            '/model/delta_robot/joint_trajectory', 
            self.listener_callback, 
            10)

    def map_deg_to_us(self, deg):
        deg += OFFSET_DEG
        deg = max(MIN_DEG, min(MAX_DEG, deg))
        us = (deg - MIN_DEG) * (MAX_US - MIN_US) / (MAX_DEG - MIN_DEG) + MIN_US
        return us

    def listener_callback(self, msg):
        if msg.points:
            # Extract ONLY Joint 1 (Index 0 -> Pin D2)
            rad1 = msg.points[0].positions[0]
            
            # Convert
            us1 = self.map_deg_to_us(math.degrees(rad1))
            
            # Store
            self.counter += 1
            self.t_data.append(self.counter)
            self.y1_data.append(us1)

# Global Node
node = None

def update_plot(frame):
    if node and len(node.y1_data) > 0:
        plt.cla()
        
        # Plot only Servo 1
        current_val = node.y1_data[-1]
        plt.plot(node.t_data, node.y1_data, label=f'Servo 1 (D2): {current_val:.1f} us', color='r', linewidth=2)
        
        # Draw limits
        plt.axhline(y=MIN_US, color='k', linestyle='--', alpha=0.3)
        plt.axhline(y=MAX_US, color='k', linestyle='--', alpha=0.3)
        
        plt.title('Servo 1 (Pin D2) Command Stream')
        plt.ylabel('Pulse Width (microseconds)')
        plt.xlabel('Packet Count')
        plt.legend(loc='upper left')
        plt.grid(True)
        
        # Dynamic Zoom: Center the view around the current value +/- 50us
        # This lets you see the "micro-jitters" clearly
        plt.ylim(current_val - 50, current_val + 50)

def main():
    global node
    rclpy.init()
    node = VisualizerNode()

    # Run ROS in background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Setup Plot
    fig = plt.figure()
    ani = animation.FuncAnimation(fig, update_plot, interval=50) 
    plt.show()

    rclpy.shutdown()

if __name__ == '__main__':
    main()