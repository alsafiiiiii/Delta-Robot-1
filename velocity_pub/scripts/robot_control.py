#!/usr/bin/env python3
import sys
import socket
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

# --- CONFIGURATION ---
ESP_IP = "10.114.194.11"   # TARGET ESP32 IP
ESP_PORT = 3333
# ---------------------

class DeltaHardwareBridge(Node):
    def __init__(self):
        super().__init__('delta_hardware_bridge')
        
        # 1. UDP Socket Setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.esp_addr = (ESP_IP, ESP_PORT)
        
        # 2. ROS 2 Subscriber
        # Listens for calculated joint angles (radians)
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/model/delta_robot/joint_trajectory',
            self.listener_callback,
            10
        )
        
        self.get_logger().info(f" BRIDGE STARTED: Listening on /model/delta_robot/joint_trajectory")
        self.get_logger().info(f" TARGET: {ESP_IP}:{ESP_PORT}")

    def listener_callback(self, msg):
        """
        Callback when a trajectory message is received.
        Extracts the first point and sends it immediately to ESP32.
        """
        if not msg.points:
            return

        try:
            # We take the first point in the trajectory for immediate execution
            # ROS uses Radians. We need to convert to Servo Degrees.
            rads = msg.points[0].positions
            
            if len(rads) < 3:
                return

            j1_deg = np.rad2deg(rads[0]) + 180.0
            j2_deg = np.rad2deg(rads[1]) + 180.0
            j3_deg = np.rad2deg(rads[2]) + 180.0

            # Send to hardware
            self.send_udp([j1_deg, j2_deg, j3_deg])

        except Exception as e:
            self.get_logger().error(f"Callback Error: {e}")

    def send_udp(self, angles):
        """ Low-latency UDP packet sender """
        try:
            # 1. Clamp values for safety (0 to 180 degrees)
            angles = np.clip(angles, 0, 180)
            
            # 2. Format Packet: "180.00,180.00,180.00"
            packet = f"{angles[0]:.2f},{angles[1]:.2f},{angles[2]:.2f}"
            
            # 3. Fire and Forget
            self.sock.sendto(packet.encode(), self.esp_addr)
            
            # Optional: Debug log (Comment out for max speed)
            # self.get_logger().info(f"Sent: {packet}")
            
        except Exception as e:
            self.get_logger().error(f"UDP Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    node = DeltaHardwareBridge()
    
    try:
        # Spin keeps the script alive and listening
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()