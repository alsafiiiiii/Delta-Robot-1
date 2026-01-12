#!/usr/bin/env python3
import socket
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

# --- CONFIGURATION ---
ESP_IP = "10.248.215.11"
ESP_PORT = 3333

class DeltaHardwareBridge(Node):
    def __init__(self):
        super().__init__('delta_hardware_bridge')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.esp_addr = (ESP_IP, ESP_PORT)
        
        self.packet_count = 0 

        self.subscription = self.create_subscription(
            JointTrajectory,
            '/model/delta_robot/joint_trajectory',
            self.listener_callback,
            10
        )
        self.get_logger().info(f"✅ Bridge Ready. Target: {ESP_IP}:{ESP_PORT}")

    def listener_callback(self, msg):
        if not msg.points: return
        try:
            rads = msg.points[0].positions
            angles_deg = np.rad2deg(rads[:5])
            angles_deg = np.clip(angles_deg, 0, 180)

            packet = struct.pack('<fffff', *angles_deg)
            self.sock.sendto(packet, self.esp_addr)
            
            # --- LOGGING EVERY PACKET ---
            # This lets you see the exact angles being sent in real-time
            self.get_logger().info(
                f"TX: {angles_deg[0]:6.2f} | {angles_deg[1]:6.2f} | {angles_deg[2]:6.2f}"
            )

        except Exception as e:
            self.get_logger().error(f"Bridge Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DeltaHardwareBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()