#!/usr/bin/env python3
import socket
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

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
            Point,
            '/delta/reference_point',
            self.listener_callback,
            10
        )
        self.get_logger().info(f"✅ Bridge Ready. Target: {ESP_IP}:{ESP_PORT}")

    def listener_callback(self, msg):
        try:
            # Prepare (X, Y, Z) packet
            packet = struct.pack('<fff', msg.x, msg.y, msg.z)
            self.sock.sendto(packet, self.esp_addr)
            
            # --- LOGGING ---
            # self.get_logger().info(f"TX: {msg.x:.4f} | {msg.y:.4f} | {msg.z:.4f}") 
            # (Commented out to reduce spam, uncomment for debug)


        except Exception as e:
            self.get_logger().error(f"Bridge Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DeltaHardwareBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()