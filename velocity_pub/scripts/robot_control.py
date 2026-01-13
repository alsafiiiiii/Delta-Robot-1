#!/usr/bin/env python3
import socket
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time

# --- CONFIGURATION ---
ESP_IP = "10.38.19.11"
ESP_PORT = 3333
TARGET_RATE = 50.0  # Hz - MUST match ESP32

class DeltaHardwareBridge(Node):
    def __init__(self):
        super().__init__('delta_hardware_bridge')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.esp_addr = (ESP_IP, ESP_PORT)
        
        self.packet_count = 0
        self.dt = 1.0 / TARGET_RATE
        self.last_point = Point()
        self.last_point.x = 0.0
        self.last_point.y = 0.0
        self.last_point.z = -0.22
        
        self.subscription = self.create_subscription(
            Point,
            '/delta/reference_point',
            self.listener_callback,
            10
        )
        
        # Timer to enforce 50Hz output (even if ROS messages are slower)
        self.timer = self.create_timer(self.dt, self.send_packet)
        
        self.get_logger().info(f"✅ Bridge Ready. Target: {ESP_IP}:{ESP_PORT} @ {TARGET_RATE}Hz")

    def listener_callback(self, msg):
        # Update latest point whenever ROS publishes
        self.last_point = msg

    def send_packet(self):
        """Sends packet at guaranteed 50Hz rate using last received point"""
        try:
            # Prepare (X, Y, Z, Mode) packet
            # Mode 0 = Linear Interpolation (Cartesian)
            packet = struct.pack('<fffB', 
                                 self.last_point.x, 
                                 self.last_point.y, 
                                 self.last_point.z, 
                                 0)  # Mode 0 = Linear
            self.sock.sendto(packet, self.esp_addr)
            self.packet_count += 1
            
        except Exception as e:
            self.get_logger().error(f"Send Error: {e}")

        # Rate Monitor (every 2 seconds)
        now = self.get_clock().now()
        if not hasattr(self, 'last_log_time'):
            self.last_log_time = now
        
        diff = (now - self.last_log_time).nanoseconds / 1e9
        if diff >= 2.0:
            rate = self.packet_count / diff
            self.get_logger().info(
                f"TX Rate: {rate:.1f} Hz | "
                f"Target: ({self.last_point.x:.3f}, {self.last_point.y:.3f}, {self.last_point.z:.3f})"
            )
            self.packet_count = 0
            self.last_log_time = now

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DeltaHardwareBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()