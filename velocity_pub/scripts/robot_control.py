#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import serial
import math

class EspSerialBridge(Node):
    def __init__(self):
        super().__init__('esp_serial_bridge')
        
        self.serial_port = '/dev/ttyUSB0' 
        self.baud_rate = 115200
        
        # --- CALIBRATION ---
        self.MIN_US = 550
        self.MAX_US = 2400
        self.MIN_DEG = 0
        self.MAX_DEG = 180
        self.OFFSET_DEG = 0.0 
        
        # Change detection threshold (8-bit encoder = ~7µs resolution)
        self.CHANGE_THRESHOLD = 0.5  # Send almost every update (0.5µs precision)
        self.last_sent = [0.0, 0.0, 0.0]

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"Connected to {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            exit(1)

        self.sub = self.create_subscription(
            JointTrajectory, 
            '/model/delta_robot/joint_trajectory', 
            self.traj_callback, 
            10)

    def map_deg_to_us(self, deg):
        # 1. Apply Offset
        deg += self.OFFSET_DEG
        
        # 2. Clamp to physical limits (Safety)
        deg = max(self.MIN_DEG, min(self.MAX_DEG, deg))
        
        # 3. Linear Map
        # (val - min_in) * (max_out - min_out) / (max_in - min_in) + min_out
        us = (deg - self.MIN_DEG) * (self.MAX_US - self.MIN_US) / (self.MAX_DEG - self.MIN_DEG) + self.MIN_US
        return us

    def traj_callback(self, msg):
        if msg.points:
            # 1. Get Radians
            rads = msg.points[0].positions[0:3]
            duration = msg.points[0].time_from_start
            
            # Duration in Milliseconds
            duration_ms = int(duration.sec * 1000 + duration.nanosec / 1e6)
            if duration_ms < 20: duration_ms = 20 # Safety floor
            
            # 2. Convert to Degrees
            deg1 = math.degrees(rads[0])
            deg2 = math.degrees(rads[1])
            deg3 = math.degrees(rads[2])
            
            # 3. Apply Calibration/Offset if needed (Currently handled by FW or IK)
            # Just clamping for safety before sending
            deg1 = max(self.MIN_DEG, min(self.MAX_DEG, deg1))
            deg2 = max(self.MIN_DEG, min(self.MAX_DEG, deg2))
            deg3 = max(self.MIN_DEG, min(self.MAX_DEG, deg3))
            
            # 4. Formulate Interpolation Commands
            # Protocol: T<idx>:<deg> D:<ms>
            
            cmds = [
                f"T0:{deg1:.2f} D:{duration_ms}\n",
                f"T1:{deg2:.2f} D:{duration_ms}\n",
                f"T2:{deg3:.2f} D:{duration_ms}\n"
            ]
            
            try:
                for cmd in cmds:
                    self.ser.write(cmd.encode('utf-8'))
                    # self.get_logger().info(f"Sent: {cmd.strip()}")
            except Exception as e:
                self.get_logger().error(f"Write Failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EspSerialBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()