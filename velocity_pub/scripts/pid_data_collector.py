#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import csv
import time
import os

class PIDLogger(Node):
    def __init__(self):
        super().__init__('pid_logger')
        
        self.declare_parameter('filename', 'pid_data.csv')
        self.filename = self.get_parameter('filename').get_parameter_value().string_value
        
        self.sub = self.create_subscription(
            Float32MultiArray, 
            '/pnp/debug', 
            self.debug_callback, 
            10
        )
        
        # Open file and write header
        # Using unbuffered write or flush to ensure data is saved if killed
        self.csv_file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['Time', 'Input', 'Output', 'Velocity'])
        
        self.start_time = None
        self.get_logger().info(f"Logging PID data to {self.filename}...")
        self.get_logger().info("Cols: Time, Input(Setpoint), Output(Actual), Velocity(Effort)")

    def debug_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        
        # MSG: [Initial_Error, Response_Dist, Velocity_Y]
        setpoint = msg.data[0] # Initial Step Input
        response = msg.data[1] # Distance Moved (Output)
        velocity = msg.data[2] # Control Effort (Optional)
        
        self.writer.writerow([f"{current_time:.4f}", f"{setpoint:.5f}", f"{response:.5f}", f"{velocity:.4f}"])
        self.csv_file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = PIDLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
