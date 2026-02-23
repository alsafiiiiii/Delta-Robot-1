#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32MultiArray
import serial
import math
import time
import re

class EspSerialBridge(Node):
    def __init__(self):
        super().__init__('esp_serial_bridge')
        
        # ======================================================================
        # --- 1. CONFIGURATION ---
        # ======================================================================
        self.serial_port = '/dev/ttyUSB0' 
        self.baud_rate = 115200
        
        # Robot Physical Limits
        self.MIN_DEG = 0.0
        self.MAX_DEG = 180.0
        # ======================================================================

        # Setup Hardware
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.01)
            self.get_logger().info(f"Connected to {self.serial_port}")
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            exit(1)

        # Regex for "D0:12.3 D1:45.6 D2:78.9"
        self.sensor_pattern = re.compile(r"D(\d):([\d\.]+)")

        # ROS Interface
        self.sub_traj = self.create_subscription(
            JointTrajectory, 
            '/delta/joint_commands', 
            self.traj_callback, 
            10
        )
        
        # Publishes [Dist0, Dist1, Dist2]
        self.pub_sensors = self.create_publisher(Float32MultiArray, '/sharp_sensors/all', 10)
        
        self.get_logger().info("Serial Bridge node started (TF logic removed)")

    # ==========================================================================
    # --- 2. SERIAL PROCESSING ---
    # ==========================================================================
    def process_serial_data(self):
        """Reads sensor data from ESP32 and publishes it to ROS"""
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Check for format: "D0:150.2 D1:120.4 D2:60.1"
                if "D0:" in line:
                    matches = self.sensor_pattern.findall(line)
                    
                    if len(matches) == 3:
                        # Sort by index (0, 1, 2)
                        matches.sort(key=lambda x: int(x[0]))
                        
                        msg = Float32MultiArray()
                        msg.data = [float(val) for _, val in matches]
                        self.pub_sensors.publish(msg)
                        
            except Exception as e:
                self.get_logger().warn(f"Serial Parse Error: {e}")

    def wait_and_listen(self, duration_sec):
        """Keeps the serial buffer clear while waiting for trajectory steps"""
        start_time = time.time()
        while (time.time() - start_time) < duration_sec:
            self.process_serial_data()
            time.sleep(0.001) 

    # ==========================================================================
    # --- 3. TRAJECTORY CALLBACK ---
    # ==========================================================================
    def traj_callback(self, msg):
        if not msg.points: return
        
        for i, point in enumerate(msg.points):
            # Calculate duration for this step
            t_abs_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            
            if i == 0: 
                dt = t_abs_sec
            else:
                prev = msg.points[i-1].time_from_start
                dt = t_abs_sec - (prev.sec + prev.nanosec * 1e-9)
            
            duration_ms = max(20, int(dt * 1000))
            
            if len(point.positions) < 3: continue
            
            # Extract and Constrain Angles
            degs = [math.degrees(p) for p in point.positions[:3]]
            degs = [max(self.MIN_DEG, min(self.MAX_DEG, d)) for d in degs]
            
            # SEND COMMAND TO ESP32
            # Format: T0:90.00 D:200\n
            raw_degs = [math.degrees(p) for p in point.positions[:3]]
        
            # --- CALIBRATION FIX ---
            # If the robot moves "Short", it means the physical arms are too low.
            # We need to shift the zero point so the code matches reality.
            # Try adding/subtracting 5 or 10 degrees.
            
            OFFSET = 05.0  # <--- ADJUST THIS. Try -10.0 or +10.0
            
            corrected_degs = [d + OFFSET for d in raw_degs]
            
            # Constrain
            final_degs = [max(self.MIN_DEG, min(self.MAX_DEG, d)) for d in corrected_degs]
            
            # SEND COMMAND
            for idx, deg in enumerate(final_degs):
                cmd = f"T{idx}:{deg:.2f} D:{duration_ms}\n"
                self.ser.write(cmd.encode('utf-8'))
            
            # Process incoming sensor data during the motion duration
            self.wait_and_listen(dt)

def main(args=None):
    rclpy.init(args=args)
    node = EspSerialBridge()
    try:
        while rclpy.ok():
            # Check for new ROS messages
            rclpy.spin_once(node, timeout_sec=0.01)
            # Continually poll serial even if no new trajectory is received
            node.process_serial_data()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()