#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray
import socket
import struct
import threading
import time
import numpy as np

# Configuration
# Configuration
RX_BUFFER_SIZE = 1024

class DeltaBridge(Node):
    def __init__(self):
        super().__init__('delta_bridge')
        
        # --- ROS2 Interfaces ---
        self.declare_parameter('esp_ip', "10.248.215.11")
        self.declare_parameter('esp_port', 3333)
        
        self.target_ip = self.get_parameter('esp_ip').value
        self.target_port = self.get_parameter('esp_port').value
        self.esp_addr = (self.target_ip, self.target_port)

        self.sub_traj = self.create_subscription(
            JointTrajectory, 
            '/model/delta_robot/joint_trajectory', 
            self.trajectory_callback, 
            10
        )
        
        # Feedback Publishers
        self.pub_status = self.create_publisher(Bool, '/delta/bridge/connected', 1)
        self.pub_joints = self.create_publisher(JointState, '/delta/real_joints', 10)
        
        # --- UDP Socket ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1) # Non-blocking for receive loop
        
        # State
        self.last_feedback_time = 0
        self.connected = False
        self.running = True
        
        # Start Receiver Thread
        self.rx_thread = threading.Thread(target=self.receive_loop)
        self.rx_thread.start()
        
        self.get_logger().info(f"Bridge Started. Target: {self.target_ip}:{self.target_port}")

    def trajectory_callback(self, msg):
        if not msg.points: return
        
        try:
            # Extract Angles (Radians -> Degrees)
            rads = msg.points[0].positions
            # Expecting at least 3 angles for the main arms
            if len(rads) < 3: return
            
            angles_deg = np.rad2deg(rads[:5]) # Take up to 5 axes
            
            # Fill remaining with 0 if less than 5
            payload = list(angles_deg)
            while len(payload) < 5: payload.append(0.0)
            
            # Clip safe limits (0-180 for standard servos)
            # Adjust these limits based on your specific Delta Robot mechanics!
            payload = np.clip(payload, 0, 180)
            
            # Pack: 5 floats (Little Endian)
            packet = struct.pack('<fffff', *payload)
            self.sock.sendto(packet, self.esp_addr)
            
            # Debug Log (Throttled)
            self.get_logger().info(f"TX -> {self.target_ip}: {payload}", throttle_duration_sec=2.0)
            
        except Exception as e:
            self.get_logger().error(f"TX Error: {e}")

    def receive_loop(self):
        self.get_logger().info("UDP Receiver Thread Started")
        
        while self.running and rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(RX_BUFFER_SIZE)
                
                # Check source
                if addr[0] != self.target_ip: continue
                
                # Parse Feedback: 5 floats + 1 uint32 (timestamp)
                # Matches `delta_feedback_t` in main5.c
                expected_len = 24 # 5*4 + 4
                if len(data) == expected_len:
                    unpacked = struct.unpack('<fffffI', data)
                    angles = unpacked[:5]
                    esp_time = unpacked[5]
                    
                    self.handle_feedback(angles, esp_time)
                    
            except socket.timeout:
                # Normal timeout, check connection status
                if time.time() - self.last_feedback_time > 1.0:
                    if self.connected:
                        self.get_logger().warn("Connection Lost (Timeout)")
                        self.connected = False
                        self.pub_status.publish(Bool(data=False))
                continue
            except Exception as e:
                self.get_logger().error(f"RX Error: {e}")
                time.sleep(0.1)

    def handle_feedback(self, angles, timestamp):
        # Update alive status
        self.last_feedback_time = time.time()
        if not self.connected:
            self.connected = True
            self.get_logger().info("Connection Established (Feedback Received)")
            self.pub_status.publish(Bool(data=True))
            
        # Publish Joint State
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['jbf1', 'jbf2', 'jbf3', 'Bevelj1', 'Bevelj2']
        # Convert back to radians for ROS
        msg.position = np.deg2rad(angles).tolist()
        self.pub_joints.publish(msg)

    def destroy_node(self):
        self.running = False
        self.rx_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DeltaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
