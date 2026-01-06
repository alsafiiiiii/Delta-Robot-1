#!/usr/bin/env python3
"""
Camera-based Pick and Place Node for Delta Robot
Uses a simple State Machine to locate, approach, pick, and place a red box.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class CameraPNP(Node):
    def __init__(self):
        super().__init__('camera_pnp')

        # --- Publishers / Subscribers ---
        self.pose_pub = self.create_publisher(Pose, '/delta/target_pose', 10)
        self.suction_pub = self.create_publisher(Bool, '/suction/command', 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # --- Tools ---
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz Control Loop

        # --- State Variables ---
        self.state = "SEARCH" # SEARCH, APPROACH, CENTERING, DESCEND, PICK, LIFT, PLACE, DROP, DONE
        self.target_found = False
        self.box_center_x = 0
        self.box_center_y = 0
        
        # Robot Current Target State (Local Coordinates)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = -0.25 # Safe height

        # --- Constants ---
        self.SAFE_Z = -0.25
        self.PICK_Z = -0.38 # Approximate height of box top relative to base (0.5 - 0.125ish)
        self.PLACE_LOCATION = [0.15, 0.0, -0.30] # Place at edge
        
        self.cam_width = 800
        self.cam_height = 800
        self.center_threshold = 20 # pixels

        self.get_logger().info("Camera PNP Node Started")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def process_image(self, image):
        # 1. Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 2. Threshold for RED color
        # Red wraps around 180, so we need two ranges
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # 3. Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500: # Filter small noise
                self.target_found = True
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    self.box_center_x = cX
                    self.box_center_y = cY
            else:
                self.target_found = False
        else:
            self.target_found = False

    def control_loop(self):
        # State Machine
        
        if self.state == "SEARCH":
            # Just go to Home/Safe Z
            self.current_z = self.SAFE_Z
            self.publish_pose(self.current_x, self.current_y, self.current_z)
            
            if self.target_found:
                self.get_logger().info("Target Found! Switching to CENTERING")
                self.state = "CENTERING"

        elif self.state == "CENTERING":
            if not self.target_found:
                self.state = "SEARCH"
                return

            # Visual Servoing Logic
            # Image Center is (400, 400)
            err_x = self.box_center_x - (self.cam_width / 2)
            err_y = self.box_center_y - (self.cam_height / 2)
            
            # Simple P-Controller
            k_p = 0.0002 
            
            # Note: Camera frame vs Robot frame mapping needs verification.
            # Usually Camera X is Robot Y (or -Y) and Camera Y is Robot X.
            # Assuming standard camera orientation:
            # Camera X+ (Right) -> Robot Y- (Right?) NO.
            # Let's assume:
            # - If box is to right of image (positive err_x), we need to move robot right.
            # - If box is below image center (positive err_y), we need to move robot BACK or FWD?
            
            # Let's align naively first. 
            # If standard top down:
            # Robot X is forward/backward?
            # Adjust mapping if specific orientation known. 
            # Based on model.sdf: <pose relative_to="camera_bar">-0.02 0 0 0 1.5708 0</pose>
            # Rotated 90 deg Pitch.
            # It's looking down.
            
            # Let's assume 1-1 mapping for now and correct signs if it runs away
            # Using -Y for Camera X because image x axis usually right, robot Y axis typically left?
            
            dx = -err_y * k_p # Image Y is typically Robot X axis (Forward/Back)
            dy = -err_x * k_p # Image X is typically Robot Y axis (Left/Right)
            
            self.current_x += dx
            self.current_y += dy
            
            self.publish_pose(self.current_x, self.current_y, self.current_z)
            
            if abs(err_x) < self.center_threshold and abs(err_y) < self.center_threshold:
                self.get_logger().info("Centered. Descending...")
                self.state = "DESCEND"

        elif self.state == "DESCEND":
            self.current_z -= 0.005 # Descend slowly
            
            # Continue Centering while descending
            if self.target_found:
                 err_x = self.box_center_x - (self.cam_width / 2)
                 err_y = self.box_center_y - (self.cam_height / 2)
                 k_p = 0.0002
                 self.current_x += (-err_y * k_p)
                 self.current_y += (-err_x * k_p)
            
            self.publish_pose(self.current_x, self.current_y, self.current_z)

            if self.current_z <= self.PICK_Z:
                self.state = "PICK"

        elif self.state == "PICK":
            self.get_logger().info("Attempting Suction...")
            msg = Bool()
            msg.data = True
            for _ in range(10): # Publish multiple times to ensure receipt
                self.suction_pub.publish(msg)
                time.sleep(0.05)
            
            time.sleep(1.0) # Wait for attach
            self.state = "LIFT"

        elif self.state == "LIFT":
            self.current_z += 0.005
            self.publish_pose(self.current_x, self.current_y, self.current_z)
            
            if self.current_z >= self.SAFE_Z:
                self.state = "PLACE"

        elif self.state == "PLACE":
            self.current_x = self.PLACE_LOCATION[0]
            self.current_y = self.PLACE_LOCATION[1]
            # self.current_z = self.PLACE_LOCATION[2] # Keep high for now?
            
            self.publish_pose(self.current_x, self.current_y, self.current_z)
            
            # Check distance (simple hack, just wait or check coords)
            # We assume the controller handles the interpolation
            # We just wait a bit
            time.sleep(3.0) 
            self.state = "DROP"

        elif self.state == "DROP":
            self.get_logger().info("Dropping object...")
            msg = Bool()
            msg.data = False
            for _ in range(10):
                self.suction_pub.publish(msg)
            time.sleep(1.0)
            self.state = "DONE"
            
        elif self.state == "DONE":
             self.get_logger().info("Mission Complete.")
             # Optionally reset or exit
             # self.destroy_node()

    def publish_pose(self, x, y, z):
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        
        # Keep orientation flat
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPNP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
