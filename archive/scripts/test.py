#!/usr/bin/env python3
"""
Camera PNP V2.1 - Active Tracking Descent & Instant Drop
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraPNP_V2(Node):
    def __init__(self):
        super().__init__('camera_pnp_v2')

        # --- Publishers / Subscribers ---
        self.pose_pub = self.create_publisher(Pose, '/delta/target_pose', 10)
        self.suction_pub = self.create_publisher(Bool, '/suction/command', 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.speed_pub = self.create_publisher(Twist, '/delta/speed_params', 10)
        self.status_pub = self.create_publisher(String, '/pnp/status', 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.025, self.control_loop)  # 40Hz (0.025s)

        # --- Settings ---
        self.set_speed(0.2, 5.0) # 0.2 m/s linear, 5.0 rad/s angular

        # --- State Machine ---
        self.state = "SEARCH"
        self.state_timer = 0
        
        # --- Vision Data ---
        self.target_found = False
        self.box_center_px = (0, 0)
        self.box_area = 0
        self.current_image = None
        
        # --- Robot Position State ---
        self.pos = [0.0, 0.0, -0.25]  # Current Target: x, y, z
        self.suction_target = [0.0, 0.0, 0.0] # Snapshot of target before descent
        
        # --- Field Constants ---
        self.SAFE_Z = -0.25
        self.PRE_PICK_Z = -0.300 # Trigger suction here
        self.PICK_Z = -0.315     # Hard limit
        # Place position: Off the belt to +Y side (conveyor moves along X)
        self.PLACE_POS = [0.0, 0.190, -0.240]
        
        # --- Camera Params ---
        self.CAM_CENTER = (400, 400) # 800x800 resolution
        self.CENTER_TOL = 15         # pixels
        
        # Offsets
        self.CAMERA_OFFSET_X = 0.02
        self.CAMERA_OFFSET_Y = 0.0

        # --- Motion / Control Constants ---
        self.K_P = 0.012             # Proportional Gain
        self.STEP_XY = 0.0025        # Max step per tick during centering
        self.DESCENT_SPEED = 0.010   # Z-axis descent speed
        
        # --- Conveyor Belt Settings ---
        self.BELT_SPEED_X = -0.05    # Meters per second (Feedforward)

        # --- Stability ---
        self.centering_counter = 0
        self.MAX_CENTERING_ITERS = 400 

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.detect_object(self.current_image)
            self.display_image()
        except Exception as e:
            self.get_logger().error(f"CV Error: {e}")

    def detect_object(self, image):
        """Standard Red Object Detection"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 100, 100]), np.array([180, 255, 255]))
        mask = mask1 | mask2
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 300:
                self.target_found = True
                self.box_area = area
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    self.box_center_px = (cx, cy)
                return
        
        self.target_found = False
        self.box_area = 0

    def control_loop(self):
        """Main State Machine"""
        dt = 0.025 # Timer period

        if self.state == "SEARCH":
            self.pos = [0.0, 0.0, self.SAFE_Z] # Reset to hover
            self.centering_counter = 0
            self.publish_pose()
            
            if self.target_found:
                self.get_logger().info("Target found → CENTERING")
                self.state = "CENTERING"
                
        elif self.state == "CENTERING":
            if not self.target_found:
                self.state = "SEARCH" # Lost it, go back
                return
            
            self.centering_counter += 1
            if self.centering_counter > self.MAX_CENTERING_ITERS:
                 self.get_logger().warn("Centering Timeout!")
                 self.state = "SEARCH"
                 return

            # Calculate Error
            err_x = self.box_center_px[0] - self.CAM_CENTER[0]
            err_y = self.box_center_px[1] - self.CAM_CENTER[1]
            
            # Map Pixel Error to Robot XY (Assuming 90deg Cam Rotation)
            dx = -err_y * self.K_P
            dy = -err_x * self.K_P
            
            # Clamp Speed
            mag = np.hypot(dx, dy)
            if mag > self.STEP_XY:
                dx, dy = dx/mag * self.STEP_XY, dy/mag * self.STEP_XY
            
            self.pos[0] += dx
            self.pos[1] += dy
            self.publish_pose()
            
            # Check Tolerance
            if abs(err_x) < self.CENTER_TOL and abs(err_y) < self.CENTER_TOL:
                self.get_logger().info("Centered → ALIGN OFFSET")
                
                # Apply Camera-to-Suction Offset ONE TIME
                self.suction_target = [
                    self.pos[0] + self.CAMERA_OFFSET_X, 
                    self.pos[1] + self.CAMERA_OFFSET_Y, 
                    self.pos[2]
                ]
                self.state = "ALIGN_SUCTION"
                self.state_timer = 0

        elif self.state == "ALIGN_SUCTION":
            # Move to the calculated suction point
            reached = self.move_towards(self.suction_target, step_mult=1.0)
            self.publish_pose()
            
            if reached:
                 self.get_logger().info("Offset Applied → DESCEND")
                 self.state = "DESCEND"
                 self.state_timer = 0
                
        elif self.state == "DESCEND":
            # Simple descent: X at constant belt speed, Y locked, Z descends
            
            # 1. X: Constant belt speed (no tracking)
            BELT_SPEED_X = -0.00125  # m/tick (belt at -0.05 m/s, 40Hz)
            self.pos[0] += BELT_SPEED_X
            
            # 2. Z: Descend
            DESCENT_SPEED = 0.005
            self.pos[2] -= DESCENT_SPEED

            # --- 4. TRIGGERS ---
            # Pre-suction trigger
            if self.pos[2] <= self.PRE_PICK_Z:
                 if self.state_timer == 0:
                     self.get_logger().info("Suction ON (Pre-trigger)")
                     self.suction(True)
                     self.state_timer = 1 

            # Bottom hit
            if self.pos[2] <= self.PICK_Z:
                self.pos[2] = self.PICK_Z
                self.publish_pose()
                self.state = "PICK"
                self.state_timer = 0
            else:
                self.publish_pose()
                
        elif self.state == "PICK":
            # Short dwell to ensure seal
            self.state_timer += 1
            if self.state_timer > 10: # Reduced from 20 -> 10 for speed
                self.state = "LIFT"
                self.state_timer = 0
                
        elif self.state == "LIFT":
            target = [self.pos[0], self.pos[1], self.SAFE_Z]
            reached = self.move_towards(target, step_mult=2.5) # Fast Lift
            self.publish_pose()
            
            if reached:
                self.state = "PLACE"
                self.get_logger().info("Lifted. Moving to Place...")

        elif self.state == "PLACE":
             # Move to drop zone
             reached = self.move_towards(self.PLACE_POS, step_mult=2.5)
             self.publish_pose()
             
             if reached:
                 # --- INSTANT DROP ---
                 # No waiting. Immediately release.
                 self.get_logger().info("Arrived → DROPPING")
                 self.suction(False) # Valve off
                 
                 msg = String()
                 msg.data = "DROPPED"
                 self.status_pub.publish(msg)
                 
                 self.state = "RESET"
                 self.state_timer = 0
                
        elif self.state == "RESET":
             # Wait a tiny bit for air to release before flying back
             self.state_timer += 1
             if self.state_timer > 5: # 0.1s wait
                 self.pos = [0.0, 0.0, self.SAFE_Z]
                 self.publish_pose()
                 self.get_logger().info("Reset complete → SEARCH")
                 self.target_found = False
                 self.state = "SEARCH"
                 self.state_timer = 0

    def move_towards(self, target, step_mult=1.0):
        """Incremental move function."""
        done = True
        for i in range(3):
            diff = target[i] - self.pos[i]
            s = self.STEP_XY if i < 2 else 0.05 # Fast Z step
            s *= step_mult
            
            if abs(diff) > s:
                self.pos[i] += np.sign(diff) * s
                done = False
            else:
                self.pos[i] = target[i]
        return done

    def suction(self, on: bool):
        msg = Bool()
        msg.data = on
        # Burst publish to ensure message is received
        for _ in range(3):
            self.suction_pub.publish(msg)

    def set_speed(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.speed_pub.publish(msg)

    def publish_pose(self):
        msg = Pose()
        msg.position.x = float(self.pos[0])
        msg.position.y = float(self.pos[1])
        msg.position.z = float(self.pos[2])
        msg.orientation.w = 1.0
        self.pose_pub.publish(msg)

    def display_image(self):
        if self.current_image is None: return
        img = self.current_image.copy()
        h, w = img.shape[:2]
        
        cv2.line(img, (w//2, h//2), (w//2, h//2), (0, 255, 0), 5)
        
        if self.target_found:
            cx, cy = self.box_center_px
            cv2.circle(img, (cx, cy), 10, (0, 0, 255), -1)
            cv2.line(img, (w//2, h//2), (cx, cy), (255, 0, 0), 2)
        
        cv2.putText(img, f"{self.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.imshow("Cam View", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPNP_V2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()