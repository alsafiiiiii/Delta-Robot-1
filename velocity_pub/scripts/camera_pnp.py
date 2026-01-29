#!/usr/bin/env python3
"""
Camera PNP V2 - Improved Pick and Place with OpenCV Visualization
Fixed: "Runaway Target" bug and State Timer initialization.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class CameraPNP_V2(Node):
    def __init__(self):
        super().__init__('camera_pnp_v2')

        # Publishers / Subscribers
        self.pose_pub = self.create_publisher(Pose, '/delta/target_pose', 10)
        self.suction_pub = self.create_publisher(Bool, '/suction/command', 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.speed_pub = self.create_publisher(Twist, '/delta/speed_params', 10)
        self.status_pub = self.create_publisher(String, '/pnp/status', 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        # Set fast speed for controller
        self.set_speed(0.2, 5.0) # 0.2 m/s linear, 5.0 rad/s angular

        # State Variables
        self.state = "SEARCH"
        self.state_timer = 0  # <--- FIXED: Added initialization
        self.target_found = False
        self.box_center_px = (0, 0)
        self.box_area = 0
        self.current_image = None
        
        # Robot pose
        self.pos = [0.0, 0.0, -0.25]  # x, y, z
        self.suction_target = [0.0, 0.0, 0.0] # <--- FIXED: Storage for locked target
        
        # Constants
        self.SAFE_Z = -0.25
        self.PRE_PICK_Z = -0.309 # Trigger suction here
        self.PICK_Z = -0.313     # Hard limit
        self.PLACE_POS = [0.15, 0.0, -0.20]
        
        # Camera params (800x800)
        self.CAM_CENTER = (400, 400)
        self.CENTER_TOL = 15  # pixels
        
        # Camera Offset (Distance from Suction Cup to Camera Lens)
        # If Camera is +X relative to Suction, we must move Robot +X to bring Suction to object.
        self.CAMERA_OFFSET_X = 0.02
        self.CAMERA_OFFSET_Y = 0.0

        # Motion params
        self.K_P = 0.010  
        self.STEP_XY = 0.002 
        self.STEP_Z = 0.1
        
        # Stability / Timeout
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
        """Detect red object and find its center."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Red color (wraps around 0/180)
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

    def display_image(self):
        """Display annotated camera view."""
        if self.current_image is None:
            return
            
        img = self.current_image.copy()
        h, w = img.shape[:2]
        
        # Draw crosshair at center
        cv2.line(img, (w//2 - 30, h//2), (w//2 + 30, h//2), (0, 255, 0), 2)
        cv2.line(img, (w//2, h//2 - 30), (w//2, h//2 + 30), (0, 255, 0), 2)
        
        # Draw target if found
        if self.target_found:
            cx, cy = self.box_center_px
            cv2.circle(img, (cx, cy), 10, (0, 0, 255), -1)
            cv2.line(img, (w//2, h//2), (cx, cy), (255, 0, 0), 2)
        
        # State display
        color = (0, 255, 0) if self.target_found else (0, 0, 255)
        cv2.putText(img, f"State: {self.state}", (10, h - 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(img, f"Pos: ({self.pos[0]:.3f}, {self.pos[1]:.3f}, {self.pos[2]:.3f})", 
                   (10, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow("Camera PNP V2", img)
        cv2.waitKey(1)

    def control_loop(self):
        """State machine control."""
        
        if self.state == "SEARCH":
            self.pos[2] = self.SAFE_Z
            self.centering_counter = 0
            self.publish_pose()
            
            if self.target_found:
                self.get_logger().info("Target found → CENTERING")
                self.state = "CENTERING"
                
        elif self.state == "CENTERING":
            if not self.target_found:
                self.state = "SEARCH"
                return
            
            self.centering_counter += 1
            if self.centering_counter > self.MAX_CENTERING_ITERS:
                 self.get_logger().warn("Centering Timeout! Resetting search.")
                 self.state = "SEARCH"
                 return

            err_x = self.box_center_px[0] - self.CAM_CENTER[0]
            err_y = self.box_center_px[1] - self.CAM_CENTER[1]
            
            # Map Pixels to Robot Motion (Camera 90deg rotated relative to Base)
            dx = -err_y * self.K_P
            dy = -err_x * self.K_P
            
            # Clamp Speed
            mag = np.hypot(dx, dy)
            if mag > self.STEP_XY:
                dx, dy = dx/mag * self.STEP_XY, dy/mag * self.STEP_XY
            
            self.pos[0] += dx
            self.pos[1] += dy
            self.publish_pose()
            
            if abs(err_x) < self.CENTER_TOL and abs(err_y) < self.CENTER_TOL:
                self.get_logger().info("Camera Centered → ALIGN SUCTION")
                
                # <--- FIXED: Lock the target ONCE here.
                # If we calculated this inside the ALIGN_SUCTION loop, 
                # self.pos would change and the target would run away infinitely.
                self.suction_target = [
                    self.pos[0] + self.CAMERA_OFFSET_X, 
                    self.pos[1] + self.CAMERA_OFFSET_Y, 
                    self.pos[2]
                ]
                self.state = "ALIGN_SUCTION"
                self.state_timer = 0

        elif self.state == "ALIGN_SUCTION":
            # Move to the locked target using visual servo loop logic (slow but accurate)
            # OR switch to direct open-loop command here if we trust the IK?
            # User wants speed. Let's use servo move for this short valid alignment,
            # but for long moves (LIFT/PLACE) we jump.
            
            reached = self.move_towards(self.suction_target, step_mult=1.0)
            self.publish_pose()
            
            self.state_timer += 1
            if reached:
                 self.get_logger().info(f"Suction Aligned (Offset applied) → DESCEND")
                 self.state = "DESCEND"
                 self.state_timer = 0
                
        elif self.state == "DESCEND":
            # Blind descent
            self.pos[2] -= self.STEP_Z
            
            # 1. Trigger suction just before hitting bottom
            if self.pos[2] <= self.PRE_PICK_Z:
                 if self.state_timer == 0:
                     self.get_logger().info("Suction ON (Pre-contact)")
                     self.suction(True)
                     self.state_timer = 1 

            # 2. Reached bottom limit
            if self.pos[2] <= self.PICK_Z:
                self.pos[2] = self.PICK_Z # Clamp
                self.publish_pose()
                self.state = "PICK"
                self.state_timer = 0
            else:
                self.publish_pose()
                
        elif self.state == "PICK":
            self.state_timer += 1
            # Wait a moment for suction to seal
            if self.state_timer > 20: 
                self.state = "LIFT"
                self.state_timer = 0
                
        # --- FAST OPEN LOOP SECTION ---
        # Instead of incremental steps, we send the final target and wait.
                
        elif self.state == "LIFT":
            # Smooth Lift to Safe Z
            target = [self.pos[0], self.pos[1], self.SAFE_Z]
            reached = self.move_towards(target, step_mult=2.0) # 2x speed for lift
            self.publish_pose()
            
            if reached:
                self.state = "PLACE"
                self.get_logger().info("Lifted. Moving to Place...")

        elif self.state == "PLACE":
             # Smooth Move to Place
             reached = self.move_towards(self.PLACE_POS, step_mult=2.0)
             self.publish_pose()
             
             if reached:
                 # Wait for controller lag to catch up
                 self.state_timer += 1
                 if self.state_timer > 20: # 1.0s wait
                     self.state = "DROP"
                     self.get_logger().info("Reached Destination (Stabilized) → DROP")
                     self.state_timer = 0
                
        elif self.state == "DROP":
            self.state_timer += 1
            if self.state_timer == 1:
                self.get_logger().info("Suction OFF")
                self.suction(False)
                
                # Report Drop
                msg = String()
                msg.data = "DROPPED"
                self.status_pub.publish(msg)
                
            if self.state_timer > 10:
                self.state = "RESET"
                self.state_timer = 0
                
        elif self.state == "RESET":
             if self.state_timer == 0:
                 self.pos = [0.0, 0.0, self.SAFE_Z] # Jump to Home
                 self.publish_pose()
                 self.state_timer += 1
                 self.get_logger().info("Fast Return...")
                 
             elif self.state_timer > 20:
                 self.get_logger().info("Reset complete → SEARCH")
                 self.target_found = False
                 self.state = "SEARCH"
                 self.state_timer = 0
             else:
                 self.state_timer += 1

    def move_towards(self, target, step_mult=1.0):
        """Incremental move (for visual servoing phases)."""
        done = True
        for i in range(3):
            diff = target[i] - self.pos[i]
            
            # Base steps
            s = self.STEP_XY if i < 2 else self.STEP_Z
            s *= step_mult
            
            if abs(diff) > s:
                 # Clamp step to avoid overshooting
                self.pos[i] += np.sign(diff) * s
                done = False
            else:
                self.pos[i] = target[i]
        return done

    def suction(self, on: bool):
        msg = Bool()
        msg.data = on
        for _ in range(5):
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

def main(args=None):
    rclpy.init(args=args)
    node = CameraPNP_V2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()