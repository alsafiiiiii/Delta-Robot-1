#!/usr/bin/env python3
"""
Camera PNP V3 - Revamped XY Logic
- Simplified state machine
- Continuous visual tracking during descent
- Workspace boundary enforcement
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraPNP(Node):
    def __init__(self):
        super().__init__('camera_pnp')

        # Publishers / Subscribers
        self.pose_pub = self.create_publisher(Pose, '/delta/target_pose', 10)
        self.suction_pub = self.create_publisher(Bool, '/suction/command', 10)
        self.status_pub = self.create_publisher(String, '/pnp/status', 10)
        self.speed_pub = self.create_publisher(Twist, '/delta/speed_params', 10)
        self.image_pub = self.create_publisher(Image, '/camera/annotated', 10)  # Annotated output
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        # Set controller speed
        self.set_speed(0.3, 5.0)

        # === STATE ===
        self.state = "SEARCH"
        self.state_timer = 0
        self.target_found = False
        self.box_center_px = (0, 0)
        self.current_image = None
        
        # === ROBOT POSE ===
        self.pos = [0.0, 0.0, -0.25]
        
        # === CONSTANTS ===
        self.SAFE_Z = -0.25
        self.PRE_PICK_Z = -0.300  # Suction triggers here
        self.PICK_Z = -0.311      # Hard floor limit
        self.PLACE_POS = [0.0, 0.18, -0.242]
        
        # Camera (800x800)
        self.CAM_CENTER = (400, 400)
        self.CENTER_TOL = 20  # pixels - slightly larger tolerance
        
        # Camera offset (suction cup is 2cm behind camera in X)
        self.CAMERA_OFFSET_X = 0.01
        self.CAMERA_OFFSET_Y = 0.0

        # === XY MOTION PARAMS (PID Control) ===
        self.K_P = 0.00003       # Proportional gain
        self.K_I = 0.000001       # Integral gain (eliminates steady-state error)
        self.K_D = 0.000035      # Derivative gain (damping)
        self.MAX_XY_SPEED = 0.008 # Max XY velocity per tick
        self.STEP_Z = 0.05       # Descent speed (slower)
        
        # PID state
        self.prev_err_x = 0
        self.prev_err_y = 0
        self.integral_x = 0
        self.integral_y = 0
        self.INTEGRAL_MAX = 5000  # Anti-windup limit
        
        # === WORKSPACE LIMITS ===
        self.WS_X_MIN, self.WS_X_MAX = -0.12, 0.12
        self.WS_Y_MIN, self.WS_Y_MAX = -0.12, 0.12
        
        self.get_logger().info("Camera PNP V3 Started")

    # ==================== IMAGE PROCESSING ====================
    
    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.detect_object(self.current_image)
            self.display_image()
        except Exception as e:
            self.get_logger().error(f"CV Error: {e}")

    def detect_object(self, image):
        """Detect red object center."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Red mask
        mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 100, 100]), np.array([180, 255, 255]))
        mask = mask1 | mask2
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 300:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    self.box_center_px = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
                    self.target_found = True
                    return
        
        self.target_found = False

    def display_image(self):
        if self.current_image is None:
            return
            
        img = self.current_image.copy()
        h, w = img.shape[:2]
        
        # Crosshair
        cv2.line(img, (w//2-30, h//2), (w//2+30, h//2), (0,255,0), 2)
        cv2.line(img, (w//2, h//2-30), (w//2, h//2+30), (0,255,0), 2)
        
        # Target
        if self.target_found:
            cx, cy = self.box_center_px
            cv2.circle(img, (cx, cy), 10, (0,0,255), -1)
            cv2.line(img, (w//2, h//2), (cx, cy), (255,0,0), 2)
        
        # Info
        cv2.putText(img, f"State: {self.state}", (10, h-60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        cv2.putText(img, f"Pos: ({self.pos[0]:.3f}, {self.pos[1]:.3f}, {self.pos[2]:.3f})", 
                   (10, h-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        
        # Publish annotated image to ROS2 topic
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Image publish error: {e}")

    # ==================== XY TRACKING ====================
    
    def track_xy(self):
        """
        PID control for XY tracking.
        P = response speed, I = eliminates steady-state error, D = damping
        Returns True if centered within tolerance.
        """
        if not self.target_found:
            return False
        
        # Pixel error
        err_px_x = self.box_center_px[0] - self.CAM_CENTER[0]
        err_px_y = self.box_center_px[1] - self.CAM_CENTER[1]
        
        # Derivative (change in error)
        d_err_x = err_px_x - self.prev_err_x
        d_err_y = err_px_y - self.prev_err_y
        
        # Integral (accumulated error) with anti-windup
        self.integral_x += err_px_x
        self.integral_y += err_px_y
        self.integral_x = np.clip(self.integral_x, -self.INTEGRAL_MAX, self.INTEGRAL_MAX)
        self.integral_y = np.clip(self.integral_y, -self.INTEGRAL_MAX, self.INTEGRAL_MAX)
        
        # Store for next iteration
        self.prev_err_x = err_px_x
        self.prev_err_y = err_px_y
        
        # PID control: output = Kp*error + Ki*integral + Kd*derivative
        # Camera-to-Robot mapping: Camera X+ = Robot Y-, Camera Y+ = Robot X-
        control_x = -err_px_y * self.K_P - self.integral_y * self.K_I - d_err_y * self.K_D
        control_y = -err_px_x * self.K_P - self.integral_x * self.K_I - d_err_x * self.K_D
        
        # Clamp velocity
        speed = np.hypot(control_x, control_y)
        if speed > self.MAX_XY_SPEED:
            control_x = control_x / speed * self.MAX_XY_SPEED
            control_y = control_y / speed * self.MAX_XY_SPEED
        
        # Apply
        self.pos[0] += control_x
        self.pos[1] += control_y
        
        # Workspace limits
        self.pos[0] = np.clip(self.pos[0], self.WS_X_MIN, self.WS_X_MAX)
        self.pos[1] = np.clip(self.pos[1], self.WS_Y_MIN, self.WS_Y_MAX)
        
        # Reset integral when centered (prevent windup)
        if abs(err_px_x) < self.CENTER_TOL and abs(err_px_y) < self.CENTER_TOL:
            self.integral_x = 0
            self.integral_y = 0
            return True
        return False

    # ==================== STATE MACHINE ====================
    
    def control_loop(self):
        
        # ===== SEARCH =====
        if self.state == "SEARCH":
            self.pos = [0.0, 0.0, self.SAFE_Z]
            self.publish_pose()
            
            if self.target_found:
                self.get_logger().info("Target found → TRACK")
                self.state = "TRACK"
                self.state_timer = 0
        
        # ===== TRACK (XY Centering) =====
        elif self.state == "TRACK":
            if not self.target_found:
                self.state_timer += 1
                if self.state_timer > 20:  # Lost for 1 sec
                    self.get_logger().warn("Target lost → SEARCH")
                    self.state = "SEARCH"
                return
            
            self.state_timer = 0
            centered = self.track_xy()
            self.publish_pose()
            
            if centered:
                self.get_logger().info("Centered → DESCEND")
                self.state = "DESCEND"
                self.state_timer = 0
        
        # ===== DESCEND (with active tracking) =====
        elif self.state == "DESCEND":
            # Continue tracking while descending
            if self.target_found:
                self.track_xy()
            
            # Descend
            self.pos[2] -= self.STEP_Z
            
            # Trigger suction at PRE_PICK_Z
            if self.pos[2] <= self.PRE_PICK_Z and self.state_timer == 0:
                self.get_logger().info("Suction ON")
                self.suction(True)
                self.state_timer = 1
            
            # Hard limit
            if self.pos[2] <= self.PICK_Z:
                self.pos[2] = self.PICK_Z
                self.state = "PICK"
                self.state_timer = 0
            
            self.publish_pose()
        
        # ===== PICK (dwell) =====
        elif self.state == "PICK":
            self.state_timer += 1
            if self.state_timer > 15:
                self.state = "LIFT"
                self.state_timer = 0
        
        # ===== LIFT =====
        elif self.state == "LIFT":
            self.pos[2] += 0.01  # Fast lift
            if self.pos[2] >= self.SAFE_Z:
                self.pos[2] = self.SAFE_Z
                self.state = "PLACE"
            self.publish_pose()
        
        # ===== PLACE =====
        elif self.state == "PLACE":
            # Move towards place position
            reached = self.move_towards(self.PLACE_POS)
            self.publish_pose()
            
            if reached:
                self.state_timer += 1
                if self.state_timer > 15:
                    self.state = "DROP"
                    self.state_timer = 0
        
        # ===== DROP =====
        elif self.state == "DROP":
            if self.state_timer == 0:
                self.get_logger().info("Suction OFF")
                self.suction(False)
                msg = String()
                msg.data = "DROPPED"
                self.status_pub.publish(msg)
            
            self.state_timer += 1
            if self.state_timer > 10:
                self.state = "RESET"
                self.state_timer = 0
        
        # ===== RESET =====
        elif self.state == "RESET":
            self.pos = [0.0, 0.0, self.SAFE_Z]
            self.publish_pose()
            self.state_timer += 1
            if self.state_timer > 20:
                self.get_logger().info("Reset complete → SEARCH")
                self.target_found = False
                self.state = "SEARCH"
                self.state_timer = 0

    # ==================== HELPERS ====================
    
    def move_towards(self, target, speed=0.005):
        """Move incrementally towards target. Returns True when reached."""
        done = True
        for i in range(3):
            diff = target[i] - self.pos[i]
            if abs(diff) > speed:
                self.pos[i] += np.sign(diff) * speed
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
    node = CameraPNP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()