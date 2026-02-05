#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from dataclasses import dataclass

# ==============================================================================
#                               USER CONFIGURATION
# ==============================================================================
@dataclass
class PNPConfig:
    """Centralized configuration for Pick and Place logic."""
    
    # --- 1. TIMINGS (Seconds) ---
    descend_time:  float = 0.3
    pick_dwell:    float = 0.1   # Time to wait while suction engages
    lift_time:     float = 0.3
    place_travel:  float = 1.0   # Time allowed to move to place zone
    drop_dwell:    float = 0.1   # Time to wait after dropping
    return_travel: float = 0.80   # Time allowed to return to search

    # --- 2. PHYSICAL POSITIONS (Meters) ---
    safe_z:    float = -0.240    # Cruising height
    pick_z:    float = -0.313    # Height to touch the object
    place_y:   float =  0.180    # Y-coordinate for drop-off
    
    # --- 3. SAFETY LIMITS ---
    y_limit_min: float = -0.18
    y_limit_max: float =  0.18
    
    # --- 4. VISION & CONTROL ---
    pixels_per_meter: float = 9500.0
    cam_center_x:     int   = 400
    cam_center_y:     int   = 400
    
    # PID Gains (Proportional, Integral, Derivative)
    pid_kp: float = 12.0
    pid_ki: float = 0.5
    pid_kd: float = 10.0

    # Alignment Thresholds
    align_tolerance_m: float = 0.005  # 5mm tolerance
    target_offset_y:   float = -0.015 # Offset from center to gripper

# ==============================================================================
#                                  NODE LOGIC
# ==============================================================================

class CameraPNP(Node):
    def __init__(self):
        super().__init__('camera_pnp')
        
        # Load Configuration
        self.config = PNPConfig()
        
        # ROS Communication
        self.pose_pub = self.create_publisher(Pose, '/delta/target_pose', 10)
        self.speed_pub = self.create_publisher(Twist, '/delta/speed_params', 10) 
        self.suction_pub = self.create_publisher(Bool, '/suction/command', 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # State Tracking
        self.bridge = CvBridge()
        self.pos = [0.0, 0.0, self.config.safe_z]
        self.state = "SEARCH"
        self.target_visible = False
        self.logic_timer = 0
        self.current_roll = 0.0 # Degrees
        
        # PID Internal State
        self.prev_error_m = 0.0
        self.integral_error = 0.0
        self.d_filter = 0.0

        # Initialization
        self.unlock_speed() 
        self.create_timer(0.01, self.control_loop) 
        self.get_logger().info("PNP Initialized. Config loaded successfully.")

    def unlock_speed(self):
        """Periodically published to keep the robot active."""
        msg = Twist()
        msg.linear.x, msg.angular.z = 0.5, 2.0
        self.speed_pub.publish(msg)

    # --------------------------------------------------------------------------
    #                               VISION LOOP
    # --------------------------------------------------------------------------
    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            # Mask for red color (Low and High range combined)
            mask1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
            mask2 = cv2.inRange(hsv, (170, 100, 100), (180, 255, 255))
            mask = mask1 | mask2
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 300:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        self.target_visible = True
                        
                        # Only run alignment logic if in ALIGN state
                        if self.state == "ALIGN":
                            self.process_alignment(cx, cy)
                        return

            self.target_visible = False
            
        except Exception as e:
            self.get_logger().error(f"CV Error: {e}")

    def process_alignment(self, cx, cy):
        # Calculate X error for PID
        error_m = (cx - self.config.cam_center_x) / self.config.pixels_per_meter
        self.run_pid(error_m)
        
        # Check Y distance for triggering descent
        dist_y_m = (cy - self.config.cam_center_y) / self.config.pixels_per_meter
        
        # Check if we are aligned within tolerance
        y_aligned = abs(dist_y_m - self.config.target_offset_y) < self.config.align_tolerance_m
        x_stable  = abs(self.prev_error_m) < self.config.align_tolerance_m
        
        if y_aligned and x_stable:
            self.change_state("DESCEND")

    def run_pid(self, error_m):
        """Calculates velocity based on visual error."""
        # Update Integral
        self.integral_error = np.clip(self.integral_error + (error_m * 0.01), -0.5, 0.5)
        
        # Update Derivative (with low-pass filter)
        delta = error_m - self.prev_error_m
        self.d_filter = (0.7 * delta) + (0.3 * self.d_filter)
        
        # Compute Output
        p_term = self.config.pid_kp * error_m
        i_term = self.config.pid_ki * self.integral_error
        d_term = self.config.pid_kd * self.d_filter
        
        vel_y = -1 * (p_term + i_term + d_term)
        
        # Apply velocity to position (Simple integration)
        self.pos[1] += np.clip(vel_y, -20.0, 20.0) * 0.01
        self.prev_error_m = error_m

    # --------------------------------------------------------------------------
    #                            STATE MACHINE
    # --------------------------------------------------------------------------
    def control_loop(self):
        # Keep robot watchdog happy
        if self.logic_timer % 10 == 0: 
            self.unlock_speed()
            
        self.logic_timer += 1
        t = self.logic_timer * 0.01 # Current state duration in seconds

        # FSM
        if self.state == "SEARCH":
            self.pos = [0.0, 0.0, self.config.safe_z]
            if self.target_visible:
                self.change_state("ALIGN")

        elif self.state == "ALIGN":
            self.pos[2] = self.config.safe_z
            if not self.target_visible:
                self.change_state("SEARCH")

        elif self.state == "DESCEND":
            self.pos[2] = self.config.pick_z
            self.set_suction(True)
            if t >= self.config.descend_time:
                self.change_state("PICK")

        elif self.state == "PICK":
            # Wait for suction to establish vacuum
            if t >= self.config.pick_dwell:
                self.change_state("LIFT")

        elif self.state == "LIFT":
            self.pos[2] = self.config.safe_z
            if t >= self.config.lift_time:
                self.change_state("PLACE")
                self.current_roll = 99.0 # Rotate during travel

        elif self.state == "PLACE":
            self.pos[1] = self.config.place_y
            if t >= self.config.place_travel:
                self.set_suction(False)
                self.change_state("DROP_WAIT")

        elif self.state == "DROP_WAIT":
            if t >= self.config.drop_dwell:
                self.change_state("RETURN")

        elif self.state == "RETURN":
            self.pos = [0.0, 0.0, self.config.safe_z]
            self.current_roll = 0.0 # Reset Orientation
            if t >= self.config.return_travel:
                self.change_state("SEARCH")

        # Safety Clamping & Publishing
        self.pos[1] = np.clip(self.pos[1], self.config.y_limit_min, self.config.y_limit_max)
        self.publish_pose()

    def change_state(self, new_state):
        self.state = new_state
        self.logic_timer = 0
        self.get_logger().info(f"State: {new_state}")

    def set_suction(self, state):
        self.suction_pub.publish(Bool(data=state))

    def publish_pose(self):
        msg = Pose()
        msg.position.x = float(self.pos[0])
        msg.position.y = float(self.pos[1])
        msg.position.z = float(self.pos[2])
        
        # Roll (X-Rotation) to Quaternion:
        # q = [sin(r/2), 0, 0, cos(r/2)]
        rad = np.radians(self.current_roll)
        msg.orientation.x = np.sin(rad * 0.5)
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = np.cos(rad * 0.5)
        
        self.pose_pub.publish(msg)

def main():
    rclpy.init()
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