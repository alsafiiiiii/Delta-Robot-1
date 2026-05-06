#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco
import numpy as np
import math
from dataclasses import dataclass

# ==============================================================================
#                               USER CONFIGURATION
# ==============================================================================
@dataclass
class PNPConfig:
    """Centralized configuration for Pick and Place logic."""
    
    # --- 1. TIMINGS (Seconds) ---
    descend_time:  float = 0.2
    pick_dwell:    float = 0.4   # Time to wait while suction engages
    lift_time:     float = 0.3
    place_travel:  float = 1.0   # Time allowed to move to place zone
    drop_dwell:    float = 0.1   # Time to wait after dropping
    return_travel: float = 0.80  # Time allowed to return to search

    # --- 2. PHYSICAL POSITIONS (Meters) ---
    safe_z:    float = -0.200    # Cruising height
    pick_z:    float = -0.315    # Height to touch the object
    place_y:   float =  -0.180    # Y-coordinate for drop-off
    
    # --- 3. SAFETY LIMITS ---
    y_limit_min: float = -0.18
    y_limit_max: float =  0.18
    
    # --- 4. VISION & CONTROL ---
    pixels_per_meter: float = 9500.0
    cam_center_x:     int   = 400
    cam_center_y:     int   = 400
    
    # PID Gains (Proportional, Integral, Derivative)
    pid_kp: float = 20.50
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
        # Debug Publisher
        self.debug_pub = self.create_publisher(Image, '/camera/pnp_debug', 10)
        
        # ArUco Configuration
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # State Tracking
        self.bridge = CvBridge()
        self.pos = [0.0, 0.0, self.config.safe_z]
        self.state = "SEARCH"
        self.target_visible = False
        self.logic_timer = 0
        
        # Orientation State
        self.current_roll = 0.0 # Tilt (Degrees)
        self.current_yaw  = 0.0 # Spin (Radians)
        self.detected_yaw = 0.0
        
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
            debug_img = img.copy()
            
            # Detect ArUco Markers
            (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)
                
                # Assuming the first marker is the target
                c = corners[0][0] # Corners of the first marker [TL, TR, BR, BL]
                
                # Calculate Center
                cx = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                cy = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)
                
                # Draw Center
                cv2.circle(debug_img, (cx, cy), 5, (0, 255, 0), -1)
                cv2.line(debug_img, (self.config.cam_center_x, self.config.cam_center_y), (cx, cy), (0, 255, 255), 2)
                
                # Calculate Orientation (Yaw)
                # Vector from TL to TR
                dx = c[1][0] - c[0][0]
                dy = c[1][1] - c[0][1]
                
                # Angle in image frame
                angle = math.atan2(dy, dx)
                # Correct for 180-degree robot rotation
                self.detected_yaw = angle + math.pi
                
                # Draw Orientation
                end_pt = (int(cx + dx), int(cy + dy))
                cv2.arrowedLine(debug_img, (cx, cy), end_pt, (255, 0, 0), 2)
                
                # Display angle information
                angle_deg = np.degrees(angle)
                corrected_deg = np.degrees(self.detected_yaw)
                cv2.putText(debug_img, f"Raw: {angle:.3f} rad ({angle_deg:.1f} deg)", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(debug_img, f"Corrected: {self.detected_yaw:.3f} rad ({corrected_deg:.1f} deg)", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                self.target_visible = True
                
                # Only run alignment logic if in ALIGN state (or SEARCH to transition)
                if self.state == "ALIGN" or self.state == "SEARCH":
                    self.process_alignment(cx, cy)
                
            else:
                self.target_visible = False
                
            # Publish Debug Image
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))
            
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
        
        if y_aligned and x_stable and self.state == "ALIGN":
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
            self.current_roll = 0.0
            self.current_yaw = 0.0
            if self.target_visible:
                self.change_state("ALIGN")

        elif self.state == "ALIGN":
            self.pos[2] = self.config.safe_z
            # Match detected orientation
            self.current_yaw = self.detected_yaw
            self.get_logger().info(f"ALIGN: Detected Yaw: {self.detected_yaw:.3f} -> Target Yaw: {self.current_yaw:.3f}")
            if not self.target_visible:
                self.change_state("SEARCH")

        elif self.state == "DESCEND":
            self.pos[2] = self.config.pick_z
            self.set_suction(True)
            # Lock Orientation
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
                self.current_roll = 0.0 # Reset Tilt
                self.current_yaw = 0.0  # Reset Yaw for placement (place aligned to world)

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
        
        # Calculate Quaternion from Roll (Tilt) and Yaw (Spin)
        # Roll is in Degrees (legacy), Yaw is in Radians
        roll_rad = np.radians(self.current_roll)
        yaw_rad = self.current_yaw
        
        # Quaternion synthesis:
        # q = q_yaw * q_roll (Intrinsic rotation scenario or extrinsic?)
        # Let's do standard RPY to Quaternion assuming Pitch=0
        
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        
        # q = [w, x, y, z] order variations exist. ROS uses [x, y, z, w]
        # w = cr*cy
        # x = sr*cy
        # y = sr*sy (since pitch is 0, cos(p/2)=1, sin(p/2)=0) -> No wait.
        # correct formula for P=0:
        # w = cr * cp * cy + sr * sp * sy = cr * 1 * cy + sr * 0 * sy = cr * cy
        # x = sr * cp * cy - cr * sp * sy = sr * 1 * cy - cr * 0 * sy = sr * cy
        # y = cr * sp * cy + sr * cp * sy = cr * 0 * cy + sr * 1 * sy = sr * sy
        # z = cr * cp * sy - sr * sp * cy = cr * 1 * sy - sr * 0 * cy = cr * sy
        
        msg.orientation.w = cr * cy
        msg.orientation.x = sr * cy
        msg.orientation.y = sr * sy
        msg.orientation.z = cr * sy
        
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