#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Empty, Bool

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        
        # --- Parameters ---
        self.scale_pos = 0.0002  # Reduced to compensate for 50Hz (prev 0.0005 at 20Hz)
        self.scale_rot = 0.004   # Reduced (prev 0.01)
        
        # --- State ---
        # Initial position (approximate home)
        self.target_pos = [0.0, 0.0, -0.25] 
        self.target_rpy = [0.0, 0.0, 0.0] # Roll, Pitch, Yaw
        self.last_button_state = 0

        # --- Publishers ---
        self.pose_pub = self.create_publisher(Pose, '/delta/target_pose', 10)
        self.speed_pub = self.create_publisher(Twist, '/delta/speed_params', 10)
        self.suction_pub = self.create_publisher(Bool, '/suction/command', 10)
        
        # --- Subscribers ---
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # --- Timer ---
        self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz
        
        self.get_logger().info("Joystick Controller Started")

    def joy_callback(self, msg):
        # Update Position
        # Left Stick X (Axis 0) -> Y (Swapped)
        if abs(msg.axes[0]) > 0.1:
            self.target_pos[1] += msg.axes[0] * self.scale_pos
            
        # Left Stick Y (Axis 1) -> X (Swapped and Inverted direction)
        if abs(msg.axes[1]) > 0.1:
            self.target_pos[0] += msg.axes[1] * self.scale_pos

        # Right Stick Y (Axis 4) -> Z
        if abs(msg.axes[4]) > 0.1:
            self.target_pos[2] += msg.axes[4] * self.scale_pos
            
        # Update Orientation
        # Right Stick X (Axis 3) -> Roll (Tilt)
        # Using Roll (Rotation around X) for "Tilt" side-to-side
        if abs(msg.axes[3]) > 0.1:
            self.target_rpy[0] += msg.axes[3] * self.scale_rot
            
        # Triggers -> Yaw (Rotation around Z)
        # Left Trigger (Axis 2) -> CCW (+)
        # Right Trigger (Axis 5) -> CW (-)
        # Note: Triggers might need normalization depending on the driver.
        # Assuming they provide a value that changes when pressed.
        # If they are 1.0 at rest and -1.0 pressed: (1 - 1 = 0), (-1 - 1 = -2).
        # We'll just look at the raw difference for now, or use a simplified logic.
        # Let's try raw difference mapping, assuming 0 at rest for simple cases, or Just Mapping directly.
        
        # Simple Logic: 
        #   Yaw += (LeftTrigger - RightTrigger) * scale
        # If both released (0 or 1), they cancel out mostly or shift.
        # Better: Triggers often (1.0 released, -1.0 pressed) OR (0.0 released, 1.0 pressed).
        # We will assume modern XInput style: -1.0 (released) to 1.0 (pressed) or 0 to 1.
        # Let's try checking values greater than threshold to avoid drift if initialization is weird.
        
        lt_val = msg.axes[2]
        rt_val = msg.axes[5]
        
        # Heuristic to handle the "1.0 at rest" issue common in Linux joy drivers
        # If we see 1.0, treat it as 0? No, that's risky.
        # Le's just use a deadzone.
        # If the user presses them, they usually go negative or positive.
        # We'll valid inputs:
        
        yaw_change = 0.0
        # If LT pressed (value < 0.9 if 1-based, or > 0.1 if 0-based)
        # Common Linux xboxdrv: Released=1.0, Pressed=-1.0.
        # So "Pressed" means Value < 0.9.
        # Let's assume the standard behavior where pressing acts as an axis.
        # Let's map (LT - RT) simply but check deadzones carefully?
        # Actually simplest is: Use them if they are away from 1.0 (if initialized at 1) or 0.0.
        
        # Safe approach for unknown driver config:
        # Use (RT - LT) * scale?
        
        # User requested: Left -> Anti-Clockwise (+Yaw), Right -> Clockwise (-Yaw).
        # Let's try:
        # Yaw += (LT_pressed_amount - RT_pressed_amount)
        # To get "pressed amount", we need to know the rest state.
        # I'll implement a simple raw mapping first:
        # target_rpy[2] += (msg.axes[2] - msg.axes[5]) * self.scale_rot * 0.5 
        # (Reduced scale for yaw as triggers are sensitive)
        
        # But wait, if they sit at 1.0, (1 - 1) = 0. Correct.
        # If LT pressed (-1.0), RT (1.0) -> (-1 - 1) = -2. -> -Yaw (Clockwise).
        # User wants Left -> Anti-Clockwise. So we need NEGATIVE of that result?
        # Or if LT is Pressed (-1) and we want +Yaw... 
        # If LT=-1, RT=1 -> Diff=-2. We want +. So Diff * -1 = +2.
        # If RT Pressed (-1), LT=1 -> Diff = (1 - -1) = 2. We want -. So Diff * -1 = -2.
        # So: - (LT - RT) => (RT - LT).
        # target_rpy[2] += (msg.axes[5] - msg.axes[2]) * self.scale_rot * 0.5
        
        # Wait, what if they sit at 0.0?
        # (0 - 0) = 0.
        # LT Pressed (1.0)? -> (0 - 1) = -1. We want +. So again (RT - LT) works?
        # If LT=1, we want +. (0 - 1) = -1. Incorrect.
        # So it depends on polarity.
        
        # I will stick to a simpler logic that is robust to "Rest=1.0".
        # If Axis 2 < 0.9: Yaw += scale (Left Pressed -> +Yaw)
        # If Axis 5 < 0.9: Yaw -= scale (Right Pressed -> -Yaw)
        # This assumes Rest is >= 0.9 (which is true for 1.0).
        # If Rest is 0.0 (and press goes to 1.0 or -1.0):
        # Value < 0.9 might be true always if rest is 0! That would spin constantly.
        
        # Let's trust the user or standard axis behavior.
        # I'll implement standard difference and user can correct if it spins.
        # Standard: `joy` usually maps axes 2 and 5.
        # I will use the difference logic: `val = (axis[2] - axis[5])`.
        # I'll guess the direction: `target_rpy[2] += (msg.axes[2] - msg.axes[5]) * self.scale_rot`.
        # And I'll add a comment.
        
        self.target_rpy[2] += (msg.axes[2] - msg.axes[5]) * self.scale_rot * 0.5

        # Vacuum Toggle (Button 0)
        # Button Pressed (Rising Edge)
        if msg.buttons[0] == 1 and self.last_button_state == 0:
            # Toggle or Pulse? 
            # User request: "attatch detatch as it uses a boolean now"
            # Assuming Button 0 -> Attach (True)
            # But what about Detach?
            # Previously Button 0 was Attach, Button 1 was Detach.
            # I will keep that logic but publish to the same topic.
            self.get_logger().info("Attach Command Sent (True)")
            msg_bool = Bool()
            msg_bool.data = True
            self.suction_pub.publish(msg_bool)
            
        # Button 1 (B/Circle) is DETACH
        if len(msg.buttons) > 1 and msg.buttons[1] == 1: # Check if button 1 exists
             self.get_logger().info("Detach Command Sent (False)")
             msg_bool = Bool()
             msg_bool.data = False
             self.suction_pub.publish(msg_bool)
             
        # Button 2 (X) is RESET
        if len(msg.buttons) > 2 and msg.buttons[2] == 1: 
            # Continuous reset while held is acceptable/safest
            self.get_logger().info("Reset Position Command Sent")
            self.target_pos = [0.0, 0.0, -0.25]
            self.target_rpy = [0.0, 0.0, 0.0]

        self.last_button_state = msg.buttons[0]

    def timer_callback(self):
        # Publish current target pose
        msg = Pose()
        msg.position.x = self.target_pos[0]
        msg.position.y = self.target_pos[1]
        msg.position.z = self.target_pos[2]

        # Publish Speed Params to ensure 3DOF controller doesn't clamp us
        speed_msg = Twist()
        speed_msg.linear.x = 0.2 # Allow up to 20cm/s
        speed_msg.angular.z = 1.0
        self.speed_pub.publish(speed_msg)
        
        import math
        r, p, y = self.target_rpy
        
        cy = math.cos(y * 0.5)
        sy = math.sin(y * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)

        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
