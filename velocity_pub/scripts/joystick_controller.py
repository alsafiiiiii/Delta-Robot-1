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
        self.max_linear_speed = 0.15 # m/s (Reduced for better control)
        self.max_angular_speed = 0.5 # rad/s
        
        # --- State ---
        # Initial position (approximate home)
        self.target_pos = [0.0, 0.0, -0.22] 
        self.target_rpy = [0.0, 0.0, 0.0] # Roll, Pitch, Yaw
        self.last_button_state = 0
        
        # Inputs (Velocities)
        self.vel_cmd = [0.0, 0.0, 0.0] # Vx, Vy, Vz
        self.ang_cmd = [0.0, 0.0, 0.0] # Roll_rate, Pitch_rate, Yaw_rate

        # --- Publishers ---
        self.pose_pub = self.create_publisher(Pose, '/delta/target_pose', 10)
        self.speed_pub = self.create_publisher(Twist, '/delta/speed_params', 10)
        self.suction_pub = self.create_publisher(Bool, '/suction/command', 10)
        
        # --- Subscribers ---
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # --- Timer ---
        self.dt = 0.02 # 50Hz
        self.timer = self.create_timer(self.dt, self.timer_callback) 
        
        self.get_logger().info("Joystick Controller Started (Velocity Mode)")

    def joy_callback(self, msg):
        # Map Joystick Inputs to Velocities
        # Left Stick X (Axis 0) -> Vy
        # Left Stick Y (Axis 1) -> Vx
        # Right Stick Y (Axis 4) -> Vz
        
        deadzone = 0.1
        
        # Vx (Inverted Axis 1)
        self.vel_cmd[0] = 0.0
        if abs(msg.axes[1]) > deadzone:
            self.vel_cmd[0] = msg.axes[1] * self.max_linear_speed
            
        # Vy (Axis 0)
        self.vel_cmd[1] = 0.0
        if abs(msg.axes[0]) > deadzone:
            self.vel_cmd[1] = msg.axes[0] * self.max_linear_speed
            
        # Vz (Axis 4)
        self.vel_cmd[2] = 0.0
        if abs(msg.axes[4]) > deadzone:
            self.vel_cmd[2] = msg.axes[4] * self.max_linear_speed
            
        # Angular Rates
        # Axis 3 -> Roll Rate
        self.ang_cmd[0] = 0.0
        if abs(msg.axes[3]) > deadzone:
            self.ang_cmd[0] = msg.axes[3] * self.max_angular_speed

        # Yaw (Triggers)
        # Assuming Triggers go from 1.0 (release) to -1.0 (press) or 0 to 1
        # Simple difference logic
        val_yaw = (msg.axes[2] - msg.axes[5]) 
        self.ang_cmd[2] = val_yaw * self.max_angular_speed * 0.5

        # Button Logic (Same as before)
        if msg.buttons[0] == 1 and self.last_button_state == 0:
            self.get_logger().info("Attach Command Sent (True)")
            self.suction_pub.publish(Bool(data=True))
        if len(msg.buttons) > 1 and msg.buttons[1] == 1: 
             self.get_logger().info("Detach Command Sent (False)")
             self.suction_pub.publish(Bool(data=False))
        if len(msg.buttons) > 2 and msg.buttons[2] == 1: 
            self.get_logger().info("Reset Position Command Sent")
            self.target_pos = [0.0, 0.0, -0.22]
            self.target_rpy = [0.0, 0.0, 0.0]

        self.last_button_state = msg.buttons[0]

    def timer_callback(self):
        # Integrate Velocity to Position
        # New_Pos = Old_Pos + (Velocity * dt)
        
        self.target_pos[0] += self.vel_cmd[0] * self.dt
        self.target_pos[1] += self.vel_cmd[1] * self.dt
        self.target_pos[2] += self.vel_cmd[2] * self.dt
        
        self.target_rpy[0] += self.ang_cmd[0] * self.dt
        self.target_rpy[1] += self.ang_cmd[1] * self.dt
        self.target_rpy[2] += self.ang_cmd[2] * self.dt

        # Publish Target Pose
        msg = Pose()
        msg.position.x = self.target_pos[0]
        msg.position.y = self.target_pos[1]
        msg.position.z = self.target_pos[2]
        
        # RPY -> Quaternion
        import math
        r, p, y = self.target_rpy
        cy = math.cos(y * 0.5); sy = math.sin(y * 0.5)
        cp = math.cos(p * 0.5); sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5); sr = math.sin(r * 0.5)
        
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        
        self.pose_pub.publish(msg)

        # Publish Speed Params (Optional, mainly for GCode/Auto moves)
        # For Teleop, we rely on the controller's bypass logic
        speed_msg = Twist()
        speed_msg.linear.x = 0.2 # Default run speed
        speed_msg.linear.y = 1.0 # Default Accel
        speed_msg.linear.z = 1.0 # Default Decel
        self.speed_pub.publish(speed_msg)

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
