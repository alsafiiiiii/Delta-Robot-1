#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import tf2_ros
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
from collections import deque

# --- CONFIGURATION ---
FIXED_FRAME = 'delta_robot/world_link'
# Replace these with your actual frame names
FRAME_BLUE  = 'delta_robot/EE'
FRAME_RED   = 'delta_robot/fused_end_effector'
MAX_POINTS  = 1000
# Fixed workspace size (meters) - CRITICAL FOR SPEED
X_LIMITS = (-0.05, 0.05)
Y_LIMITS = (-0.05, 0.05)
# ---------------------

class FastPathVisualizer(Node):
    def __init__(self):
        super().__init__('fast_path_visualizer')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Use Deque for O(1) appends and pops
        self.blue_x = deque(maxlen=MAX_POINTS)
        self.blue_y = deque(maxlen=MAX_POINTS)
        self.red_x = deque(maxlen=MAX_POINTS)
        self.red_y = deque(maxlen=MAX_POINTS)
        
        self.lock = threading.Lock()
        
        # 30Hz Sampling
        self.create_timer(0.033, self.sample_positions)
        self.get_logger().info("Fast Visualizer Started")
    
    def sample_positions(self):
        try:
            now = rclpy.time.Time()
            
            # Lookup Blue
            t_blue = self.tf_buffer.lookup_transform(FIXED_FRAME, FRAME_BLUE, now)
            bx, by = t_blue.transform.translation.x, t_blue.transform.translation.y
            
            # Lookup Red
            t_red = self.tf_buffer.lookup_transform(FIXED_FRAME, FRAME_RED, now)
            rx, ry = t_red.transform.translation.x, t_red.transform.translation.y

            # Thread-safe write
            with self.lock:
                self.blue_x.append(bx)
                self.blue_y.append(by)
                self.red_x.append(rx)
                self.red_y.append(ry)
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

def main(args=None):
    rclpy.init(args=args)
    node = FastPathVisualizer()
    
    # Setup Plot
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title('High-Performance TF Tracker')
    
    # Set FIXED limits. 
    # If we change limits during animation, blit=True breaks.
    ax.set_xlim(X_LIMITS)
    ax.set_ylim(Y_LIMITS)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.grid(True)
    ax.set_aspect('equal') # Important for robot paths
    
    # Initialize Lines (animated=True is required for blit)
    line_blue, = ax.plot([], [], 'b-', lw=1.5, label='Simulated', animated=True)
    line_red, = ax.plot([], [], 'r--', lw=1.5, label='Sensor', animated=True)
    ax.legend(loc='upper right')
    
    # Pre-allocate background for blitting
    def init():
        return line_blue, line_red

    def update_plot(frame):
        # Thread-safe read
        with node.lock:
            # Quick tuple copy is faster than np.array conversion
            bx, by = tuple(node.blue_x), tuple(node.blue_y)
            rx, ry = tuple(node.red_x), tuple(node.red_y)
        
        line_blue.set_data(bx, by)
        line_red.set_data(rx, ry)
        
        return line_blue, line_red
    
    # ROS Thread
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    
    # Start Animation
    # interval=20ms = 50 FPS
    # blit=True ONLY redraws the lines, not the grid/axes
    FuncAnimation(fig, update_plot, init_func=init, interval=20, blit=True)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()