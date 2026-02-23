#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

class TFErrorPlotter(Node):
    def __init__(self):
        super().__init__('tf_error_plotter')
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Frame names to compare
        self.reference_frame = 'delta_robot/world_link'
        self.commanded_frame = 'delta_robot/EE'  # From simulation/controller
        self.actual_frame = 'delta_robot/end_effector'  # From sensors
        
        # Data storage (last 100 points)
        self.max_points = 100
        self.times = deque(maxlen=self.max_points)
        self.error_x = deque(maxlen=self.max_points)
        self.error_y = deque(maxlen=self.max_points)
        self.error_z = deque(maxlen=self.max_points)
        self.error_total = deque(maxlen=self.max_points)
        
        self.start_time = self.get_clock().now()
        
        # Setup plot
        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('TF Error: Commanded vs Actual End-Effector Position', fontsize=14)
        
        # Timer for updating (20 Hz)
        self.timer = self.create_timer(0.05, self.update_data)
        
        self.get_logger().info(f"Comparing '{self.commanded_frame}' vs '{self.actual_frame}'")
        
    def update_data(self):
        try:
            # Get both transforms
            now = rclpy.time.Time()
            
            commanded_tf = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.commanded_frame,
                now
            )
            
            actual_tf = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.actual_frame,
                now
            )
            
            # Calculate errors
            dx = actual_tf.transform.translation.x - commanded_tf.transform.translation.x
            dy = actual_tf.transform.translation.y - commanded_tf.transform.translation.y
            dz = actual_tf.transform.translation.z - commanded_tf.transform.translation.z
            
            total_error = np.sqrt(dx**2 + dy**2 + dz**2)
            
            # Store data
            current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.times.append(current_time)
            self.error_x.append(dx * 1000)  # Convert to mm
            self.error_y.append(dy * 1000)
            self.error_z.append(dz * 1000)
            self.error_total.append(total_error * 1000)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=1.0)
    
    def update_plot(self, frame):
        if len(self.times) == 0:
            return
        
        times = list(self.times)
        
        # Plot X error
        self.axs[0, 0].clear()
        self.axs[0, 0].plot(times, list(self.error_x), 'r-', linewidth=2)
        self.axs[0, 0].set_ylabel('X Error (mm)')
        self.axs[0, 0].grid(True, alpha=0.3)
        self.axs[0, 0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        
        # Plot Y error
        self.axs[0, 1].clear()
        self.axs[0, 1].plot(times, list(self.error_y), 'g-', linewidth=2)
        self.axs[0, 1].set_ylabel('Y Error (mm)')
        self.axs[0, 1].grid(True, alpha=0.3)
        self.axs[0, 1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        
        # Plot Z error
        self.axs[1, 0].clear()
        self.axs[1, 0].plot(times, list(self.error_z), 'b-', linewidth=2)
        self.axs[1, 0].set_ylabel('Z Error (mm)')
        self.axs[1, 0].set_xlabel('Time (s)')
        self.axs[1, 0].grid(True, alpha=0.3)
        self.axs[1, 0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        
        # Plot Total error
        self.axs[1, 1].clear()
        self.axs[1, 1].plot(times, list(self.error_total), 'm-', linewidth=2)
        self.axs[1, 1].set_ylabel('Total Error (mm)')
        self.axs[1, 1].set_xlabel('Time (s)')
        self.axs[1, 1].grid(True, alpha=0.3)
        
        # Show current values
        if len(self.error_total) > 0:
            current_total = self.error_total[-1]
            self.axs[1, 1].set_title(f'Current: {current_total:.2f} mm', fontsize=10)
        
        plt.tight_layout()

def main(args=None):
    rclpy.init(args=args)
    node = TFErrorPlotter()
    
    # Setup animation
    ani = animation.FuncAnimation(
        node.fig, 
        node.update_plot, 
        interval=50,  # 20 Hz
        blit=False
    )
    
    # Show plot in non-blocking mode
    plt.ion()
    plt.show()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()
