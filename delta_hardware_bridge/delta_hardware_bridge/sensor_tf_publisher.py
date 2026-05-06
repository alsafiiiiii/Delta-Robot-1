#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import tf2_ros

class SensorTfPublisher(Node):
    def __init__(self):
        super().__init__('sensor_tf_publisher')
        
        # --- Configuration ---
        # Adjust these offsets based on your physical "home" position
        self.HOME_OFFSET = [170.0, 170.0]  # [X, Y] in mm
        
        # --- TF Buffer and Listener (To get Z from Gazebo/Rviz) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- TF Broadcaster (To publish the final fused position) ---
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # --- Subscriptions ---
        # Set queue depth to 1 for real-time sensor feedback
        self.sub_sensors = self.create_subscription(
            Float32MultiArray,
            '/sharp_sensors/all',
            self.sensor_callback,
            1 
        )
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        self.get_logger().info("Sensor TF Publisher with TF Listener started")

    def sensor_callback(self, msg):
        """
        Triggered when ESP32 sends new Sharp sensor data.
        Fuses X/Y from sensors with Z from existing TF tree.
        """
        if len(msg.data) >= 2:
            # 1. Update X and Y from sensors
            # Note: Ensure these indices match your hardware!
            # If msg.data[0] is X and msg.data[1] is Y in your firmware:
            self.current_x = (msg.data[1] - self.HOME_OFFSET[1]) / 1000.0
            self.current_y = (-msg.data[0] + self.HOME_OFFSET[0]) / 1000.0
            
            # 2. Lookup Z from the simulation/robot TF tree safely
            try:
                # Use Time() to get the latest available transform
                # We use a 0.05s timeout to avoid blocking the callback too long
                now = rclpy.time.Time()
                
                # CRITICAL FIX: Swap the arguments if needed, but standard is (target, source)
                # target_frame: 'delta_robot/world_link' (Base)
                # source_frame: 'delta_robot/EE' (Moving Part)
                
                if self.tf_buffer.can_transform('delta_robot/world_link', 'delta_robot/EE', now, timeout=rclpy.duration.Duration(seconds=0.05)):
                    t_z = self.tf_buffer.lookup_transform(
                        'delta_robot/world_link', 
                        'delta_robot/EE', 
                        rclpy.time.Time()) # Get latest
                    
                    self.current_z = t_z.transform.translation.z
                else:
                    self.get_logger().warn("Transform Z not ready yet", throttle_duration_sec=1.0)
                    # Keep previous Z value
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"TF Error: {e}", throttle_duration_sec=1.0)
                # If Z isn't available yet, keep the previous value

            # 3. Publish the fused result
            self.publish_fused_tf()

    def publish_fused_tf(self):
        """Broadcasts the final position to the TF tree"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'delta_robot/world_link'
        t.child_frame_id = 'delta_robot/fused_end_effector' # Rename so it doesn't conflict
        
        t.transform.translation.x = self.current_x
        t.transform.translation.y = self.current_y
        t.transform.translation.z = self.current_z
        
        # Maintain identity rotation (Delta robots usually don't tilt the EE)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SensorTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()