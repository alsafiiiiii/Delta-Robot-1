#!/usr/bin/env python3
"""
Delta Robot IK Calibration Script

Moves the robot through predefined test positions and compares:
- Commanded positions (from IK)
- Actual positions (from sensors)

Then calculates optimal geometry parameters.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_ros import TransformListener, Buffer
import tf2_ros
import time
import numpy as np
from scipy.optimize import minimize

class IKCalibrator(Node):
    def __init__(self):
        super().__init__('ik_calibrator')
        
        # TF listener for sensor feedback
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher for commanded positions
        self.pose_pub = self.create_publisher(Pose, '/delta/target_pose', 10)
        
        # Data storage
        self.commanded_positions = []
        self.measured_positions = []
        
        self.get_logger().info("IK Calibrator initialized")
    
    def move_and_measure(self, x, y, z, settle_time=2.0):
        """
        Move to position and measure where robot actually went.
        
        Args:
            x, y, z: Target position
            settle_time: Seconds to wait for robot to settle
        """
        # Send command
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.orientation.w = 1.0
        
        self.pose_pub.publish(msg)
        self.get_logger().info(f"Commanded: ({x:.3f}, {y:.3f}, {z:.3f})")
        
        # Wait for robot to move and settle
        time.sleep(settle_time)
        
        # Read sensor position
        try:
            t = self.tf_buffer.lookup_transform(
                'delta_robot/world_link',
                'delta_robot/fused_end_effector',
                rclpy.time.Time()
            )
            measured_x = t.transform.translation.x
            measured_y = t.transform.translation.y
            measured_z = t.transform.translation.z
            
            self.get_logger().info(f"Measured:  ({measured_x:.3f}, {measured_y:.3f}, {measured_z:.3f})")
            
            # Store data
            self.commanded_positions.append([x, y, z])
            self.measured_positions.append([measured_x, measured_y, measured_z])
            
            return (measured_x, measured_y, measured_z)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Could not read sensor position: {e}")
            return None
    
    def run_calibration(self):
        """Execute calibration routine with test movements."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("Starting IK Calibration Procedure")
        self.get_logger().info("="*60)
        
        # Test positions - a grid around home
        # Using Z from simulation since no Z sensor
        test_positions = [
            (0.00,  0.00, -0.235),  # Home/center
            (0.03,  0.00, -0.235),  # +X
            (-0.03, 0.00, -0.235),  # -X
            (0.00,  0.03, -0.235),  # +Y
            (0.00, -0.03, -0.235),  # -Y
            (0.02,  0.02, -0.235),  # +X+Y
            (-0.02, 0.02, -0.235),  # -X+Y
            (0.02, -0.02, -0.235),  # +X-Y
            (-0.02,-0.02, -0.235),  # -X-Y
        ]
        
        self.get_logger().info(f"\nWill test {len(test_positions)} positions...")
        input("Press ENTER to start calibration movements...")
        
        for i, (x, y, z) in enumerate(test_positions, 1):
            self.get_logger().info(f"\n--- Test {i}/{len(test_positions)} ---")
            result = self.move_and_measure(x, y, z)
            if result is None:
                self.get_logger().warn(f"Skipping position ({x}, {y}, {z}) - no sensor data")
        
        self.analyze_results()
    
    def analyze_results(self):
        """Analyze collected data and suggest parameter corrections."""
        if len(self.commanded_positions) < 3:
            self.get_logger().error("Not enough data points for calibration!")
            return
        
        cmd = np.array(self.commanded_positions)
        meas = np.array(self.measured_positions)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("CALIBRATION RESULTS")
        self.get_logger().info("="*60)
        
        # Calculate errors
        errors = meas - cmd
        mean_error = np.mean(errors, axis=0)
        rms_error = np.sqrt(np.mean(errors**2, axis=0))
        
        self.get_logger().info(f"\nMean Error (X, Y, Z): ({mean_error[0]*1000:.2f}, {mean_error[1]*1000:.2f}, {mean_error[2]*1000:.2f}) mm")
        self.get_logger().info(f"RMS Error  (X, Y, Z): ({rms_error[0]*1000:.2f}, {rms_error[1]*1000:.2f}, {rms_error[2]*1000:.2f}) mm")
        
        # Calculate scaling factors (simple linear approximation)
        # Only for XY since Z comes from simulation
        scale_x = np.std(meas[:, 0]) / np.std(cmd[:, 0]) if np.std(cmd[:, 0]) > 0.001 else 1.0
        scale_y = np.std(meas[:, 1]) / np.std(cmd[:, 1]) if np.std(cmd[:, 1]) > 0.001 else 1.0
        
        self.get_logger().info(f"\nScaling Factors:")
        self.get_logger().info(f"  X: {scale_x:.4f} (actual/commanded)")
        self.get_logger().info(f"  Y: {scale_y:.4f} (actual/commanded)")
        
        # Current geometry
        current_params = {
            'r_base': 0.103,
            'r_ee': 0.040,
            'l_upper': 0.105,
            'l_lower': 0.205
        }
        
        self.get_logger().info(f"\nCurrent Geometry Parameters:")
        self.get_logger().info(f"  r_base  = {current_params['r_base']*1000:.1f} mm")
        self.get_logger().info(f"  r_ee    = {current_params['r_ee']*1000:.1f} mm")
        self.get_logger().info(f"  l_upper = {current_params['l_upper']*1000:.1f} mm")
        self.get_logger().info(f"  l_lower = {current_params['l_lower']*1000:.1f} mm")
        
        # Suggest corrections based on scaling
        # If robot moves less, arms might be shorter
        avg_scale = (scale_x + scale_y) / 2.0
        
        self.get_logger().info(f"\nSuggested Corrections (based on {avg_scale:.3f} average scale):")
        
        if avg_scale < 0.95:
            self.get_logger().info(f"  Robot moves LESS than commanded → Arms might be SHORTER")
            suggested_l_lower = current_params['l_lower'] * avg_scale
            self.get_logger().info(f"  Try: l_lower = {suggested_l_lower*1000:.1f} mm (currently {current_params['l_lower']*1000:.1f} mm)")
        elif avg_scale > 1.05:
            self.get_logger().info(f"  Robot moves MORE than commanded → Arms might be LONGER")
            suggested_l_lower = current_params['l_lower'] * avg_scale
            self.get_logger().info(f"  Try: l_lower = {suggested_l_lower*1000:.1f} mm (currently {current_params['l_lower']*1000:.1f} mm)")
        else:
            self.get_logger().info(f"  Scaling is close to 1.0 - geometry seems correct!")
            self.get_logger().info(f"  Errors might be due to:")
            self.get_logger().info(f"    - Sensor calibration (HOME_OFFSET)")
            self.get_logger().info(f"    - Mechanical play/backlash")
            self.get_logger().info(f"    - Coordinate frame mismatch")
        
        # Save detailed results
        results_file = '/tmp/ik_calibration_results.txt'
        with open(results_file, 'w') as f:
            f.write("Delta Robot IK Calibration Results\n")
            f.write("="*60 + "\n\n")
            f.write("Commanded vs Measured Positions:\n")
            f.write(f"{'Commanded (m)':<30s} {'Measured (m)':<30s} {'Error (mm)':<20s}\n")
            f.write("-"*80 + "\n")
            for cmd_pos, meas_pos in zip(self.commanded_positions, self.measured_positions):
                err = np.array(meas_pos) - np.array(cmd_pos)
                f.write(f"({cmd_pos[0]:6.3f}, {cmd_pos[1]:6.3f}, {cmd_pos[2]:6.3f})  ")
                f.write(f"({meas_pos[0]:6.3f}, {meas_pos[1]:6.3f}, {meas_pos[2]:6.3f})  ")
                f.write(f"({err[0]*1000:5.1f}, {err[1]*1000:5.1f}, {err[2]*1000:5.1f})\n")
        
        self.get_logger().info(f"\nDetailed results saved to: {results_file}")


def main(args=None):
    rclpy.init(args=args)
    calibrator = IKCalibrator()
    
    try:
        calibrator.run_calibration()
    except KeyboardInterrupt:
        pass
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
