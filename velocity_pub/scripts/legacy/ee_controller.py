#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class BevelGearController(Node):
    """
    High-accuracy differential joint controller.
    Simulates servo-coupled bevel gear motion between:
      - Inputs:  Bevelj1, Bevelj2
      - Outputs: BeveljEE, Tj1
    """
    def __init__(self):
        super().__init__('bevel_gear_controller')

        # Parameters
        self.joint1_name = 'Bevelj1'
        self.joint2_name = 'Bevelj2'
        self.output_joint_ee_name = 'BeveljEE'
        self.output_joint_tlink_name = 'Tj1'
        self.control_rate = 100.0  # Hz (increase from 50 → 100 for finer control)
        self.alpha = 0.1  # low-pass filter coefficient (0 = no smoothing, 1 = full smoothing)
        self.last_publish_time = 0.0
        self.min_publish_interval = 1.0 / self.control_rate

        # State
        self.bevel1_angle = 0.0
        self.bevel2_angle = 0.0
        self.filtered_bevel1 = 0.0
        self.filtered_bevel2 = 0.0
        self.joint_names_found = False
        self.last_targets = (None, None)

        # ROS 2 setup
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 20
        )

        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/model/delta_robot/joint_trajectory', 20
        )

        self.timer = self.create_timer(1.0 / self.control_rate, self.publish_command)
        self.get_logger().info('High-accuracy Bevel Gear Controller initialized.')

    def joint_state_callback(self, msg: JointState):
        """Update servo input angles with smoothing."""
        try:
            idx1 = msg.name.index(self.joint1_name)
            idx2 = msg.name.index(self.joint2_name)

            raw1 = float(msg.position[idx1])
            raw2 = float(msg.position[idx2])

            # Low-pass filter for smoother signal
            self.filtered_bevel1 = self.alpha * raw1 + (1 - self.alpha) * self.filtered_bevel1
            self.filtered_bevel2 = self.alpha * raw2 + (1 - self.alpha) * self.filtered_bevel2

            self.bevel1_angle = self.filtered_bevel1
            self.bevel2_angle = self.filtered_bevel2

            if not self.joint_names_found:
                self.get_logger().info('Found input joints in /joint_states.')
                self.joint_names_found = True

        except ValueError:
            if not self.joint_names_found:
                self.get_logger().warn(
                    f"Waiting for '{self.joint1_name}' and '{self.joint2_name}' in /joint_states...",
                    throttle_duration_sec=5,
                )

    def publish_command(self):
        """Compute and publish differential output commands."""
        if not self.joint_names_found:
            return

        # Rate limiter
        now = time.time()
        if now - self.last_publish_time < self.min_publish_interval:
            return
        self.last_publish_time = now

        # --- Differential computation ---
        b1 = self.bevel1_angle
        b2 = self.bevel2_angle

        # More precise floating-point math
        target_angle_ee = 0.25 * (b1 + b2)
        target_angle_tlink = 0.5 * (b1 - b2)

        # Skip if change is too small (avoids redundant publishing)
        if self.last_targets == (target_angle_ee, target_angle_tlink):
            return
        self.last_targets = (target_angle_ee, target_angle_tlink)

        # --- Trajectory Message ---
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [self.output_joint_ee_name, self.output_joint_tlink_name]

        point = JointTrajectoryPoint()
        point.positions = [target_angle_ee, target_angle_tlink]
        point.time_from_start = Duration(sec=0, nanosec=int(1e9 / self.control_rate))

        traj_msg.points.append(point)
        self.trajectory_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BevelGearController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
