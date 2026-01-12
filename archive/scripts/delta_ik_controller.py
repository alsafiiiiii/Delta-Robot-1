#!/usr/bin/env python3
"""
Delta Robot Inverse Kinematics Controller for ROS2
Subscribes to desired XYZ positions and publishes joint trajectories
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame


class DeltaIKController(Node):
    def __init__(self):
        super().__init__('delta_ik_controller')
        
        # Delta robot parameters
        self.r_base = 0.07582127019  # r1 in meters
        self.r_ee = 0.035            # r2 in meters
        self.l1 = 0.075              # upper arm length in meters
        self.l2 = 0.2639602098       # forearm length in meters
        
        # Initialize robot
        self.robot = RobotDelta(np.array([self.r_base, self.r_ee, self.l1, self.l2]))
        
        # Publisher for joint trajectory
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/model/delta_robot/joint_trajectory',
            10
        )
        
        # Subscriber for target position
        self.position_sub = self.create_subscription(
            Point,
            '/delta/target_position',
            self.position_callback,
            10
        )
        
        self.get_logger().info('Delta IK Controller initialized')
        self.get_logger().info(f'Robot parameters: r_base={self.r_base}, r_ee={self.r_ee}, l1={self.l1}, l2={self.l2}')
        self.get_logger().info('Listening on /delta/target_position (geometry_msgs/Point)')
        self.get_logger().info('Publishing to /model/delta_robot/joint_trajectory')
    
    def position_callback(self, msg):
        """
        Callback for target position
        Computes inverse kinematics and publishes joint trajectory
        """
        # Extract target position
        target_position = np.array([[msg.x], [msg.y], [msg.z]])
        
        self.get_logger().info(f'Received target: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}')
        
        try:
            # Create target frame (no rotation)
            target_frame = Frame.from_euler_3(np.array([0., 0., 0.]), target_position)
            
            # Compute inverse kinematics
            joint_angles_rad = self.robot.inverse(target_frame)
            
            # Log results
            self.get_logger().info(f'Computed joint angles (rad): {joint_angles_rad.flatten()}')
            self.get_logger().info(f'Computed joint angles (deg): {np.rad2deg(joint_angles_rad.flatten())}')
            
            # Create and publish trajectory message
            self.publish_trajectory(joint_angles_rad.flatten())
            
        except Exception as e:
            self.get_logger().error(f'IK computation failed: {str(e)}')
    
    def publish_trajectory(self, joint_angles):
        """
        Publish joint trajectory message
        """
        msg = JointTrajectory()
        msg.joint_names = ['jbf1', 'jbf2', 'jbf3']
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [float(joint_angles[0]), 
                          float(joint_angles[1]), 
                          float(joint_angles[2])]
        point.time_from_start = Duration(sec=2, nanosec=0)  # 2 seconds to reach target
        
        msg.points.append(point)
        
        # Publish
        self.joint_pub.publish(msg)
        self.get_logger().info('Published joint trajectory')


def main(args=None):
    rclpy.init(args=args)
    node = DeltaIKController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
