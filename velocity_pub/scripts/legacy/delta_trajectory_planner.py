#!/usr/bin/env python3
"""
Delta Robot Trajectory Planner - JSON Input Only
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
import numpy as np
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame
import json


class DeltaTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('delta_trajectory_planner')
        
        # Delta robot parameters
        self.r_base = 0.07582127019
        self.r_ee = 0.035
        self.l1 = 0.075
        self.l2 = 0.2639602098
        
        # Initialize robot
        self.robot = RobotDelta(np.array([self.r_base, self.r_ee, self.l1, self.l2]))
        
        # Trajectory parameters
        self.declare_parameter('max_velocity', 0.01)
        self.declare_parameter('max_acceleration', 2.0)
        self.declare_parameter('trajectory_resolution', 100)
        
        self.max_vel = self.get_parameter('max_velocity').value
        self.max_accel = self.get_parameter('max_acceleration').value
        self.resolution = self.get_parameter('trajectory_resolution').value
        
        # Publisher
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/model/delta_robot/joint_trajectory',
            10
        )
        
        # Subscriber
        self.path_sub = self.create_subscription(
            String,
            '/delta/path_command',
            self.path_callback,
            10
        )
        
        self.get_logger().info('Delta Trajectory Planner initialized')
        self.get_logger().info(f'Max velocity: {self.max_vel} m/s, Max acceleration: {self.max_accel} m/s²')
        self.get_logger().info('Listening on /delta/path_command')
        self.get_logger().info('Send JSON: [[x1,y1,z1], [x2,y2,z2], ...] in METERS')
    
    def cartesian_to_joint_trajectory(self, waypoints):
        """Convert Cartesian waypoints to joint trajectory"""
        trajectory = []
        cumulative_time = 0.0
        
        # Convert all waypoints to joint angles
        joint_waypoints = []
        for wp in waypoints:
            target_position = np.array([[wp[0]], [wp[1]], [wp[2]]])
            target_frame = Frame.from_euler_3(np.array([0., 0., 0.]), target_position)
            try:
                joint_angles = self.robot.inverse(target_frame).flatten()
                joint_waypoints.append(joint_angles)
            except Exception as e:
                self.get_logger().error(f'IK failed for waypoint {wp}: {e}')
                return None
        
        # Generate trajectory with velocity profile
        for i in range(len(waypoints) - 1):
            start_pos = np.array(waypoints[i])
            end_pos = np.array(waypoints[i + 1])
            distance = np.linalg.norm(end_pos - start_pos)
            
            if distance < 1e-6:
                continue
            
            # Calculate motion profile times
            t_accel = self.max_vel / self.max_accel
            d_accel = 0.5 * self.max_accel * t_accel**2
            
            if 2 * d_accel > distance:
                t_accel = np.sqrt(distance / self.max_accel)
                t_const = 0.0
                actual_max_v = self.max_accel * t_accel
            else:
                t_const = (distance - 2 * d_accel) / self.max_vel
                actual_max_v = self.max_vel
            
            t_total = 2 * t_accel + t_const
            num_points = max(int(t_total * self.resolution), 2)
            times = np.linspace(0, t_total, num_points)
            
            for t in times:
                if t <= t_accel:
                    s = 0.5 * self.max_accel * t**2
                    v = self.max_accel * t
                elif t <= t_accel + t_const:
                    s = d_accel + actual_max_v * (t - t_accel)
                    v = actual_max_v
                else:
                    s = distance - 0.5 * self.max_accel * (t_total - t)**2
                    v = actual_max_v - self.max_accel * (t - t_accel - t_const)
                
                alpha = s / distance
                cart_pos = start_pos + alpha * (end_pos - start_pos)
                
                target_frame = Frame.from_euler_3(np.array([0., 0., 0.]), cart_pos.reshape(3, 1))
                joint_angles = self.robot.inverse(target_frame).flatten()
                
                if len(trajectory) > 0:
                    dt = 1.0 / self.resolution
                    joint_vel = (joint_angles - trajectory[-1][0]) / dt
                else:
                    joint_vel = np.zeros(3)
                
                trajectory.append((joint_angles, joint_vel, cumulative_time + t))
            
            cumulative_time += t_total
        
        return trajectory
    
    def path_callback(self, msg):
        """Execute path from JSON waypoint list"""
        try:
            waypoints = json.loads(msg.data)
            self.get_logger().info(f'Executing path with {len(waypoints)} waypoints')
            
            trajectory = self.cartesian_to_joint_trajectory(waypoints)
            if trajectory:
                self.publish_trajectory(trajectory)
            
        except Exception as e:
            self.get_logger().error(f'Path execution failed: {e}')
    
    def publish_trajectory(self, trajectory):
        """Publish joint trajectory"""
        msg = JointTrajectory()
        msg.joint_names = ['jbf1', 'jbf2', 'jbf3']
        
        for joint_angles, joint_vel, time_stamp in trajectory:
            point = JointTrajectoryPoint()
            point.positions = [float(joint_angles[0]), float(joint_angles[1]), float(joint_angles[2])]
            point.velocities = [float(joint_vel[0]), float(joint_vel[1]), float(joint_vel[2])]
            
            sec = int(time_stamp)
            nanosec = int((time_stamp - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            
            msg.points.append(point)
        
        self.joint_pub.publish(msg)
        self.get_logger().info(f'Published trajectory with {len(trajectory)} points, duration: {time_stamp:.2f}s')


def main(args=None):
    rclpy.init(args=args)
    node = DeltaTrajectoryPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
