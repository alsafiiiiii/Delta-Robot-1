#!/usr/bin/env python3
"""
Delta Robot G-code Interpreter with Position-Based Completion
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame
from gcodeparser import GcodeParser
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from collections import deque

class DeltaGcodeInterpreter(Node):
    def __init__(self):
        super().__init__('delta_gcode_interpreter')
        
        # Delta robot parameters
        self.r_base = 0.07582127019
        self.r_ee = 0.035
        self.l1 = 0.075
        self.l2 = 0.2639602098
        
        self.robot = RobotDelta(np.array([self.r_base, self.r_ee, self.l1, self.l2]))
        
        # Current position
        self.current_pos = np.array([0.0, 0.0, -250.0])
        self.current_joint_angles = np.array([0.0, 0.0, 0.0])
        self.state_received = False
        
        self.feedrate = 1000.0
        self.absolute_mode = True
        self.units = 1.0
        
        # Command queue and execution state
        self.command_queue = deque()
        self.is_moving = False
        self.target_position = None  # Track target for position-based completion
        self.position_tolerance = 2.0  # 2mm tolerance
        self.min_move_time = None  # Minimum time before checking position
        
        # Publishers
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/model/delta_robot/joint_trajectory',
            10
        )
        
        self.marker_pub = self.create_publisher(
            Marker,
            '/delta/trajectory_marker',
            10
        )
        
        self.marker_id = 0
        
        # Subscribers
        self.gcode_sub = self.create_subscription(
            String,
            '/delta/gcode',
            self.gcode_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Timer to process command queue
        self.queue_timer = self.create_timer(0.05, self.process_queue)  # Check more frequently
        
        self.get_logger().info('Delta G-code Interpreter with position-based completion')
        self.get_logger().info('Commands execute sequentially')
    
    def publish_trajectory_marker(self, waypoints):
        """Publish trajectory as LINE_STRIP marker for RViz"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = self.marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.scale.x = 0.002
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        for wp in waypoints:
            point = Point()
            point.x = wp[0] / 1000.0
            point.y = wp[1] / 1000.0
            point.z = wp[2] / 1000.0
            marker.points.append(point)
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        self.marker_pub.publish(marker)
        self.marker_id += 1
    
    def joint_state_callback(self, msg):
        """Update current position from joint feedback"""
        try:
            joint_names = ['jbf1', 'jbf2', 'jbf3']
            angles = []
            
            for name in joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    angles.append(msg.position[idx])
            
            if len(angles) == 3:
                self.current_joint_angles = np.array(angles)
                
                theta = np.array(angles)
                ee_frame = self.robot.forward(theta)
                
                ee_pos_m = ee_frame.t_3_1.flatten()
                self.current_pos = ee_pos_m * 1000.0
                
                if not self.state_received:
                    self.get_logger().info(
                        f'Initial state: X:{self.current_pos[0]:.2f} '
                        f'Y:{self.current_pos[1]:.2f} Z:{self.current_pos[2]:.2f} mm'
                    )
                    self.state_received = True
                    
        except Exception as e:
            self.get_logger().warn(f'Failed to update state: {e}')
    
    def gcode_callback(self, msg):
        """Process G-code string and add commands to queue"""
        if not self.state_received:
            self.get_logger().warn('No joint state received yet, waiting...')
            return
        
        self.get_logger().info(f'Received G-code block')
        
        try:
            # Split by newlines and semicolons
            lines = msg.data.replace(';', '\n').split('\n')
            
            for line_str in lines:
                line_str = line_str.strip()
                if not line_str:
                    continue
                
                # Parse individual line
                try:
                    parsed = GcodeParser(line_str)
                    for line in parsed.lines:
                        if line.command:
                            self.command_queue.append(line)
                            self.get_logger().info(f'Queued: {line.command} {line.params}')
                except:
                    pass
                    
        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')
    
    def process_queue(self):
        """Process command queue - check position-based completion"""
        # Check if movement is complete based on position
        if self.is_moving:
            # Wait minimum time before checking position
            if self.min_move_time is not None:
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time < self.min_move_time:
                    return  # Still in minimum wait period
            
            # Check if we reached target position
            if self.target_position is not None:
                distance = np.linalg.norm(self.current_pos - self.target_position)
                
                if distance < self.position_tolerance:
                    self.is_moving = False
                    self.target_position = None
                    self.get_logger().info(f'✓ Reached target (error: {distance:.2f}mm)')
        
        # Execute next command if not moving
        if not self.is_moving and len(self.command_queue) > 0:
            line = self.command_queue.popleft()
            self.get_logger().info(
                f'▶ Executing: {line.command} {line.params} '
                f'(queue: {len(self.command_queue)} remaining)'
            )
            self.execute_gcode_line(line)
    
    def execute_gcode_line(self, line):
        """Execute a parsed G-code line"""
        if not line.command:
            return
        
        cmd_letter, cmd_num = line.command
        cmd = f'{cmd_letter}{cmd_num}' if cmd_num is not None else cmd_letter
        params = line.params
        
        if cmd in ['G0', 'G1']:
            self.linear_move(params)
        elif cmd in ['G2', 'G3']:
            self.arc_move(params, clockwise=(cmd == 'G2'))
        elif cmd == 'G28':
            self.home()
        elif cmd == 'G90':
            self.absolute_mode = True
            self.get_logger().info('→ Absolute mode')
        elif cmd == 'G91':
            self.absolute_mode = False
            self.get_logger().info('→ Relative mode')
        elif cmd == 'G20':
            self.units = 25.4
            self.get_logger().info('→ Units: inches')
        elif cmd == 'G21':
            self.units = 1.0
            self.get_logger().info('→ Units: mm')
        elif cmd == 'M114':
            self.get_logger().info(
                f'→ Position: X:{self.current_pos[0]:.2f} '
                f'Y:{self.current_pos[1]:.2f} Z:{self.current_pos[2]:.2f}'
            )
    
    def generate_linear_waypoints(self, start, end):
        """Generate interpolated waypoints for linear move"""
        distance = np.linalg.norm(end - start)
        
        if distance < 0.5:
            return [start, end]
        
        num_points = max(int(distance * 2), 5)
        num_points = min(num_points, 100)
        
        waypoints = []
        for i in range(num_points):
            alpha = i / (num_points - 1)
            point = start + alpha * (end - start)
            waypoints.append(point)
        
        return waypoints
    
    def linear_move(self, params):
        """Execute G0/G1 linear move"""
        target = self.current_pos.copy()
        
        if 'X' in params:
            target[0] = (params['X'] * self.units) if self.absolute_mode \
                       else self.current_pos[0] + (params['X'] * self.units)
        if 'Y' in params:
            target[1] = (params['Y'] * self.units) if self.absolute_mode \
                       else self.current_pos[1] + (params['Y'] * self.units)
        if 'Z' in params:
            target[2] = (params['Z'] * self.units) if self.absolute_mode \
                       else self.current_pos[2] + (params['Z'] * self.units)
        
        if 'F' in params:
            self.feedrate = params['F'] * self.units
        
        waypoints = self.generate_linear_waypoints(self.current_pos, target)
        self.interpolated_move(waypoints, target)
    
    def arc_move(self, params, clockwise=True):
        """Execute G2/G3 arc move"""
        target = self.current_pos.copy()
        
        if 'X' in params:
            target[0] = (params['X'] * self.units) if self.absolute_mode \
                       else self.current_pos[0] + (params['X'] * self.units)
        if 'Y' in params:
            target[1] = (params['Y'] * self.units) if self.absolute_mode \
                       else self.current_pos[1] + (params['Y'] * self.units)
        if 'Z' in params:
            target[2] = (params['Z'] * self.units) if self.absolute_mode \
                       else self.current_pos[2] + (params['Z'] * self.units)
        
        I = (params['I'] * self.units) if 'I' in params else 0.0
        J = (params['J'] * self.units) if 'J' in params else 0.0
        
        center = self.current_pos[:2] + np.array([I, J])
        
        if 'F' in params:
            self.feedrate = params['F'] * self.units
        
        waypoints = self.generate_arc(self.current_pos, target, center, clockwise)
        self.interpolated_move(waypoints, target)
    
    def generate_arc(self, start, end, center, clockwise):
        """Generate waypoints for arc"""
        start_angle = np.arctan2(start[1] - center[1], start[0] - center[0])
        end_angle = np.arctan2(end[1] - center[1], end[0] - center[0])
        
        radius = np.linalg.norm(start[:2] - center)
        
        angle_diff = end_angle - start_angle
        
        if clockwise:
            if angle_diff > 0:
                angle_diff -= 2 * np.pi
        else:
            if angle_diff < 0:
                angle_diff += 2 * np.pi
        
        arc_length = abs(angle_diff * radius)
        num_points = max(int(arc_length * 2), 10)
        num_points = min(num_points, 200)
        waypoints = []
        
        for i in range(num_points):
            alpha = i / (num_points - 1)
            angle = start_angle + alpha * angle_diff
            
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = start[2] + alpha * (end[2] - start[2])
            
            waypoints.append(np.array([x, y, z]))
        
        return waypoints
    
    def interpolated_move(self, waypoints, target):
        """Execute move with position-based completion tracking"""
        if len(waypoints) < 2:
            return
        
        self.publish_trajectory_marker(waypoints)
        
        total_distance = 0.0
        for i in range(len(waypoints) - 1):
            total_distance += np.linalg.norm(waypoints[i+1] - waypoints[i])
        
        if total_distance < 0.5:
            return
        
        time_seconds = (total_distance / self.feedrate) * 60.0
        time_seconds = max(time_seconds, 0.1)
        
        self.get_logger().info(
            f'  → {total_distance:.1f}mm in {time_seconds:.2f}s @ F{self.feedrate:.0f}'
        )
        
        msg = JointTrajectory()
        msg.joint_names = ['jbf1', 'jbf2', 'jbf3']
        
        try:
            for i, wp in enumerate(waypoints):
                wp_m = wp / 1000.0
                
                target_position = np.array([[wp_m[0]], [wp_m[1]], [wp_m[2]]])
                target_frame = Frame.from_euler_3(np.array([0., 0., 0.]), target_position)
                joint_angles = self.robot.inverse(target_frame).flatten()
                
                point = JointTrajectoryPoint()
                point.positions = [float(joint_angles[0]), 
                                 float(joint_angles[1]), 
                                 float(joint_angles[2])]
                
                alpha = i / (len(waypoints) - 1)
                point_time = alpha * time_seconds
                point.time_from_start = Duration(
                    sec=int(point_time), 
                    nanosec=int((point_time % 1) * 1e9)
                )
                
                if i < len(waypoints) - 1:
                    segment_dist = np.linalg.norm(waypoints[i+1] - waypoints[i])
                    segment_time = (segment_dist / self.feedrate) * 60.0
                    velocity = segment_dist / segment_time if segment_time > 0 else 0
                    point.velocities = [velocity / 100.0] * 3
                else:
                    point.velocities = [0.0, 0.0, 0.0]
                
                msg.points.append(point)
            
            self.joint_pub.publish(msg)
            
            # Set movement state with position-based tracking
            self.is_moving = True
            self.target_position = target.copy()
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.min_move_time = current_time + (time_seconds * 0.8)  # Wait 80% of expected time
            
        except Exception as e:
            self.get_logger().error(f'IK failed: {e}')
            self.is_moving = False
    
    def home(self):
        """Move to home position"""
        self.get_logger().info('Homing...')
        target = np.array([0.0, 0.0, -250.0])
        waypoints = self.generate_linear_waypoints(self.current_pos, target)
        self.interpolated_move(waypoints, target)

def main(args=None):
    rclpy.init(args=args)
    node = DeltaGcodeInterpreter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
