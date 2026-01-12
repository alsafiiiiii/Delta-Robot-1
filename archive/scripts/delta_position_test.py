#!/usr/bin/env python3
"""
Simple test node to publish target positions
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.publisher = self.create_publisher(Point, '/delta/target_position', 10)
        self.get_logger().info('Position publisher ready. Use: ros2 topic pub /delta/target_position geometry_msgs/Point ...')


def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
