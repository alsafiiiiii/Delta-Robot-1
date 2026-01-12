import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 3DOF Kinematics Controller (interpolates path)
        Node(
            package='velocity_pub',
            executable='delta_3dof_controller.py',
            name='delta_controller',
            output='screen',
            parameters=[{'use_sim': False}]
        ),
        
        # 2. Hardware Bridge (UDP to ESP32)
        Node(
            package='velocity_pub',
            executable='robot_control.py',
            name='delta_bridge',
            output='screen'
        ),
        
        # 3. Joystick Input
        Node(
            package='velocity_pub',
            executable='joystick_controller.py',
            name='joystick_input',
            output='screen'
        ),
        
        # 4. G-Code Interpreter (Optional, can be run manually if needed)
        # Node(
        #     package='velocity_pub',
        #     executable='delta_gcode_interpreter.py',
        #     name='gcode_interpreter'
        # )
    ])
