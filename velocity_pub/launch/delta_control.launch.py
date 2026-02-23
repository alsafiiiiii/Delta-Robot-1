import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Launch Arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Run in simulation mode (true) or real robot mode (false)'
    )
    
    use_sim = LaunchConfiguration('use_sim')

    return LaunchDescription([
        use_sim_arg,
        
        # 1. 3DOF Kinematics Controller (interpolates path)
        Node(
            package='velocity_pub',
            executable='delta_3dof_controller.py',
            name='delta_controller',
            output='screen',
            parameters=[{'use_sim': use_sim}]
        ),
        
        # 2. Hardware Bridge (UDP to ESP32) - REAL ROBOT ONLY
        Node(
            package='velocity_pub',
            executable='robot_control.py',
            name='delta_bridge',
            output='screen',
            condition=UnlessCondition(use_sim)
        ),
        
        # 3. Joystick Input
        Node(
            package='velocity_pub',
            executable='joystick_controller.py',
            name='joystick_input',
            output='screen'
        ),
    ])
