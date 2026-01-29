"""
Unified 5DOF Simulation Launch

Launches:
- Gazebo simulation with delta robot
- 5DOF controller (IK + interpolation)
- Simulation control bridge (ESP32-like behavior)
- GUI controller (optional)

Usage: ros2 launch velocity_pub sim_5dof.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get package paths
    delta_robot_sim = get_package_share_directory('delta_robot_sim')
    velocity_pub = get_package_share_directory('velocity_pub')
    
    # Launch arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch GUI controller'
    )
    
    # 1. Include existing Gazebo spawn launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(delta_robot_sim, 'launch', 'delta_robot_spawn.launch.py')
        )
    )
    
    # 2. 5DOF Controller
    controller_5dof = Node(
        package='velocity_pub',
        executable='delta_5dof_controller.py',
        name='delta_5dof_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # 3. Simulation Control Bridge (ESP32-like interpolation for Gazebo)
    sim_control = Node(
        package='velocity_pub',
        executable='sim_control.py',
        name='sim_control_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # 4. GUI Controller (optional)
    gui_controller = Node(
        package='velocity_pub',
        executable='gui_controller.py',
        name='gui_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    return LaunchDescription([
        gui_arg,
        gazebo_launch,
        controller_5dof,
        sim_control,
        gui_controller,
    ])
