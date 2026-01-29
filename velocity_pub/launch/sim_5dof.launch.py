"""
Unified 5DOF Simulation Launch with Conveyor System

Launches:
- Gazebo simulation with delta robot
- Conveyor belt (TrackController)
- Conveyor manager (spawns boxes, controls belt)
- 5DOF controller (IK + interpolation)
- Simulation control bridge (ESP32-like behavior)
- GUI controller (optional)

Usage: ros2 launch velocity_pub sim_5dof.launch.py
       ros2 launch velocity_pub sim_5dof.launch.py gui:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get package paths
    delta_robot_sim = get_package_share_directory('delta_robot_sim')
    velocity_pub = get_package_share_directory('velocity_pub')
    delta_robot_desc = get_package_share_directory('delta_robot_description')
    
    # Launch arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch GUI controller'
    )
    
    conveyor_arg = DeclareLaunchArgument(
        'conveyor',
        default_value='true',
        description='Launch conveyor belt system'
    )
    
    # 1. Include existing Gazebo spawn launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(delta_robot_sim, 'launch', 'delta_robot_spawn.launch.py')
        )
    )
    
    # 2. Spawn Conveyor Belt
    # Belt is 1.5m long, 0.3m wide, 0.05m thick
    # Positioned so belt surface is near robot workspace
    # Belt rotated 90deg (Y=1.5708) so it moves along Y axis
    # Z position: Belt center at Z, surface at Z+0.025
    # For PICK_Z = -0.31, we want belt surface ~ -0.32
    spawn_conveyor = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(delta_robot_desc, 'models', 'conveyor.sdf'),
            '-name', 'conveyor_belt',
            '-x', '0.0',
            '-y', '-0.0170',
            '-z', '-0.0186'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('conveyor'))
    )
    
    # 2b. Spawn Landing Pad (for contact detection)
    # Position at drop zone: PLACE_POS = [0.0, 0.19, -0.24] -> ground below it
    spawn_landing_pad = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(delta_robot_desc, 'models', 'landing_pad.sdf'),
            '-name', 'landing_pad',
            '-x', '0.0',
            '-y', '0.19',
            '-z', '0.0'  # Ground level below drop zone
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('conveyor'))
    )
    
    # 3. Conveyor Manager (spawns boxes, controls belt velocity)
    conveyor_manager = Node(
        package='velocity_pub',
        executable='conveyor_manager.py',
        name='conveyor_manager',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('conveyor'))
    )
    
    # 4. 5DOF Controller
    controller_5dof = Node(
        package='velocity_pub',
        executable='delta_5dof_controller.py',
        name='delta_5dof_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # 5. Simulation Control Bridge (ESP32-like interpolation for Gazebo)
    sim_control = Node(
        package='velocity_pub',
        executable='sim_control.py',
        name='sim_control_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # 6. GUI Controller (optional)
    gui_controller = Node(
        package='velocity_pub',
        executable='gui_controller.py',
        name='gui_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    # Delay conveyor manager to let Gazebo spawn conveyor first
    delayed_conveyor_manager = TimerAction(
        period=3.0,
        actions=[conveyor_manager]
    )
    
    return LaunchDescription([
        gui_arg,
        conveyor_arg,
        gazebo_launch,
        spawn_conveyor,
        spawn_landing_pad,
        controller_5dof,
        sim_control,
        gui_controller,
        delayed_conveyor_manager,
    ])
