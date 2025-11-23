import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Setup Paths ---
    delta_robot_description_path = get_package_share_directory('delta_robot_description')
    delta_robot_sim_path = get_package_share_directory('delta_robot_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=str(Path(delta_robot_description_path).parent.resolve())
    )

    # Load SDF
    sdf_file = os.path.join(delta_robot_description_path, 'models', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup World
    world_file = os.path.join(delta_robot_sim_path, 'worlds', 'empty.sdf')
    
    # Path to RViz config (we will create this in Step 2)

    # --- 2. Launch Gazebo ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_file}'
        }.items(),
    )

    # --- 3. Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # --- 4. Spawn Entity ---
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-name', 'delta_robot',
            '-allow_renaming', 'false',
            '-x', '0.0', '-y', '0.0', '-z', '0.50',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ],
    )

    # --- 5. Bridge ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(delta_robot_sim_path, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # --- 6. RViz2 ---
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # Validates if config file exists, otherwise launches default
        parameters=[{'use_sim_time': True}] # Important for syncing with Gazebo
    )

    return LaunchDescription([
        gazebo_resource_path,
        gz_sim,
        gz_spawn_entity,
        bridge,
        rviz2, 
    ])