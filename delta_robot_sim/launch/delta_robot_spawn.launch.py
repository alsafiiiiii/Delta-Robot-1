# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
from pathlib import Path


from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Configure ROS nodes for launch


    # Setup project paths
    delta_robot_description_path = get_package_share_directory('delta_robot_description')
    delta_robot_sim_path = get_package_share_directory('delta_robot_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')


    # Set Gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=str(Path(delta_robot_description_path).parent.resolve())
    )


    # Load the SDF file from "description" package
    sdf_file = os.path.join(delta_robot_description_path, 'models', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()


    # Setup to launch the simulator and Gazebo world
    world_file = os.path.join(delta_robot_sim_path, 'worlds', 'empty.sdf')
    
    # Custom GUI config file path
    gui_config_file = os.path.join(delta_robot_sim_path, 'config', 'gui.config')


    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': world_file + ' -v 4 -r '#--physics-engine gz-physics-bullet-featherstone-plugin
        }.items(),
    )
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[sdf_file],
        output=['screen']
    )
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




    # Spawn the robot in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-name', 'delta_robot',
            '-allow_renaming', 'false'
        ],
    )




    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(delta_robot_sim_path, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )



    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world file, e.g., empty'
    )
    


    return LaunchDescription([
        gazebo_resource_path,
        # joint_state_publisher_gui,
        # robot_state_publisher,
        world_arg,
        gz_sim,
        gz_spawn_entity,
        bridge,       
    ])
