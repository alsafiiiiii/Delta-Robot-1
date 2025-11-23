import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Find the path to your xacro file
    pkg_path = get_package_share_directory('delta_robot_description')
    xacro_file = '/home/rikisu/major_project_ws/src/delta_robot_description/models/delta_robot.xacro'

    # 2. Process the Xacro file using 'Command'
    # This runs the "xacro" command line tool and captures the output XML
    robot_description_config = Command(['xacro ', xacro_file])
    
    # 3. Create the robot_state_publisher node
    # This node publishes the processed URDF to the /robot_description topic
    params = {'robot_description': robot_description_config}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 4. Spawn the robot in Gazebo using the topic
    # We tell Gazebo to listen to /robot_description for the XML
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'delta_robot',
                   '-z', '0.5'], # Spawn slightly above ground
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        spawn_entity
    ])