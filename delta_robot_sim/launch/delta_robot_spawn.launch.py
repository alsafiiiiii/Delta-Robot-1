import os
import math
from pathlib import Path
from setuptools import Command
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
import xacro

def generate_launch_description():
    # --- 1. Setup Paths ---
    delta_robot_description_path = get_package_share_directory(
        "delta_robot_description"
    )
    delta_robot_sim_path = get_package_share_directory("delta_robot_sim")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(Path(delta_robot_description_path).parent.resolve()),
    )

    # Gazebo plugin path
    gazebo_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value="/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/",
    )

    use_sim_feedback_arg = DeclareLaunchArgument(
        "use_sim_feedback",
        default_value="true",
        description="Set false when real motors are running to avoid feedback conflict",
    )

    # Load SDF
    xacro_file = os.path.join(delta_robot_description_path, "models", "model_high.sdf.xacro")

    # Process the XACRO file right now and convert it into a standard Python string
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # Load Box SDF
    box_sdf_file = os.path.join(delta_robot_description_path, "models", "box.sdf")
    with open(box_sdf_file, "r") as infp:
        infp.read()

    # Pre-process SDF to replace $(find delta_robot_sim) with actual path
    # because standard SDF parser doesn't resolve $(find ...)
    robot_desc = robot_desc.replace("$(find delta_robot_sim)", delta_robot_sim_path)

    # Setup World
    world_file = os.path.join(delta_robot_sim_path, "worlds", "empty.sdf")

    # Path to RViz config (we will create this in Step 2)

    # --- 2. Launch Gazebo ---
    gz_gui_config = os.path.join(delta_robot_sim_path, "config", "config.config")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -v 1 --gui-config {gz_gui_config} {world_file} --physics-engine gz-physics-bullet-featherstone-plugin"
        }.items(),
    )

    # --- 3. Robot State Publisher ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": False},
            {"robot_description": robot_desc},
        ],
        remappings=[
            ("/tf", "/tf_rsp_ignore"),
            ("/tf_static", "/tf_static_rsp_ignore"),
        ],
    )

    # --- 4. Spawn Entity ---
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_desc,
            "-name",
            "delta_robot",
            "-allow_renaming",
            "false",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.50",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            str(math.pi),
        ],
    )

    # --- 5. Bridge ---
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    delta_robot_sim_path, "config", "ros_gz_bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )


    # --- 6. RViz2 ---
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(delta_robot_sim_path, "config", "delta_robot.rviz"),
        ],
        parameters=[{"use_sim_time": False}],
    )

    # --- 7. ROS2 Control Spawners ---
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    load_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_feedback_arg,
            gazebo_resource_path,
            gazebo_plugin_path,
            gz_sim,
            gz_spawn_entity,
            bridge,
            robot_state_publisher,
            rviz2,
            load_joint_state_broadcaster,
            load_joint_trajectory_controller,
        ]
    )
