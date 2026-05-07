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
    delta_robot_sim = get_package_share_directory("delta_robot_sim")
    delta_hardware_bridge = get_package_share_directory("delta_hardware_bridge")
    delta_robot_gui = get_package_share_directory("delta_robot_gui")

    # Launch arguments
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true", description="Launch GUI controller"
    )

    # 1. Gazebo Simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(delta_robot_sim, "launch", "delta_robot_spawn.launch.py")
        )
    )

    # 2. 5DOF Kinematics Controller
    controller_5dof = Node(
        package="delta_controllers",
        executable="5dof_controller_node",
        name="delta_5dof_controller",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # 3. Master Hardware Node (SDK Master)
    motor_node = Node(
        package="delta_hardware_bridge",
        executable="motor_control_node",
        name="motor_control_node",
        output="screen",
        parameters=[
            {"device_name": "/dev/ttyACM0"},
            {"baudrate": 1000000},
            {"moving_speed": 0},
            {"moving_acc": 0},
        ],
    )

    # 4. GUI Controller
    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(delta_robot_gui, "launch", "delta_robot_gui.launch.py")
        ),
        launch_arguments={
            "sim_mode": "false",
            "joint_command_topic": "/delta/joint_commands",
        }.items(),
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    return LaunchDescription(
        [gui_arg, gazebo_launch, controller_5dof, motor_node, gui_launch]
    )
