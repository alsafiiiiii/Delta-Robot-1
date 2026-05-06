from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_pose_topic = LaunchConfiguration("target_pose_topic")
    speed_topic = LaunchConfiguration("speed_topic")
    joint_command_topic = LaunchConfiguration("joint_command_topic")
    joint_state_topic = LaunchConfiguration("joint_state_topic")
    torque_command_topic = LaunchConfiguration("torque_command_topic")
    sim_mode = LaunchConfiguration("sim_mode")
    qt_qpa_platform = LaunchConfiguration("qt_qpa_platform")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target_pose_topic",
                default_value="/delta/target_pose",
                description="Topic used for cartesian pose targets",
            ),
            DeclareLaunchArgument(
                "speed_topic",
                default_value="/delta/speed_params",
                description="Topic used for speed parameters",
            ),
            DeclareLaunchArgument(
                "joint_command_topic",
                default_value="/joint_trajectory_controller/joint_trajectory",
                description="Topic used for joint command trajectories",
            ),
            DeclareLaunchArgument(
                "joint_state_topic",
                default_value="/joint_states",
                description="Topic used for joint feedback",
            ),
            DeclareLaunchArgument(
                "torque_command_topic",
                default_value="delta_motors/torque_command",
                description="Topic used for torque enable/disable commands",
            ),
            DeclareLaunchArgument(
                "sim_mode",
                default_value="true",
                description="Run GUI in simulation mode (no motor torque commands)",
            ),
            DeclareLaunchArgument(
                "qt_qpa_platform",
                default_value="wayland",
                description="Qt platform plugin used by the GUI process",
            ),
            Node(
                package="delta_robot_gui",
                executable="delta_robot_gui",
                name="delta_robot_gui",
                output="screen",
                additional_env={"QT_QPA_PLATFORM": qt_qpa_platform},
                parameters=[
                    {"target_pose_topic": target_pose_topic},
                    {"speed_topic": speed_topic},
                    {"joint_command_topic": joint_command_topic},
                    {"joint_state_topic": joint_state_topic},
                    {"torque_command_topic": torque_command_topic},
                    {"sim_mode": sim_mode},
                    {"joint_names": [
                        "motor_joint_1",
                        "motor_joint_2",
                        "motor_joint_3",
                        "motor_joint_4",
                        "motor_joint_5",
                    ]},
                    {"motor_ids": [1, 2, 3, 4, 5]},
                ],
            ),
        ]
    )
