from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('velocity_pub')
    
    # Arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use Gazebo simulation (True) or Physical Hardware (False)'
    )
    
    gcode_file_arg = DeclareLaunchArgument(
        'gcode_file',
        default_value='',
        description='Path to G-Code file to execute (Optional)'
    )

    use_sim = LaunchConfiguration('use_sim')
    gcode_file = LaunchConfiguration('gcode_file')

    # 1. Hardware Bridge (Only if NOT Sim)
    delta_bridge = Node(
        package='velocity_pub',
        executable='delta_bridge.py',
        name='delta_bridge',
        condition=UnlessCondition(use_sim),
        output='screen'
    )

    # 2. Controller (Runs in both, but adapts via parameter)
    delta_controller = Node(
        package='velocity_pub',
        executable='delta_3dof_controller.py',
        name='delta_3dof_controller',
        output='screen',
        parameters=[{'use_sim': use_sim}]
    )

    # 3. G-Code Interpreter (Runs in both)
    # Only launch if a file is provided? Or always launch but wait?
    # For now, let's always launch it, it handles "no file" gracefully or we can make it conditional.
    # The current interpreter requires a file arg.
    # Let's make it conditional on the arg being non-empty? 
    # Launch doesn't easily support "if string not empty".
    # User can run it separately if they want.
    # We will exclude it from here to avoid "file not found" errors on startup
    # unless we want to force a default.
    # Let's LEAVE IT OUT for now, user typically runs gcode manually.
    # OR: We can launch it if the user provides a file.
    
    # Let's keep it simple: Bridge + Controller.
    
    return LaunchDescription([
        use_sim_arg,
        gcode_file_arg,
        delta_bridge,
        delta_controller
    ])
