"""Launch file for klask sprite generator system.

This launch file is used to drive the pegs around the board, capture video,
and extract sprites of the pegs and ball for later usage in the renderer.

Example usage:
    ros2 launch klask_sprite_generator_pkg sprite_generator_launch.py
"""

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate the launch description for the klask sprite generator system."""
    sprite_pkg_share = get_package_share_directory("klask_sprite_generator_pkg")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(sprite_pkg_share, "config", "sprite_generator_params.yaml"),
        description="Path to sprite generator parameter file",
    )

    # Get path to motors launch file
    motor_pkg_share = get_package_share_directory("klask_motor_commander_pkg")
    motors_launch_file = os.path.join(motor_pkg_share, "launch", "motors_launch.py")

    # Include motors launch file with both players, camera, and viewer enabled
    motors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(motors_launch_file),
        launch_arguments={
            "player": "both",
            "start_commander": "true",
            "start_camera": "true",
            "start_viewer": "true",
        }.items(),
    )

    # Sprite generator node
    sprite_generator_node = Node(
        package="klask_sprite_generator_pkg",
        executable="klask_sprite_generator_node",
        name="sprite_generator_node",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    return LaunchDescription([params_file_arg, motors_launch, sprite_generator_node])
