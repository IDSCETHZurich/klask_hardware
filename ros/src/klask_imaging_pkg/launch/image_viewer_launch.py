"""Launch file for image viewer node with parameter configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for image viewer node."""

    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("klask_imaging_pkg"),
                "config",
                "image_viewer_params.yaml",
            ]
        ),
        description="Path to the image viewer node parameters YAML file",
    )

    # Image viewer node
    image_viewer_node = Node(
        package="klask_imaging_pkg",
        executable="image_viewer",
        name="image_viewer",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        emulate_tty=True,  # Better log formatting
    )

    return LaunchDescription(
        [
            params_file_arg,
            image_viewer_node,
        ]
    )
