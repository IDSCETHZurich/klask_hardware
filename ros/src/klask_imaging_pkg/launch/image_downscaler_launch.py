"""Launch file for image downscaler node with parameter configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for image downscaler node."""
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("klask_imaging_pkg"),
                "config",
                "image_downscaler_params.yaml",
            ]
        ),
        description="Path to the image downscaler node parameters YAML file",
    )

    # Image downscaler node
    image_downscaler_node = Node(
        package="klask_imaging_pkg",
        executable="image_downscaler",
        name="image_downscaler",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            params_file_arg,
            image_downscaler_node,
        ]
    )
