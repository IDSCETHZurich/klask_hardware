"""Launch file for camera node with parameter configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for camera node."""

    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("klask_imaging_pkg"), "config", "camera_node_params.yaml"]
        ),
        description="Path to the camera node parameters YAML file",
    )

    # Camera node
    camera_node = Node(
        package="klask_imaging_pkg",
        executable="camera_node",
        name="camera_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        emulate_tty=True,  # Better log formatting
    )

    return LaunchDescription(
        [
            params_file_arg,
            camera_node,
        ]
    )
