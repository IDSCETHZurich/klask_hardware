"""Launch file for the velocity profile system identification experiment.

Launches the motor control system (CAN, ODrive, motor commander with homing),
camera node, and the velocity profile node. The test node manages rosbag
recording internally, writing one bag per run to bag_output_dir.

This launch file builds the motor system nodes directly (rather than including
motors_launch.py) so that max_velocity can be overridden for the test.

Example usage:
    ros2 launch klask_system_id velocity_profile_launch.py
    ros2 launch klask_system_id velocity_profile_launch.py player:=right pattern:=circle
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup function to resolve launch arguments and build node list."""
    player = LaunchConfiguration("player").perform(context)
    params_file_path = LaunchConfiguration("params_file").perform(context)
    with open(params_file_path) as f:
        profile_params = yaml.safe_load(f)
    max_velocity = float(profile_params["/velocity_profile_node"]["ros__parameters"]["v_max"])

    nodes_to_launch = []

    # --- Motor system config files ---
    motor_pkg_share = get_package_share_directory("klask_motor_commander_pkg")
    player_params_file = os.path.join(motor_pkg_share, "config", "player_params.yaml")
    open_loop_params_file = os.path.join(motor_pkg_share, "config", "open_loop_controller_params.yaml")
    odrive_params_file = os.path.join(motor_pkg_share, "config", "odrive_controller_params.yaml")
    motor_commander_params_file = os.path.join(motor_pkg_share, "config", "motor_commander_params.yaml")

    # --- Camera node ---
    imaging_pkg_share = get_package_share_directory("klask_imaging_pkg")
    camera_params_file = os.path.join(imaging_pkg_share, "config", "camera_node_params.yaml")
    nodes_to_launch.append(
        Node(
            package="klask_imaging_pkg",
            executable="camera_node",
            name="camera_node",
            parameters=[camera_params_file],
            output="screen",
        )
    )

    # --- CAN interface setup ---
    nodes_to_launch.append(ExecuteProcess(cmd=["ip", "link", "set", "can0", "down"], output="screen"))
    nodes_to_launch.append(
        ExecuteProcess(
            cmd=[
                "ip",
                "link",
                "set",
                "can0",
                "up",
                "type",
                "can",
                "bitrate",
                "250000",
            ],
            output="screen",
        )
    )

    # --- ODrive CAN nodes ---
    if player in ["right", "both"]:
        for axis_id in [0, 1]:
            nodes_to_launch.append(
                Node(
                    package="odrive_can",
                    executable="odrive_can_node",
                    name=f"can_node_{axis_id}",
                    namespace=f"odrive_axis{axis_id}",
                    parameters=[
                        {
                            "node_id": axis_id,
                            "interface": "can0",
                            "axis_idle_on_shutdown": True,
                        }
                    ],
                    output="screen",
                )
            )

    if player in ["left", "both"]:
        for axis_id in [2, 3]:
            nodes_to_launch.append(
                Node(
                    package="odrive_can",
                    executable="odrive_can_node",
                    name=f"can_node_{axis_id}",
                    namespace=f"odrive_axis{axis_id}",
                    parameters=[
                        {
                            "node_id": axis_id,
                            "interface": "can0",
                            "axis_idle_on_shutdown": True,
                        }
                    ],
                    output="screen",
                )
            )

    # --- Motor commander node (with max_velocity override) ---
    nodes_to_launch.append(
        Node(
            package="klask_motor_commander_pkg",
            executable="klask_motor_commander",
            parameters=[
                player_params_file,
                open_loop_params_file,
                odrive_params_file,
                motor_commander_params_file,
                {
                    "player": player,
                    "max_velocity": max_velocity,
                },
            ],
            output="screen",
        )
    )

    # --- Foxglove bridge ---
    nodes_to_launch.append(
        ExecuteProcess(
            cmd=[
                "ros2",
                "launch",
                "foxglove_bridge",
                "foxglove_bridge_launch.xml",
            ],
            output="log",
        )
    )

    # --- Velocity profile node ---
    nodes_to_launch.append(
        Node(
            package="klask_system_id",
            executable="velocity_profile_node",
            name="velocity_profile_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
            emulate_tty=True,
        )
    )

    return nodes_to_launch


def generate_launch_description():
    """Generate launch description for the velocity profile test."""
    player_arg = DeclareLaunchArgument(
        "player",
        default_value="both",
        description='Which player(s) to drive: "left", "right", or "both"',
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("klask_system_id"),
                "config",
                "velocity_profile_params.yaml",
            ]
        ),
        description="Path to the velocity profile parameters YAML file",
    )

    return LaunchDescription(
        [
            player_arg,
            params_file_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
