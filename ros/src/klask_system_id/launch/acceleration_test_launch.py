"""Launch file for the acceleration test system identification experiment.

Launches the motor control system (CAN, ODrive, motor commander with homing),
camera node, and the acceleration test node. The test node manages rosbag
recording internally, writing one bag per velocity step to bag_output_dir.

This launch file builds the motor system nodes directly (rather than including
motors_launch.py) so that max_velocity can be overridden for high-speed tests.

Example usage:
    ros2 launch klask_system_id acceleration_test_launch.py
    ros2 launch klask_system_id acceleration_test_launch.py player:=right
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup function to resolve launch arguments and build node list."""
    player = LaunchConfiguration("player").perform(context)
    max_velocity = float(LaunchConfiguration("max_velocity").perform(context))

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

    # --- State estimator node (motor commander needs it for calibration) ---
    estimator_pkg_share = get_package_share_directory("klask_state_estimator_pkg")
    estimator_params_file = os.path.join(estimator_pkg_share, "config", "state_estimator_params.yaml")
    nodes_to_launch.append(
        Node(
            package="klask_state_estimator_pkg",
            executable="state_estimator",
            name="state_estimator",
            parameters=[estimator_params_file],
            output="screen",
            emulate_tty=True,
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

    # --- Acceleration test node ---
    nodes_to_launch.append(
        Node(
            package="klask_system_id",
            executable="acceleration_test_node",
            name="acceleration_test_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
            emulate_tty=True,
        )
    )

    return nodes_to_launch


def generate_launch_description():
    """Generate launch description for the acceleration test."""
    player_arg = DeclareLaunchArgument(
        "player",
        default_value="left",
        description='Which player to test: "left" or "right"',
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("klask_system_id"),
                "config",
                "acceleration_test_params.yaml",
            ]
        ),
        description="Path to the acceleration test parameters YAML file",
    )

    max_velocity_arg = DeclareLaunchArgument(
        "max_velocity",
        default_value="1.0",
        description="Override max_velocity for the motor commander (m/s)",
    )

    return LaunchDescription(
        [
            player_arg,
            params_file_arg,
            max_velocity_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
