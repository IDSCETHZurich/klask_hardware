from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    # Get the player parameter value
    player = LaunchConfiguration("player").perform(context)
    start_commander = LaunchConfiguration("start_commander").perform(context)

    # Get path to parameter file
    pkg_share = get_package_share_directory("klask_motor_commander_pkg")
    player_params_file = os.path.join(pkg_share, "config", "player_params.yaml")
    open_loop_params_file = os.path.join(
        pkg_share, "config", "open_loop_controller_params.yaml"
    )
    odrive_params_file = os.path.join(
        pkg_share, "config", "odrive_controller_params.yaml"
    )
    motor_commander_params_file = os.path.join(
        pkg_share, "config", "motor_commander_params.yaml"
    )

    nodes_to_launch = []

    # CAN interface setup (commented out by default)
    nodes_to_launch.append(
        ExecuteProcess(cmd=["ip", "link", "set", "can0", "down"], output="screen")
    )

    nodes_to_launch.append(
        ExecuteProcess(
            cmd=["ip", "link", "set", "can0", "up", "type", "can", "bitrate", "250000"],
            output="screen",
        )
    )

    # Right player ODrive nodes (axis 0 and 1)
    if player in ["right", "both"]:
        nodes_to_launch.append(
            Node(
                package="odrive_can",
                executable="odrive_can_node",
                name="can_node_0",
                namespace="odrive_axis0",
                parameters=[
                    {"node_id": 0, "interface": "can0", "axis_idle_on_shutdown": True}
                ],
                output="screen",
            )
        )

        nodes_to_launch.append(
            Node(
                package="odrive_can",
                executable="odrive_can_node",
                name="can_node_1",
                namespace="odrive_axis1",
                parameters=[
                    {"node_id": 1, "interface": "can0", "axis_idle_on_shutdown": True}
                ],
                output="screen",
            )
        )

    # Left player ODrive nodes (axis 2 and 3)
    if player in ["left", "both"]:
        nodes_to_launch.append(
            Node(
                package="odrive_can",
                executable="odrive_can_node",
                name="can_node_2",
                namespace="odrive_axis2",
                parameters=[
                    {"node_id": 2, "interface": "can0", "axis_idle_on_shutdown": True}
                ],
                output="screen",
            )
        )

        nodes_to_launch.append(
            Node(
                package="odrive_can",
                executable="odrive_can_node",
                name="can_node_3",
                namespace="odrive_axis3",
                parameters=[
                    {"node_id": 3, "interface": "can0", "axis_idle_on_shutdown": True}
                ],
                output="screen",
            )
        )

    # Motor commander node (optional, controlled by start_commander parameter)
    if start_commander == "true":
        nodes_to_launch.append(
            Node(
                package="klask_motor_commander_pkg",
                executable="klask_motor_commander",
                parameters=[
                    player_params_file,
                    open_loop_params_file,
                    odrive_params_file,
                    motor_commander_params_file,
                ],
                output="screen",
            )
        )

    # Foxglove bridge
    nodes_to_launch.append(
        ExecuteProcess(
            cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"],
            output="log",
        )
    )

    return nodes_to_launch


def generate_launch_description():
    # Declare launch arguments
    player_arg = DeclareLaunchArgument(
        "player",
        default_value="both",
        description='Which player to launch: "left", "right", or "both" (default)',
    )

    start_commander_arg = DeclareLaunchArgument(
        "start_commander",
        default_value="true",
        description='Whether to start the motor commander node: "true" (default) or "false"',
    )

    return LaunchDescription(
        [player_arg, start_commander_arg, OpaqueFunction(function=launch_setup)]
    )
