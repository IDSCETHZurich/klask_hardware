"""Launch file for klask motor control system.

Example usage:
    # Launch both players with motor commander:
    ros2 launch klask_motor_commander_pkg motors_launch.py

    # Launch only right player:
    ros2 launch klask_motor_commander_pkg motors_launch.py player:=right

    # Launch only left player:
    ros2 launch klask_motor_commander_pkg motors_launch.py player:=left

    # Launch ODrive nodes without motor commander:
    ros2 launch klask_motor_commander_pkg motors_launch.py start_commander:=false

    # Launch right player only without commander:
    ros2 launch klask_motor_commander_pkg motors_launch.py player:=right start_commander:=false

    # Launch with camera and image viewer:
    ros2 launch klask_motor_commander_pkg motors_launch.py start_camera:=true start_viewer:=true
"""

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """Setup function to launch nodes based on launch arguments."""
    # Get the player parameter value
    player = LaunchConfiguration("player").perform(context)
    start_commander = LaunchConfiguration("start_commander").perform(context)
    start_camera = LaunchConfiguration("start_camera").perform(context)
    start_viewer = LaunchConfiguration("start_viewer").perform(context)
    start_downscaler = LaunchConfiguration("start_downscaler").perform(context)

    # Get path to parameter file
    pkg_share = get_package_share_directory("klask_motor_commander_pkg")
    player_params_file = os.path.join(pkg_share, "config", "player_params.yaml")
    open_loop_params_file = os.path.join(pkg_share, "config", "open_loop_controller_params.yaml")
    odrive_params_file = os.path.join(pkg_share, "config", "odrive_controller_params.yaml")
    motor_commander_params_file = os.path.join(pkg_share, "config", "motor_commander_params.yaml")

    # Get imaging package config files
    imaging_pkg_share = get_package_share_directory("klask_imaging_pkg")
    camera_params_file = os.path.join(imaging_pkg_share, "config", "camera_node_params.yaml")
    image_viewer_params_file = os.path.join(imaging_pkg_share, "config", "image_viewer_params.yaml")
    downscaler_params_file = os.path.join(imaging_pkg_share, "config", "image_downscaler_params.yaml")

    nodes_to_launch = []

    # Camera node
    if start_camera == "true":
        nodes_to_launch.append(
            Node(
                package="klask_imaging_pkg",
                executable="camera_node",
                name="camera_node",
                parameters=[camera_params_file],
                output="screen",
            )
        )

    if start_downscaler == "true":
        nodes_to_launch.append(
            Node(
                package="klask_imaging_pkg",
                executable="image_downscaler",
                name="image_downscaler",
                parameters=[downscaler_params_file],
                output="screen",
            )
        )

    # CAN interface setup
    nodes_to_launch.append(ExecuteProcess(cmd=["ip", "link", "set", "can0", "down"], output="screen"))

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
                parameters=[{"node_id": 0, "interface": "can0", "axis_idle_on_shutdown": True}],
                output="screen",
            )
        )

        nodes_to_launch.append(
            Node(
                package="odrive_can",
                executable="odrive_can_node",
                name="can_node_1",
                namespace="odrive_axis1",
                parameters=[{"node_id": 1, "interface": "can0", "axis_idle_on_shutdown": True}],
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
                parameters=[{"node_id": 2, "interface": "can0", "axis_idle_on_shutdown": True}],
                output="screen",
            )
        )

        nodes_to_launch.append(
            Node(
                package="odrive_can",
                executable="odrive_can_node",
                name="can_node_3",
                namespace="odrive_axis3",
                parameters=[{"node_id": 3, "interface": "can0", "axis_idle_on_shutdown": True}],
                output="screen",
            )
        )

    # Motor commander node (optional, controlled by start_commander parameter)
    if start_commander == "true":
        motor_commander_node = Node(
            package="klask_motor_commander_pkg",
            executable="klask_motor_commander",
            parameters=[
                player_params_file,
                open_loop_params_file,
                odrive_params_file,
                motor_commander_params_file,
                {"player": player},  # Pass player selection from launch argument
            ],
            output="screen",
        )
        nodes_to_launch.append(motor_commander_node)

        # Shut down the entire launch when the motor commander exits
        nodes_to_launch.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=motor_commander_node,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            )
        )

    # Foxglove bridge
    nodes_to_launch.append(
        ExecuteProcess(
            cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"],
            output="log",
        )
    )

    # Image viewer node
    if start_viewer == "true":
        nodes_to_launch.append(
            Node(
                package="klask_imaging_pkg",
                executable="image_viewer",
                name="image_viewer",
                parameters=[image_viewer_params_file],
                output="screen",
            )
        )

    return nodes_to_launch


def generate_launch_description():
    """Generate the launch description for the klask motor commander system."""
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

    start_camera_arg = DeclareLaunchArgument(
        "start_camera",
        default_value="false",
        description='Whether to start the camera node: "true" or "false" (default)',
    )

    start_viewer_arg = DeclareLaunchArgument(
        "start_viewer",
        default_value="false",
        description='Whether to start the image viewer node: "true" or "false" (default)',
    )

    start_downscaler_arg = DeclareLaunchArgument(
        "start_downscaler",
        default_value="false",
        description='Whether to add the downscaler node: "true" or "false (default)"',
    )

    return LaunchDescription(
        [
            player_arg,
            start_commander_arg,
            start_camera_arg,
            start_viewer_arg,
            start_downscaler_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
