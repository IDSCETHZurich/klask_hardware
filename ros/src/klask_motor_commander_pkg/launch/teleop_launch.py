"""Launch file for keyboard teleoperation control.

Example usage:
    # Control left player (default):
    ros2 launch klask_motor_commander_pkg teleop_launch.py

    # Control right player:
    ros2 launch klask_motor_commander_pkg teleop_launch.py player:=right

    # Control left player (explicit):
    ros2 launch klask_motor_commander_pkg teleop_launch.py player:=left

Note: Requires xterm to be installed for keyboard input window.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Get the player parameter value
    player = LaunchConfiguration("player").perform(context)

    # Determine topic based on player selection
    if player == "left":
        topic = "/cmd_vel/left_player"
        node_name = "teleop_left_player"
    else:  # Default to right
        topic = "/cmd_vel/right_player"
        node_name = "teleop_right_player"

    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name=node_name,
        output="screen",
        prefix="xterm -e",
        remappings=[("/cmd_vel", topic)],
    )

    return [teleop_node]


def generate_launch_description():
    player_arg = DeclareLaunchArgument(
        "player",
        default_value="left",
        description='Which player to control: "left" or "right" (default)',
    )

    return LaunchDescription([player_arg, OpaqueFunction(function=launch_setup)])
