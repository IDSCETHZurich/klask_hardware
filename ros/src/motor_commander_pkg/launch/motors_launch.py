from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    # Get the player parameter value
    player = LaunchConfiguration("player").perform(context)

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

    # Motor commander node
    nodes_to_launch.append(
        Node(
            package="motor_commander_pkg", executable="motor_commander", output="screen"
        )
    )

    # Foxglove bridge (commented out by default)
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

    return LaunchDescription([player_arg, OpaqueFunction(function=launch_setup)])
