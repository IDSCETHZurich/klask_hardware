from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    player_arg = DeclareLaunchArgument(
        "player",
        default_value="both",
        description='Which player to launch: "left", "right", or "both" (default)',
    )

    # Get launch configuration
    player = LaunchConfiguration("player")

    # Helper conditions
    launch_left = IfCondition(
        "'"
        + LaunchConfiguration("player")
        + "' == 'left' or '"
        + LaunchConfiguration("player")
        + "' == 'both'"
    )
    launch_right = IfCondition(
        "'"
        + LaunchConfiguration("player")
        + "' == 'right' or '"
        + LaunchConfiguration("player")
        + "' == 'both'"
    )

    # CAN interface setup
    set_can_down = ExecuteProcess(
        cmd=["sudo", "ip", "link", "set", "can0", "down"], output="screen"
    )

    set_can_bitrate = ExecuteProcess(
        cmd=[
            "sudo",
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

    # Left player ODrive nodes (axis 0 and 1)
    left_axis0_node = Node(
        package="odrive_can",
        executable="odrive_can_node",
        name="can_node_0",
        namespace="odrive_axis0",
        parameters=[{"node_id": 0, "interface": "can0", "axis_idle_on_shutdown": True}],
        output="screen",
        condition=launch_left,
    )

    left_axis1_node = Node(
        package="odrive_can",
        executable="odrive_can_node",
        name="can_node_1",
        namespace="odrive_axis1",
        parameters=[{"node_id": 1, "interface": "can0", "axis_idle_on_shutdown": True}],
        output="screen",
        condition=launch_left,
    )

    # Right player ODrive nodes (axis 2 and 3)
    right_axis2_node = Node(
        package="odrive_can",
        executable="odrive_can_node",
        name="can_node_2",
        namespace="odrive_axis2",
        parameters=[{"node_id": 2, "interface": "can0", "axis_idle_on_shutdown": True}],
        output="screen",
        condition=launch_right,
    )

    right_axis3_node = Node(
        package="odrive_can",
        executable="odrive_can_node",
        name="can_node_3",
        namespace="odrive_axis3",
        parameters=[{"node_id": 3, "interface": "can0", "axis_idle_on_shutdown": True}],
        output="screen",
        condition=launch_right,
    )

    # Motor commander node
    motor_commander_node = Node(
        package="motor_commander_pkg", executable="motor_commander", output="screen"
    )

    # Foxglove bridge
    foxglove_bridge_launch = ExecuteProcess(
        cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"],
        output="log",
    )

    return LaunchDescription(
        [
            player_arg,
            # set_can_down,
            # set_can_bitrate,
            left_axis0_node,
            left_axis1_node,
            right_axis2_node,
            right_axis3_node,
            motor_commander_node,
            # foxglove_bridge_launch
        ]
    )
