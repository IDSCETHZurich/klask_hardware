from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.actions import ExecuteProcess

def generate_launch_description():
    foxglove_bridge_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'foxglove_bridge', 'foxglove_bridge_launch.xml'],
        output='log'
    )
    set_can_down = ExecuteProcess(
        cmd=['sudo', 'ip', 'link', 'set', 'can0', 'down'],
        output='screen'
    )

    # Set CAN bitrate
    set_can_bitrate = ExecuteProcess(
        cmd=['sudo', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate', '250000'],
        output='screen'
    )

    odrive_can_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'motor_commander_pkg', 'left_player.yaml'],
        output='screen'
    )


    return LaunchDescription([
        #set_can_down,
        #set_can_bitrate,
        odrive_can_launch,

        # Node(
        #     package='klask_state_estimation',
        #     executable='ball_peg_detection',
        #     name='ball_peg_detection',
        #     output='screen'
        # ),
        
        Node(
            package='motor_commander_pkg',
            executable='motor_commander',
            output='screen'
        ),

        # foxglove_bridge_launch
    ])