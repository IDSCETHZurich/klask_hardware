#!/bin/bash
set -e

# Default values
PLAYER="both"
START_VIEWER="true"

# Parse named arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --player)
            PLAYER="$2"
            shift 2
            ;;
        --viewer)
            START_VIEWER="true"
            shift
            ;;
        --no-viewer)
            START_VIEWER="false"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--player left|right|both] [--viewer|--no-viewer]"
            exit 1
            ;;
    esac
done

# Source ROS environment
source /ros_entrypoint.sh

# Launch with specified parameters
exec ros2 launch klask_motor_commander_pkg motors_launch.py \
    start_camera:=true \
    start_viewer:="$START_VIEWER" \
    player:="$PLAYER"
