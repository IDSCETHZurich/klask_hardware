#!/bin/bash

# Get the directory of this script
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

TAG="latest"

docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$SCRIPT_DIR/../src:/root/ros2_ws/src:rw" \
    --name="klask_ros_sdk_container_$TAG" \
    klask_ros_sdk:$TAG 