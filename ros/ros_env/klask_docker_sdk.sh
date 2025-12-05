#!/bin/bash

# Unified SDK Docker management script
# Usage: ./klask_docker_sdk.sh [build|run|connect|stop|status]

# To enable tab completion, add this to your ~/.bashrc 
# or execute it in your shell for to have it for the current session:
#     source /path/to/klask_docker_sdk.sh --completion

# Bash completion support
if [[ "$1" == "--completion" ]]; then
    _klask_sdk_completions() {
        local commands="build run connect stop status logs"
        COMPREPLY=($(compgen -W "$commands" -- "${COMP_WORDS[1]}"))
    }
    # Register for various ways the script might be called
    complete -F _klask_sdk_completions klask_docker_sdk.sh
    complete -F _klask_sdk_completions ./klask_docker_sdk.sh
    complete -F _klask_sdk_completions ./ros/env/klask_docker_sdk.sh
    complete -F _klask_sdk_completions ros/env/klask_docker_sdk.sh
    return 0 2>/dev/null || exit 0
fi

set -e

# Get the directory of this script
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Configuration
IMAGE_NAME="klask_ros_sdk"
TAG="latest"
CONTAINER_NAME="${IMAGE_NAME}_container_${TAG}"
VIDEO_DEVICE="/dev/video0"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_usage() {
    echo "Usage: $0 <command>"
    echo ""
    echo "Commands:"
    echo "  build    Build the SDK Docker image"
    echo "  run      Run the SDK container (detached)"
    echo "  connect  Connect to the running SDK container"
    echo "  stop     Stop the SDK container"
    echo "  status   Show container status"
    echo ""
}

cmd_build() {
    echo -e "${GREEN}Building SDK Docker image...${NC}"

    docker build -t "${IMAGE_NAME}:${TAG}" -f "$SCRIPT_DIR/SDK.Dockerfile" "$SCRIPT_DIR"

    echo -e "${GREEN}Build complete!${NC}"
}

cmd_run() {
    # Check if container is already running
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo -e "${YELLOW}Container '${CONTAINER_NAME}' is already running.${NC}"
        echo "Use '$0 connect' to attach to it, or '$0 stop' to stop it first."
        return 1
    fi

    echo -e "${GREEN}Starting SDK container...${NC}"

    docker run -it -d --rm \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$SCRIPT_DIR/../src:/root/ros2_ws/src:rw" \
        --volume="$SCRIPT_DIR/../.vscode:/root/ros2_ws/.vscode:rw" \
        --volume="${CONTAINER_NAME}_build:/root/ros2_ws/build:rw" \
        --volume="${CONTAINER_NAME}_install:/root/ros2_ws/install:rw" \
        --volume="${CONTAINER_NAME}_log:/root/ros2_ws/log:rw" \
        --device="$VIDEO_DEVICE:/dev/video0" \
        --network=host \
        --cap-add=NET_ADMIN \
        --cap-add=NET_RAW \
        --name="${CONTAINER_NAME}" \
        "${IMAGE_NAME}:${TAG}"

    echo -e "${GREEN}Container started! Use '$0 connect' to attach.${NC}"
}

cmd_connect() {
    # Check if container is running
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo -e "${RED}Container '${CONTAINER_NAME}' is not running.${NC}"
        echo "Use '$0 run' to start it first."
        return 1
    fi

    echo -e "${GREEN}Connecting to SDK container...${NC}"
    docker exec -it -w /root/ros2_ws "${CONTAINER_NAME}" bash
}

cmd_stop() {
    # Check if container is running
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo -e "${YELLOW}Container '${CONTAINER_NAME}' is not running.${NC}"
        return 0
    fi

    echo -e "${GREEN}Stopping SDK container...${NC}"

    docker stop "${CONTAINER_NAME}"
    
    echo -e "${GREEN}Container stopped!${NC}"
}

cmd_status() {
    echo -e "${GREEN}SDK Container Status:${NC}"
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo -e "  Status: ${GREEN}Running${NC}"
        docker ps --filter "name=${CONTAINER_NAME}" --format "  ID: {{.ID}}\n  Created: {{.RunningFor}}\n  Ports: {{.Ports}}"
    else
        echo -e "  Status: ${RED}Not running${NC}"
    fi
    
    echo ""
    echo -e "${GREEN}SDK Image:${NC}"
    if docker images "${IMAGE_NAME}:${TAG}" --format "{{.Repository}}" | grep -q "${IMAGE_NAME}"; then
        docker images "${IMAGE_NAME}:${TAG}" --format "  Repository: {{.Repository}}:{{.Tag}}\n  ID: {{.ID}}\n  Size: {{.Size}}\n  Created: {{.CreatedSince}}"
    else
        echo -e "  ${RED}Image not found. Run '$0 build' first.${NC}"
    fi
}

# Main
if [ $# -eq 0 ]; then
    print_usage
    exit 1
fi

case "$1" in
    build)
        cmd_build
        ;;
    run)
        cmd_run
        ;;
    connect)
        cmd_connect
        ;;
    stop)
        cmd_stop
        ;;
    status)
        cmd_status
        ;;
    -h|--help|help)
        print_usage
        ;;
    *)
        echo -e "${RED}Unknown command: $1${NC}"
        echo ""
        print_usage
        exit 1
        ;;
esac
