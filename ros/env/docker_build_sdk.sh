#!/bin/bash

# Get the directory of this script
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Build the SDK Docker image
docker build -t klask_ros_sdk:latest -f $SCRIPT_DIR/SDK.Dockerfile .