#!/bin/bash

# Documentation builder script for KLASK Hardware
# Usage: ./docs.sh --build

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Get the repository root (two levels up from docs/env)
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

IMAGE_NAME="klask-docs-builder"
CONTAINER_NAME="klask-docs-build"

function build_image() {
    echo "Building documentation container image..."
    docker build -t "$IMAGE_NAME" "$SCRIPT_DIR"
    echo "Image built successfully"
}

function build_docs() {
    echo "Building documentation..."
    
    # Check if image exists, if not build it
    if ! docker image inspect "$IMAGE_NAME" &> /dev/null; then
        echo "Image not found. Building first..."
        build_image
    fi
    
    # Run the container to build docs
    docker run --rm \
        --name "$CONTAINER_NAME" \
        -v "$REPO_ROOT:/workspace" \
        -u "$(id -u):$(id -g)" \
        "$IMAGE_NAME"
    
    echo "Documentation built successfully at $REPO_ROOT/site"
}

function show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --build          Build the documentation"
    echo "  --rebuild-image  Rebuild the Docker image"
    echo "  --help           Show this help message"
    echo ""
    echo "Example:"
    echo "  $0 --build"
}

# Parse command line arguments
if [ $# -eq 0 ]; then
    show_usage
    exit 1
fi

case "$1" in
    --build)
        build_docs
        ;;
    --rebuild-image)
        build_image
        ;;
    --help)
        show_usage
        ;;
    *)
        echo "Error: Unknown option '$1'"
        show_usage
        exit 1
        ;;
esac
