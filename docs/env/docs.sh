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
    local build_cmd="${1:-/usr/local/bin/build-all.sh}"
    local doc_type="${2:-all}"
    
    echo "Building $doc_type documentation..."
    
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
        "$IMAGE_NAME" \
        "$build_cmd"
    
    if [ "$doc_type" = "all" ]; then
        echo "Documentation built successfully at $REPO_ROOT/site"
    elif [ "$doc_type" = "doxygen" ]; then
        echo "Doxygen documentation built at $REPO_ROOT/docs/reference/api/doxygen"
    else
        echo "MkDocs documentation built at $REPO_ROOT/site"
    fi
}

function show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --build          Build all documentation (Doxygen + MkDocs)"
    echo "  --doxygen        Build only Doxygen API documentation"
    echo "  --mkdocs         Build only MkDocs documentation"
    echo "  --rebuild-image  Rebuild the Docker image"
    echo "  --help           Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 --build         # Build everything"
    echo "  $0 --doxygen       # Build only Doxygen API docs"
    echo "  $0 --mkdocs        # Build only MkDocs site"
}

# Parse command line arguments
if [ $# -eq 0 ]; then
    show_usage
    exit 1
fi

case "$1" in
    --build)
        build_docs "/usr/local/bin/build-all.sh" "all"
        ;;
    --doxygen)
        build_docs "/usr/local/bin/build-doxygen.sh" "doxygen"
        ;;
    --mkdocs)
        build_docs "/usr/local/bin/build-mkdocs.sh" "mkdocs"
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
