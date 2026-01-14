#!/bin/bash
# Script to check clang-format compliance

set -e

WORKSPACE_ROOT="$1"
shift
FILES="$@"

cd "$WORKSPACE_ROOT"

echo "Checking clang-format compliance for ${#FILES[@]} files..."

# Run clang-format in check mode
if clang-format --style=file --dry-run --Werror $FILES; then
    echo "All files are properly formatted according to .clang-format"
    exit 0
else
    echo "✗ Some files are not formatted according to .clang-format"
    echo "Run 'clang-format -i <file>' to fix formatting"
    exit 1
fi
