#!/usr/bin/bash

set -e

COLCON_PARALLEL_WORKERS=2

while [[ $# -gt 0 ]]; do
    case "$1" in
    --colcon-parallel-workers)
        COLCON_PARALLEL_WORKERS="$2"
        shift 2
        ;;
    *)
        echo "Error: unknown option '$1'" >&2
        echo "Usage: $0 [--colcon-parallel-workers <N>]" >&2
        exit 1
        ;;
    esac
done

if ! [[ $COLCON_PARALLEL_WORKERS =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: parallel workers must be a positive integer, got '$COLCON_PARALLEL_WORKERS'" >&2
    exit 1
fi

# Change to repository root directory
cd "$(dirname "$0")/../.."

# Build the Docker image
echo "Building DRS Docker image (parallel workers: ${COLCON_PARALLEL_WORKERS})..."

# Build the image from repository root
docker build \
    --build-arg COLCON_PARALLEL_WORKERS="${COLCON_PARALLEL_WORKERS}" \
    -t tier4/pkg-drs-runtime:latest \
    -f docker/runtime/Dockerfile .

echo "Docker image built successfully: tier4/pkg-drs-runtime:latest"
