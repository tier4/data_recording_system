#!/usr/bin/bash

set -e

# Change to repository root directory
cd "$(dirname "$0")/../.."

# Build the Docker image
echo "Building DRS Docker image..."

# Build the image from repository root
docker build -t tier4/pkg-drs-calibration:latest -f docker/calibration/Dockerfile .

echo "Docker image built successfully: tier4/pkg-drs-calibration:latest"
