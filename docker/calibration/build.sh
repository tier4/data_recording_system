#!/usr/bin/bash

set -e

# Change to parent directory
cd ../..

# Build the Docker image
echo "Building DRS Docker image..."

# Build the image from parent directory
docker build -t tier4/drs-calibration:latest -f docker/calibration/Dockerfile .

echo "Docker image built successfully: tier4/drs-calibration:latest"
