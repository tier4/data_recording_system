#!/bin/bash
colcon build \
    --symlink-install --continue-on-error --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to drs_launch

