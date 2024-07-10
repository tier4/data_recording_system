#!/bin/bash
colcon build \
    --symlink-install --continue-on-error --cmake-args -DCMAKE_BUILD_TYPE=Release
