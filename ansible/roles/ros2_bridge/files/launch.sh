#!/bin/bash
set -e

# Log startup
logger -t drs-launch "Starting DRS ros2-bridge Service"

# Source environment files
if [ -f /opt/drs/config/drs.env ]; then
    source /opt/drs/config/drs.env
fi

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f /opt/drs/install/setup.bash ]; then
    source /opt/drs/install/setup.bash
fi

# Launch ROS2 application
exec ros2 run ros2_bridge ros2_bridge_node