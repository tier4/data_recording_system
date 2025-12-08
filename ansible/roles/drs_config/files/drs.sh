#!/bin/bash
# Load DRS configuration if available
if [ -f /opt/drs/config/drs.env ]; then
    source /opt/drs/config/drs.env
fi

# Load DRS ROS 2 environment if available
if [ -f /opt/drs/install/setup.bash ]; then
    source /opt/drs/install/setup.bash
fi
