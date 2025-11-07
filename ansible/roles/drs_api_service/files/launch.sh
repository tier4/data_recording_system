#!/bin/bash
set -e

# Log startup
logger -t drs-api "Starting DRS API Service"

# Source environment files
if [ -f /opt/drs/config/drs.env ]; then
    # shellcheck disable=SC1091
    source /opt/drs/config/drs.env
fi

# Launch ROS2 application
exec /usr/local/bin/module-manager -config=/opt/drs/config/module-manager.yaml
