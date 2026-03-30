#!/bin/bash
set -euo pipefail

if ! command -v colcon &>/dev/null; then
    echo "Error: colcon is not installed." >&2
    echo "" >&2
    echo "Install it with:" >&2
    echo "  sudo apt install python3-colcon-common-extensions" >&2
    echo "  # or: pip install colcon-common-extensions" >&2
    exit 1
fi

colcon build --merge-install --install-base /opt/drs/install --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to drs_launch
