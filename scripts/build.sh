#!/bin/bash

colcon build --merge-install --install-base /opt/drs/install --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to drs_launch
