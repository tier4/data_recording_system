#!/bin/bash

colcon build --merge-install --install-base /opt/drs/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to drs_launch
