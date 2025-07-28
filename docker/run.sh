#!/usr/bin/bash

set -e

docker run -it --rm \
    --net=host \
    --gpus all \
    --sysctl net.core.rmem_max=2147483647 \
    --sysctl net.core.wmem_max=2147483647 \
    --sysctl net.ipv4.ipfrag_high_thresh=134217728 \
    --sysctl net.ipv4.ipfrag_time=1 \
    -e LOCAL_UID=$(id -u) -e LOCAL_GID=$(id -g) -e LOCAL_USER=$(id -un) -e LOCAL_GROUP=$(id -gn) \
    -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -e XAUTHORITY=${XAUTHORITY} \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /etc/localtime:/etc/localtime:ro \
    tier4/drs:latest
