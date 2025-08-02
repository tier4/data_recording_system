#!/usr/bin/bash

set -e

docker run -it --rm \
    --net=host \
    --gpus all \
    -e LOCAL_UID=$(id -u) -e LOCAL_GID=$(id -g) -e LOCAL_USER=$(id -un) -e LOCAL_GROUP=$(id -gn) \
    -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -e XAUTHORITY=${XAUTHORITY} \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /etc/localtime:/etc/localtime:ro \
    tier4/drs-runtime:latest "$@"
