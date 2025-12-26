#!/usr/bin/bash

set -e

# Help message
show_help() {
    cat <<EOF
Usage: $(basename "$0") [--local] [--option DOCKER_OPTIONS] [--] COMMAND [ARGS...]

Launch a Docker container with GPU and GUI support

Options:
    --local      Use local Docker image (tier4/pkg-drs-calibration:latest)
    --option     Treat following arguments as additional Docker options
    --           Treat following arguments as commands to run inside the container
    --help, -h   Show this help message

Examples:
    # Basic usage
    $(basename "$0") python test.py

    # Use local image
    $(basename "$0") --local python test.py

    # Add volume mount
    $(basename "$0") --option -v /home/user/data:/data -- python test.py

    # Add multiple options
    $(basename "$0") --option -v /data:/data -e MY_VAR=value --privileged -- bash

EOF
}

# Check for help option
if [[ $1 == "--help" ]] || [[ $1 == "-h" ]]; then
    show_help
    exit 0
fi

# Base Docker options
BASE_DOCKER_OPTS=(
    -it --rm
    --net=host
    --gpus all
    -e "LOCAL_UID=$(id -u)"
    -e "LOCAL_GID=$(id -g)"
    -e "LOCAL_USER=$(id -un)"
    -e "LOCAL_GROUP=$(id -gn)"
    -e "DISPLAY=$DISPLAY"
    -v "/tmp/.X11-unix/:/tmp/.X11-unix"
    --device=/dev/dri
    -e "XAUTHORITY=${XAUTHORITY}"
    -e "XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"
    -e "NVIDIA_DRIVER_CAPABILITIES=all"
    -v "/etc/localtime:/etc/localtime:ro"
)

# Mount XAUTHORITY if it is set
if [[ -n ${XAUTHORITY} ]]; then
    BASE_DOCKER_OPTS+=(-v "${XAUTHORITY}:${XAUTHORITY}")
fi

# Additional Docker options and command arguments
USE_LOCAL_IMAGE=false
EXTRA_DOCKER_OPTS=()
COMMAND_ARGS=()
PARSING_MODE="command"

while [[ $# -gt 0 ]]; do
    case $1 in
    --local)
        USE_LOCAL_IMAGE=true
        shift
        ;;
    --option)
        PARSING_MODE="docker"
        shift
        ;;
    --)
        shift
        COMMAND_ARGS=("$@")
        break
        ;;
    *)
        if [[ $PARSING_MODE == "docker" ]]; then
            if [[ $1 == -* ]]; then
                EXTRA_DOCKER_OPTS+=("$1")
                shift
                if [[ $# -gt 0 && $1 != -* && $1 != "--" && $1 != "--option" && $1 != "--local" ]]; then
                    EXTRA_DOCKER_OPTS+=("$1")
                    shift
                fi
            else
                EXTRA_DOCKER_OPTS+=("$1")
                shift
            fi
        else
            COMMAND_ARGS+=("$1")
            shift
        fi
        ;;
    esac
done

# Debug output (comment out if not needed)
# echo "Extra Docker opts: ${EXTRA_DOCKER_OPTS[@]}"
# echo "Command args: ${COMMAND_ARGS[@]}"

# Select Docker image based on --local flag
if [[ $USE_LOCAL_IMAGE == true ]]; then
    DOCKER_IMAGE="tier4/pkg-drs-calibration:latest"
else
    DOCKER_IMAGE="ghcr.io/tier4/pkg-drs-calibration:latest"
fi

# Execute Docker command
docker run "${BASE_DOCKER_OPTS[@]}" "${EXTRA_DOCKER_OPTS[@]}" "$DOCKER_IMAGE" "${COMMAND_ARGS[@]}"
