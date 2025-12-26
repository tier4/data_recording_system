#!/usr/bin/bash

# Get the user ID and group ID of the local user
USER_ID=${LOCAL_UID}
USER_NAME=${LOCAL_USER}
GROUP_ID=${LOCAL_GID}
GROUP_NAME=${LOCAL_GROUP}

# Check if any of the variables are empty
if [[ -z $USER_ID || -z $USER_NAME || -z $GROUP_ID || -z $GROUP_NAME ]]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    source /opt/drs/install/setup.bash
    exec "$@"
else
    echo "Starting with user: $USER_NAME >> UID $USER_ID, GID: $GROUP_ID"

    # Create group and user with GID/UID
    groupadd -g "$GROUP_ID" "$GROUP_NAME"
    useradd -u "$USER_ID" -g "$GROUP_ID" -s /bin/bash -m -d /home/"$USER_NAME" "$USER_NAME"

    # Add sudo privileges to the user
    echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >>/etc/sudoers

    # Set HOME environment variable
    export HOME=/home/"$USER_NAME"

    # Copy bashrc to user's home
    cp /etc/bash.bashrc "$HOME/.bashrc"
    chown "$USER_NAME:$GROUP_NAME" "$HOME/.bashrc"

    # Source ROS 2
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    source /opt/drs/install/setup.bash

    # Set working directory
    cd "$HOME"

    # Prepare runtime directory
    mkdir -p /run/user/$USER_ID
    chmod 700 /run/user/$USER_ID
    chown "$USER_NAME":"$GROUP_NAME" /run/user/$USER_ID

    # Execute the command as the user
    exec gosu "$USER_NAME" "$@"
fi
