FROM ubuntu:22.04

# Re-declare ARG after FROM to use in RUN commands
ARG ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

# Set timezone and locale
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV LANG=en_US.UTF-8

# Install runtime dependencies only
RUN apt update && \
    apt install -y --no-install-recommends \
        software-properties-common \
        tzdata \
        locales \
        curl \
        ca-certificates \
        python3 \
        python3-pip \
        libpython3.10 \
        gosu \
        build-essential \
        pkg-config \
        cmake && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install ROS2 repository and runtime packages
RUN curl -sSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -Po '"tag_name": "\K.*?(?=")' > /tmp/ros_apt_version && \
    curl -sSL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$(cat /tmp/ros_apt_version)/ros2-apt-source_$(cat /tmp/ros_apt_version).jammy_all.deb" -o /tmp/ros2-apt-source.deb && \
    apt update && \
    apt install -y /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb /tmp/ros_apt_version && \
    apt update && \
    apt install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-core \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rosbag2-storage-mcap \
        ros-${ROS_DISTRO}-rosbag2 \
        ros-dev-tools \
        libgrpc++-dev \
        libprotobuf-dev \
        protobuf-compiler \
        protobuf-compiler-grpc \
        python3-rosdep && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt/drs
COPY src/ /opt/drs/src/

# Initialize rosdep and install runtime dependencies for built packages
RUN rosdep init && \
    rosdep update && \
    apt update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon list --packages-up-to ros2_bridge | awk '{print $2}' | xargs rosdep install -iry --from-paths && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create install directory
RUN mkdir -p /opt/drs/install

# Build DRS packages
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --install-base /opt/drs/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to ros2_bridge

# Add cyclonedds.xml
COPY docker/cyclonedds.xml /opt/drs/config/
ENV CYCLONEDDS_URI=file:///opt/drs/config/cyclonedds.xml

# Add ROS environment variables
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=1
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Copy bashrc and entrypoint
COPY docker/bashrc /etc/bash.bashrc
COPY docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
