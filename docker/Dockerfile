ARG CUDA_VERSION=12.6.0
ARG ROS_DISTRO=humble

# Build stage
FROM nvidia/cuda:${CUDA_VERSION}-devel-ubuntu22.04 AS builder

# Re-declare ARG after FROM to use in RUN commands
ARG ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

# Set timezone and locale
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt update && apt install -y tzdata locales && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# Install basic tools and setup ROS2 repository
RUN apt update && \
    apt install -y software-properties-common curl git && \
    add-apt-repository -y universe && \
    curl -sSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -Po '"tag_name": "\K.*?(?=")' > /tmp/ros_apt_version && \
    curl -sSL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$(cat /tmp/ros_apt_version)/ros2-apt-source_$(cat /tmp/ros_apt_version).jammy_all.deb" -o /tmp/ros2-apt-source.deb && \
    apt install -y /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb /tmp/ros_apt_version && \
    apt update && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble and required packages
RUN apt update && \
    apt install -y ros-${ROS_DISTRO}-ros-core ros-dev-tools ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-rosbag2-storage-mcap ros-${ROS_DISTRO}-rosbag2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt/drs

# Copy source code (assumes vcs import has been done in src directory)
COPY src/ /opt/drs/src/

# Initialize and update rosdep
RUN rosdep init && \
    rosdep update

# Install ROS dependencies
RUN apt update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install -y -r --from-paths src --ignore-src && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create install directory
RUN mkdir -p /opt/drs/install

# Build DRS packages
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --merge-install --install-base /opt/drs/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to drs_launch

# Runtime stage
FROM nvidia/cuda:${CUDA_VERSION}-runtime-ubuntu22.04

# Re-declare ARG after FROM
ARG ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

# Set timezone and locale
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV LANG=en_US.UTF-8

# Install runtime dependencies only
RUN apt update && \
    apt install -y --no-install-recommends \
        tzdata \
        locales \
        curl \
        ca-certificates \
        python3 \
        python3-pip \
        libpython3.10 \
        gosu && \
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
        python3-rosdep && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Copy source temporarily for rosdep
COPY --from=builder /opt/drs/src /opt/drs/src

# Initialize rosdep and install runtime dependencies for built packages
RUN rosdep init && \
    rosdep update && \
    apt update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install -y -r --from-paths /opt/drs/src --ignore-src || true && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /opt/drs/src

# Copy only built packages from builder stage
COPY --from=builder /opt/drs/install /opt/drs/install

# Add cyclonedds.xml
ADD docker/cyclonedds.xml /opt/drs/config/
ENV CYCLONEDDS_URI=file:///opt/drs/config/cyclonedds.xml

# Add ROS environment variables
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=1
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Copy bashrc and entrypoint
ADD docker/bashrc /etc/bash.bashrc
ADD docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh

# entrypointとしてentrypoint.shを設定
ENTRYPOINT ["/entrypoint.sh"]