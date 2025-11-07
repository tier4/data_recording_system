FROM ubuntu:22.04

# Re-declare ARG after FROM to use in RUN commands
ARG ROS_DISTRO=humble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Set timezone and locale
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && apt-get install -y tzdata locales gosu && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# Install basic tools and setup ROS2 repository
RUN apt-get update && \
    apt-get install -y software-properties-common curl git python3 python3-pip && \
    add-apt-repository -y universe && \
    curl -sSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -Po '"tag_name": "\K.*?(?=")' > /tmp/ros_apt_version && \
    curl -sSL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$(cat /tmp/ros_apt_version)/ros2-apt-source_$(cat /tmp/ros_apt_version).jammy_all.deb" -o /tmp/ros2-apt-source.deb && \
    apt-get install -y /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb /tmp/ros_apt_version && \
    apt-get update && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble and required packages
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-desktop ros-dev-tools ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-rosbag2-storage-mcap ros-${ROS_DISTRO}-rosbag2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt/drs
# COPY src/ /opt/drs/src/

RUN mkdir -p /opt/drs/src && \
    git clone https://github.com/tier4/CalibrationTools.git -b feat/drs_202505 /opt/drs/src/calibration_tools && \
    vcs import src < /opt/drs/src/calibration_tools/calibration_tools_standalone.repos

# Initialize and update rosdep
RUN rosdep init && \
    rosdep update

# Install ROS dependencies
RUN apt-get update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install -y -r --from-paths `colcon list --packages-up-to sensor_calibration_tools -p` --ignore-src --rosdistro ${ROS_DISTRO} && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create install directory
RUN mkdir -p /opt/drs/install

# Build DRS packages
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --install-base /opt/drs/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to sensor_calibration_tools

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

# entrypointとしてentrypoint.shを設定
ENTRYPOINT ["/entrypoint.sh"]
