# Data Recording System

Forked from [edge-auto-jetson](https://github.com/tier4/edge-auto-jetson)

[Edge.Auto](https://sensor.tier4.jp/) sensor fusion system can be realized by using this with [edge-auto](https://github.com/tier4/edge-auto) repository.

<!-- ![object detection example](docs/sample.png "edge-auto-jetson object detection example") -->

# Prerequisites

### System Overview

This repository is based on a natively built ROS2 environment. The system overview is shown below.

<!-- ![system overview](docs/overview.drawio.svg "edge-auto-jetson overview") -->

### System Requirement

- Camera: v4l2 compatible cameras, including TIER IV Automotive HDR Camera C2
- ECU: Jetson AGX Orin based ECU, including [Anvil](https://connecttech.com/product/anvil-embedded-system-with-nvidia-jetson-agx-orin/) from ConnectTech Inc.
- NVIDIA L4T: R35.4.1 (including Ubuntu 20.04). BSP can be downloaded [here](https://connecttech.com/product/anvil-embedded-system-with-nvidia-jetson-agx-orin/)
- ROS: ROS2 Humble (native build)

## Getting Started

```shell
git clone git@github.com:tier4/data_recording_system.git
cd data_recording_system
./setup-dev-env.sh
vcs import src < autoware.repos
rosdep install -y -r --from-paths `colcon list --packages-up-to drs_launch -p` --ignore-src
./build.sh
./setup-drs.sh
```

## Related repositories

- [tier4/edge-auto](https://github.com/tier4/edge-auto)
  - Meta-repository containing `autoware.repos` to construct ROS-based workspace on x86-based ECU.
- [tier4/edge-auto-jetson](https://github.com/tier4/edge-auto-jetson)
  - Meta-repository containing `autoware.repos` file to construct ROS-based workspace on Jetson-based ECU.
- [tier4/edge_auto_launch](https://github.com/tier4/edge_auto_launch)
  - Launch configuration repository containing node configurations and their parameters for x86-based ECU.
- [tier4/edge_auto_jetson_launch](https://github.com/tier4/edge_auto_jetson_launch)
  - Launch configuration repository containing node configurations and their parameters for Jetson-based ECU.
- [tier4/edge_auto_individual_params](https://github.com/tier4/edge_auto_individual_params)
  - Repository for managing system parameters including camera parameters, driver parameters, etc.
- [tier4/nebula](https://github.com/tier4/nebula)
  - ROS2 package for unified ethernet-based LiDAR driver.
- [tier4/tier4_automotive_hdr_camera](https://github.com/tier4/tier4_automotive_hdr_camera)
  - Kernel driver for using TIER IV cameras with Video4Linux2 interface.
- [tier4/ros2_v4l2_camera](https://github.com/tier4/ros2_v4l2_camera)
  - ROS2 package for camera driver using Video4Linux2.
- [tier4/sensor_trigger](https://github.com/tier4/sensor_trigger)
  - ROS2 package for generating sensor trigger signals.
- [tier4/calibration_tools](https://github.com/tier4/CalibrationTools)
  - Repository for calibration tools to estimate parameters on autonomous driving systems.
- [autowarefoundation/autoware.universe](https://github.com/autowarefoundation/autoware.universe)
  - Repository for experimental, cutting-edge ROS packages for autonomous driving.
