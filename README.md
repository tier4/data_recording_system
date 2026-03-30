# Data Recording System

## Overview

The Data Recording System (DRS) is a high-performance sensor data recording system designed for autonomous vehicles. It runs on NVIDIA Jetson AGX Orin-based embedded ECUs and synchronously records data from multiple sensors including cameras, LiDARs, and IMUs.

### Key Features

- **Multi-Sensor Support**: Cameras (v4l2-compatible, TIER IV C1/C2/C3 cams), LiDARs/Radars (Nebula integration), GNSS/INS (OxTS)
- **Distributed Architecture**: Parallel processing across multiple ECUs (ecu0/ecu1)
- **High-Precision Time Synchronization**: Inter-sensor synchronization via PTP (Precision Time Protocol)
- **Hardware Trigger**: GPIO-based sensor synchronization trigger generation
- **ROS 2 Based**: Native build with ROS 2 Humble, optimized for embedded systems
- **API/Dashboard**: System management via gRPC API and Web dashboard

## Dependencies

- cuda >= 11.8
- gcc <= 11.x
- ros 2 distro == humble

For detailed dependency libraries, please refer to [`docker/runtime/Dockerfile`](docker/runtime/Dockerfile).

## Build

<!-- markdown-link-check-disable -->

This project depends on the private repository [tier4/c2_readout_delay_setter](https://github.com/tier4/c2_readout_delay_setter) (will be made public soon).

<!-- markdown-link-check-enable -->

`<GITHUB_TOKEN>` is a GitHub personal access token with permission to clone this repository. You can create one at <https://github.com/settings/tokens>.

```bash
git clone https://github.com/tier4/data_recording_system.git
cd data_recording_system
./scripts/setup_repos.sh --token <GITHUB_TOKEN>
rosdep install -y -r --from-paths `colcon list --packages-up-to drs_launch -p` --ignore-src
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to drs_launch
```
