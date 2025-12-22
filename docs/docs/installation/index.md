---
sidebar_position: 1
---

# Installation

:::info
This page is under construction.
:::

This section covers the complete installation process for the Data Recording System, including hardware setup, software installation, and sensor configuration.

## Installation Overview

The DRS installation process consists of three main phases:

1. **Hardware Setup** - Physical installation and configuration of ECUs and controllers
2. **Software Setup** - Installation of ROS 2, Docker, and DRS components
3. **Sensor Configuration** - Configuration of cameras, LiDARs, and INS/GNSS sensors

## Installation Phases

### Hardware Setup

- **[Hardware Setup Overview](./hardware-setup/index.md)** - Complete hardware installation guide
- **[Anvil ECU Setup](./hardware-setup/anvil-ecu-setup.md)** - Anvil ECU installation and configuration
- **[Raspberry Pi Controller Setup](./hardware-setup/raspi-controller-setup.md)** - RasPi controller setup
- **[Network Configuration](./hardware-setup/network-configuration.md)** - Network setup and configuration
- **[NAS Configuration](./hardware-setup/nas-configuration.md)** - Network Attached Storage setup

### Software Setup

- **[Software Setup Overview](./software-setup/index.md)** - Software installation guide
- **[ROS 2 Installation](./software-setup/ros2-installation.md)** - ROS 2 Humble installation
- **[Docker Setup](./software-setup/docker-setup.md)** - Docker environment configuration
- **[DRS Build](./software-setup/drs-build.md)** - Building DRS from source
- **[Ansible Deployment](./software-setup/ansible-deployment.md)** - Automated deployment with Ansible

### Sensor Configuration

- **[Sensor Configuration Overview](./sensor-configuration/index.md)** - Sensor setup guide
- **[Camera Driver Setup](./sensor-configuration/camera-driver-setup.md)** - Camera driver installation
- **[LiDAR Setup](./sensor-configuration/lidar-setup.md)** - LiDAR configuration
- **[INS Setup](./sensor-configuration/ins-setup.md)** - INS/GNSS configuration
- **[Sensor Trigger Setup](./sensor-configuration/sensor-trigger-setup.md)** - Sensor trigger configuration

## Installation Order

It is recommended to follow this installation order:

1. Hardware Setup
2. Software Setup
3. Sensor Configuration
