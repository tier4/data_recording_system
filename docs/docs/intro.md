---
sidebar_position: 1
slug: /
---

# Welcome to Data Recording System

The **Data Recording System (DRS)** is a high-performance sensor data recording system designed for Co-MLOps platform. It runs on NVIDIA Jetson AGX Orin-based embedded ECUs and synchronously records data from multiple sensors including cameras, LiDARs, and IMUs.

## Key Features

- **Multi-Sensor Support**: Cameras (v4l2-compatible, TIER IV C1/C2/C3 cams), LiDARs (Nebula integration), GNSS/INS (OxTS)
- **Distributed Architecture**: Parallel processing across multiple ECUs (ecu0/ecu1)
- **High-Precision Time Synchronization**: Inter-sensor synchronization via PTP (Precision Time Protocol)
- **Hardware Trigger**: GPIO-based sensor synchronization trigger generation
- **ROS 2 Based**: Native build with ROS 2 Humble, optimized for embedded systems
- **API/Dashboard**: System management via gRPC API and Web dashboard

## Documentation Structure

This documentation is organized to guide you through the complete lifecycle of setting up and operating a DRS:

- **Getting Started** - Introduction and system overview
- **Installation** - Hardware, software, and sensor setup
- **Calibration** - Camera and LiDAR calibration
- **Operation** - Data recording, system management, and data handling
- **Testing** - Verification and testing procedures
- **Troubleshooting** - Common issues and solutions
- **Reference** - Technical reference documentation

## System Requirements

- **Hardware**: NVIDIA Jetson AGX Orin (Anvil ECUs)
- **OS**: Ubuntu 22.04
- **Software**: ROS 2 Humble, Docker, CUDA >= 11.8

For detailed requirements, see [Prerequisites](./getting-started/prerequisites.md).
