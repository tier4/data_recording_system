---
sidebar_position: 1
---

# Calibration

This section covers the calibration process for cameras and LiDARs in the Data Recording System.

## Calibration Overview

Calibration is essential for accurate sensor data recording. DRS requires calibration for:

- **Camera Intrinsic Parameters** - Internal camera parameters (focal length, distortion, etc.)
- **Camera-LiDAR Extrinsic Parameters** - Relative pose between cameras and LiDARs
- **LiDAR-LiDAR Parameters** - Relative pose between multiple LiDARs

## Calibration Process

The calibration workflow consists of:

1. **Setup** - Prepare calibration environment and tools
2. **Sensor Operation Check** - Verify all sensors are working correctly
3. **Camera Calibration** - Intrinsic and extrinsic camera calibration
4. **LiDAR Calibration** - LiDAR-LiDAR calibration
5. **Result Integration** - Combine all calibration results

## Calibration Guides

### Setup

- **[Calibration Setup Overview](./setup/index.md)** - Calibration environment setup
- **[Docker Setup](./setup/docker-setup.md)** - Docker-based calibration setup
- **[Source Build](./setup/source-build.md)** - Building calibration tools from source
- **[Prerequisites](./setup/prerequisites.md)** - Calibration prerequisites

### Sensor Operation Check

Before starting calibration, verify that all sensors are working correctly:

- **[Camera Verification](../testing/sensor-verification/camera-verification.md)** - Camera operation check
- **[LiDAR Verification](../testing/sensor-verification/lidar-verification.md)** - LiDAR operation check

### Camera Calibration

- **[Camera Calibration Overview](./camera-calibration/index.md)** - Camera calibration guide
- **[Intrinsic Calibration](./camera-calibration/intrinsic-calibration.md)** - Camera intrinsic calibration
- **[Extrinsic Calibration](./camera-calibration/extrinsic-calibration.md)** - Camera-LiDAR extrinsic calibration
- **[Result Verification](./camera-calibration/result-verification.md)** - Verify calibration results

### LiDAR Calibration

- **[LiDAR Calibration Overview](./lidar-calibration/index.md)** - LiDAR calibration guide
- **[LiDAR-LiDAR Calibration](./lidar-calibration/lidar-lidar-calibration.md)** - LiDAR-LiDAR calibration
- **[Result Verification](./lidar-calibration/result-verification.md)** - Verify LiDAR calibration

### Result Integration

- **[Result Integration Overview](./result-integration/index.md)** - Integration guide
- **[Multi-TF Generation](./result-integration/multi-tf-generation.md)** - Generate multi_tf_static.yaml
- **[Applying Results](./result-integration/applying-results.md)** - Apply calibration to ECUs
