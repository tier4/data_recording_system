# Multi TF Publisher

A ROS2 node that publishes multiple static transforms from a single YAML configuration file.

## Overview

This node replaces multiple `periodic_transform_publisher` instances with a single node that publishes all static transforms to `/tf_static`. It supports hierarchical YAML configuration and automatically publishes camera optical link transforms.

## Parameters

- `config_file` (string, required): Path to YAML configuration file containing transform definitions
- `publish_camera_optical_link` (bool, default: true): Whether to automatically publish camera_link to camera_optical_link transforms

## YAML Configuration Format

The YAML file uses a hierarchical structure where parent frames contain child frames with transform data:

```yaml
parent_frame:
  child_frame:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```

### Example Configuration

```yaml
base_link:
  drs_base_link:
    x: 0.6895
    y: 0.0
    z: 1.971
    roll: 0.000
    pitch: -0.020
    yaw: 0.000

drs_base_link:
  lidar_front:
    x: 0.4734
    y: 0.05
    z: -0.0702
    roll: 0.005
    pitch: 0.000
    yaw: -1.5708

lidar_front:
  camera0/camera_link:
    x: -0.017018
    y: 0.167448
    z: -0.111674
    roll: -0.003068
    pitch: -0.000116
    yaw: 1.573144
```

## Camera Optical Link Transform

When `publish_camera_optical_link` is true, the node automatically creates transforms from `camera_link` to `camera_optical_link` frames with a fixed rotation that aligns the optical frame with ROS conventions:
- No translation
- Rotation: quaternion (0.5, -0.5, 0.5, -0.5)

## Usage

### Launch File Example

```xml
<include file="$(find-pkg-share drs_launch)/launch/component/multi_tf_publisher.launch.py">
  <arg name="config_file" value="$(find-pkg-share individual_params)/config/default/multi_tf_static.yaml"/>
  <arg name="publish_camera_optical_link" value="true"/>
</include>
```

### Command Line

```bash
ros2 run multi_tf_publisher multi_tf_publisher --ros-args \
  -p config_file:=/path/to/transforms.yaml \
  -p publish_camera_optical_link:=true
```