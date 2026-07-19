# Multi Transform Publisher

A ROS 2 node (`multi_transform_publisher`) that publishes multiple static transforms from a single YAML configuration file.

## Overview

This node publishes all static transforms to `/tf_static` from a single hierarchical YAML configuration file. It also automatically publishes camera optical link transforms.

## Parameters

- `config_file` (string, required): Path to YAML configuration file containing transform definitions
- `publish_camera_optical_link` (bool, default: true): Whether to automatically publish camera_link to camera_optical_link transforms
- `periodic_publish` (bool, default: true): Whether to republish the transforms every `publish_period` seconds instead of publishing them once at startup. Both modes publish to `/tf_static` via `StaticTransformBroadcaster`; this only controls repetition. Republishing is needed because recording splits bags periodically, and a one-time publication would appear only in the first split file.
- `publish_period` (double, default: 1.0): Period in seconds for periodic publishing (only used when `periodic_publish` is true)

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

### Standalone Node Launch

```xml
<include file="$(find-pkg-share drs_launch)/launch/component/multi_transform_publisher.launch.xml">
  <arg name="param_root_dir" value="$(find-pkg-share individual_params)/config/default"/>
  <arg name="publish_camera_optical_link" value="true"/>
</include>
```

The launch file passes `$(var param_root_dir)/multi_tf_static.yaml` to the node's `config_file` parameter.

### Command Line

```bash
# Run as standalone node
ros2 run multi_transform_publisher multi_transform_publisher --ros-args \
  -p config_file:=/path/to/transforms.yaml \
  -p publish_camera_optical_link:=true

# Run as component
ros2 component load /ComponentManager multi_transform_publisher MultiTfPublisher \
  -p config_file:=/path/to/transforms.yaml \
  -p publish_camera_optical_link:=true
```
