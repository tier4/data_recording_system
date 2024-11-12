# PointCloudOffsetConcatenationComponent

## Overview

`PointCloudOffsetConcatenationComponent` is a ROS 2 node designed to concatenate multiple point cloud streams with optional offsets in their timestamps. This component subscribes to multiple input topics, applies the specified offsets, transforms the point clouds to a common frame, and publishes the concatenated result.

## Features

- Subscribes to multiple point cloud topics.
- Applies timestamp offsets to synchronize point clouds.
- Transforms point clouds to a specified output frame.
- Concatenates point clouds and publishes the combined result.

## Parameters

### Required Parameters

- `output_frame` (string): The frame in which the concatenated point clouds will be published.
- `input_topics` (array of strings): List of input topics to subscribe to. At least two topics are required.

### Optional Parameters

- `input_offset` (array of doubles): List of timestamp offsets corresponding to the input topics. If provided, the length must match the number of input topics.

## Subscribed Topics

- Topics specified in the `input_topics` parameter. Each topic should publish messages of type `sensor_msgs/msg/PointCloud2`.

## Published Topics

- `output` (sensor_msgs/msg/PointCloud2): The concatenated point cloud in the specified `output_frame`.

## Usage

### Building the Package

1. Clone the repository into your autoware.universe workspace:
2. Build the package:

   ```bash
   colcon build --symlink-install --cmake-args -GNinja --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --symlink-install --packages-up-to pointcloud_concatenate
   ```

3. Source the setup file:

   ```bash
   source install/setup.bash
   ```

### Running the Node

To run the `PointCloudOffsetConcatenationComponent`, you can use a launch file or run it directly using the ROS 2 command line interface. Here is an example launch file:

```xml
<launch>
  <node pkg="pointcloud_concatenate" exec="pointcloud_concatenate_node" name="point_cloud_offset_concatenator" output="screen">
    <param name="output_frame" value="base_link"/>
    <param name="input_topics" value="['/sensing/lidar/front/pandar_points', '/sensing/lidar/left/pandar_points', '/sensing/lidar/right/pandar_points', '/sensing/lidar/top/pandar_points']"/>
    <param name="input_offset_msec" value="[0.0, 30.0, 30.0, 100.0]"/>
  </node>
</launch>
```
