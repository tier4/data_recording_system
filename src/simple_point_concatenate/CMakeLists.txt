
cmake_minimum_required(VERSION 3.14)
project(pointcloud_concatenate)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED)

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
)

# Targets
ament_auto_add_library(pointcloud_concatenate_node SHARED
src/pointcloud_concatenate/offset_concatenate_pointclouds.cpp
)

rclcpp_components_register_node(pointcloud_concatenate_node
  PLUGIN "pointcloud_concatenate::PointCloudOffsetConcatenationComponent"
  EXECUTABLE offset_concatenate_pointclouds_node)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
