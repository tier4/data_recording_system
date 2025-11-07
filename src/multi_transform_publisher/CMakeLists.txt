cmake_minimum_required(VERSION 3.5)
project(multi_transform_publisher)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# Add library for component
add_library(multi_transform_publisher_component SHARED
  src/multi_transform_publisher.cpp
)

# Include directories
target_include_directories(multi_transform_publisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIRS}
)

# Link libraries
ament_target_dependencies(multi_transform_publisher_component
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  geometry_msgs
)

target_link_libraries(multi_transform_publisher_component
  yaml-cpp
)

# Register component
rclcpp_components_register_node(multi_transform_publisher_component
  PLUGIN "MultiTfPublisher"
  EXECUTABLE multi_transform_publisher
)

# Install
install(TARGETS
  multi_transform_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  multi_transform_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
