// Copyright 2026 TIER IV, Inc.

#include "vehicle_can_decoder/vehicle_can_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<vehicle_can_decoder::VehicleCanNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
