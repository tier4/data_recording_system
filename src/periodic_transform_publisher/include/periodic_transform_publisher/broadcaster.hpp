#ifndef PERIODIC_TRANSFORM_PUBLISHER__BROADCASTER_HPP_
#define PERIODIC_TRANSFORM_PUBLISHER__BROADCASTER_HPP_
/*
 * ref:
 * https://github.com/ros2/geometry2/blob/humble/tf2_ros/src/static_transform_broadcaster_node.cpp
 */

#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

std::string get_unique_node_name()
{
  static const std::string chars = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
  static std::random_device rd;
  static std::minstd_rand g{rd()};

  // The uniform_int_distribution takes a closed interval of [a, b]; since that
  // would include the \0, we remove one from our string length.
  static std::uniform_int_distribution<std::string::size_type> pick(0, chars.length() - 1);

  std::string s{"periodic_transform_publisher_"};

  size_t orig_length = s.length();
  s.resize(orig_length + 16);

  for (size_t i = orig_length; i < s.length(); ++i) {
    s[i] = chars[pick(g)];
  }

  return s;
}

class Broadcaster : public rclcpp::Node
{
 public:
  explicit Broadcaster(const rclcpp::NodeOptions & options)
      : rclcpp::Node(get_unique_node_name(), options) {

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;

    tf_msg_.transform.translation.x = this->declare_parameter("translation.x", 0.0, descriptor);
    tf_msg_.transform.translation.y = this->declare_parameter("translation.y", 0.0, descriptor);
    tf_msg_.transform.translation.z = this->declare_parameter("translation.z", 0.0, descriptor);
    tf_msg_.transform.rotation.x = this->declare_parameter("rotation.x", 0.0, descriptor);
    tf_msg_.transform.rotation.y = this->declare_parameter("rotation.y", 0.0, descriptor);
    tf_msg_.transform.rotation.z = this->declare_parameter("rotation.z", 0.0, descriptor);
    tf_msg_.transform.rotation.w = this->declare_parameter("rotation.w", 1.0, descriptor);
    tf_msg_.header.frame_id = this->declare_parameter("frame_id", std::string("/frame"), descriptor);
    tf_msg_.child_frame_id =
        this->declare_parameter("child_frame_id", std::string("/child"), descriptor);

    double interval_sec = this->declare_parameter("interval_sec", 1.0, descriptor);

    // check frame_id != child_frame_id
    if (tf_msg_.header.frame_id == tf_msg_.child_frame_id) {
      RCLCPP_ERROR(
          this->get_logger(),
          "cannot publish transform from %s to %s, exiting",
          tf_msg_.header.frame_id.c_str(), tf_msg_.child_frame_id.c_str()
      );
      throw std::runtime_error("child_frame_id cannot equal frame_id");
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(interval_sec),
                                  std::bind(&Broadcaster::callback, this));
  }

  void callback() {
    tf_msg_.header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(tf_msg_);
  }

 protected:
  geometry_msgs::msg::TransformStamped tf_msg_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

};

#endif // PERIODIC_TRANSFORM_PUBLISHER__BROADCASTER_HPP_
