// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pointcloud_concatenate/offset_concatenate_pointclouds.hpp"

#include <pcl_ros/transforms.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace pointcloud_concatenate
{
PointCloudOffsetConcatenationComponent::PointCloudOffsetConcatenationComponent(const rclcpp::NodeOptions & node_options)
: Node("point_cloud_offset_concatenator_component", node_options)
{
  // Set parameters
  {
    output_frame_ = static_cast<std::string>(declare_parameter("output_frame", ""));
    if (output_frame_.empty()) {
      RCLCPP_ERROR(get_logger(), "Need an 'output_frame' parameter to be set before continuing!");
      return;
    }
    declare_parameter("input_topics", std::vector<std::string>());
    input_topics_ = get_parameter("input_topics").as_string_array();
    if (input_topics_.empty()) {
      RCLCPP_ERROR(get_logger(), "Need a 'input_topics' parameter to be set before continuing!");
      return;
    }
    if (input_topics_.size() == 1) {
      RCLCPP_ERROR(get_logger(), "Only one topic given. Need at least two topics to continue.");
      return;
    }

    input_offset_msec_ = declare_parameter("input_offset_msec", std::vector<double>{});
    if (!input_offset_msec_.empty() && input_topics_.size() != input_offset_msec_.size()) {
      RCLCPP_ERROR(get_logger(), "The number of topics does not match the number of offsets.");
      return;
    }
    for (const auto & offset : input_offset_msec_) {
      RCLCPP_INFO(get_logger(), "Offset: %f", offset);
    }

    angle_limit_lidar_index_ = declare_parameter("angle_limit_lidar_index", -1);
    RCLCPP_INFO(get_logger(), "angle_limit_lidar_index: %d", angle_limit_lidar_index_);
    if (angle_limit_lidar_index_ > static_cast<int8_t>(input_topics_.size()) - 1) {
      RCLCPP_ERROR(get_logger(), "The angle_limit_lidar_index is out of range.");
      return;
    }
    auto angle_range_deg = declare_parameter("angle_range_deg", std::vector<double>{});
    if (angle_range_deg.size() != 2) {
      RCLCPP_ERROR(get_logger(), "The angle range must be a vector of two elements.");
      return;
    }
    for (const auto & angle_deg : angle_range_deg) {
      RCLCPP_INFO(get_logger(), "angle_range: %f", angle_deg);
      angle_range_.push_back(angle_deg * M_PI / 180.0);
    }
  }

  // Initialize offset map
  {
    for (size_t i = 0; i < input_offset_msec_.size(); ++i) {
      offset_map_msec_[input_topics_[i]] = input_offset_msec_[i];
      std::cout << "offset_map_msec_[" << input_topics_[i] << "] = " << offset_map_msec_[input_topics_[i]] << std::endl;
    }
  }

  // tf2 listener
  {
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  }

  // Output Publishers
  {
    pub_output_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "output", rclcpp::SensorDataQoS().keep_last(maximum_queue_size_));
  }

  // Subscribers
  {
    RCLCPP_INFO_STREAM(get_logger(), "Subscribing to " << input_topics_.size() << " user given topics as inputs:");
    for (auto & input_topic : input_topics_) {
      RCLCPP_INFO_STREAM(get_logger(), " - " << input_topic);
    }

    // Subscribe to the filters
    filters_.resize(input_topics_.size());

    for (size_t d = 0; d < input_topics_.size(); ++d) {
      points_[input_topics_[d]] = nullptr;
      points_stamp_[input_topics_[d]] = 0;

      std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)> cb = std::bind(
        &PointCloudOffsetConcatenationComponent::cloud_callback, this, std::placeholders::_1, input_topics_[d]);

      filters_[d] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topics_[d], rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), cb);
    }
  }
}

int8_t PointCloudOffsetConcatenationComponent::get_topic_index(const std::string & topic_name) const
{
  auto it = std::find(input_topics_.begin(), input_topics_.end(), topic_name);

  if (it != input_topics_.end()) {
    return std::distance(input_topics_.begin(), it);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Topic %s not found in input_topics_", topic_name.c_str());
    return -1;
  }
}

void PointCloudOffsetConcatenationComponent::transformPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in, sensor_msgs::msg::PointCloud2::SharedPtr & out)
{
  transformPointCloud(in, out, output_frame_);
}

void PointCloudOffsetConcatenationComponent::transformPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in, sensor_msgs::msg::PointCloud2::SharedPtr & out,
  const std::string & target_frame)
{
  if (target_frame != in->header.frame_id) {
    if (!pcl_ros::transformPointCloud(target_frame, *in, *out, *tf2_buffer_)) {
      RCLCPP_ERROR(
        this->get_logger(), "[transformPointCloud] Error converting first input dataset from %s to %s.",
        in->header.frame_id.c_str(), target_frame.c_str());
      return;
    }
  } else {
    out = std::make_shared<sensor_msgs::msg::PointCloud2>(*in);
  }
}

void PointCloudOffsetConcatenationComponent::combineClouds(sensor_msgs::msg::PointCloud2::SharedPtr & concat_cloud_ptr)
{
  for (const auto & e : points_) {
    if (e.second != nullptr) {
      sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud_ptr(new sensor_msgs::msg::PointCloud2());
      transformPointCloud(e.second, transformed_cloud_ptr);

      if (concat_cloud_ptr == nullptr) {
        concat_cloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(*transformed_cloud_ptr);
      } else {
        pcl::concatenatePointCloud(*concat_cloud_ptr, *transformed_cloud_ptr, *concat_cloud_ptr);
      }
    }
  }
}

void PointCloudOffsetConcatenationComponent::publish()
{
  sensor_msgs::msg::PointCloud2::SharedPtr concat_cloud_ptr = nullptr;

  combineClouds(concat_cloud_ptr);

  if (concat_cloud_ptr) {
    RCLCPP_INFO(this->get_logger(), "Publishing a pointcloud message with %d points.\n", concat_cloud_ptr->width);
    auto output = std::make_unique<sensor_msgs::msg::PointCloud2>(*concat_cloud_ptr);
    pub_output_->publish(std::move(output));
  } else {
    RCLCPP_WARN(this->get_logger(), "concat_cloud_ptr is nullptr, skipping pointcloud publish.");
  }
}

bool PointCloudOffsetConcatenationComponent::is_in_side_lidar_area(const float x, const float y) const
{
  if (x < 0.0) {
    return false;
  }
  // Check if the point is in the side lidar area based on the x/y angle
  const float angle = std::atan2(y, x);
  const float right_angle = angle_range_[0];
  const float left_angle = angle_range_[1];
  if (angle < right_angle || angle > left_angle) {
    return true;
  }
  return false;
}

void PointCloudOffsetConcatenationComponent::convertToXYZIICloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr, sensor_msgs::msg::PointCloud2::SharedPtr & output_ptr,
  const uint8_t topic_index)
{
  output_ptr->header = input_ptr->header;
  output_ptr->height = input_ptr->height;
  output_ptr->width = input_ptr->width;
  output_ptr->is_dense = input_ptr->is_dense;

  sensor_msgs::PointCloud2Modifier output_modifier(*output_ptr);
  output_modifier.setPointCloud2Fields(
    5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
    sensor_msgs::msg::PointField::FLOAT32, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, "index", 1,
    sensor_msgs::msg::PointField::UINT8);

  output_modifier.reserve(input_ptr->width);

  bool has_intensity = std::any_of(
    input_ptr->fields.begin(), input_ptr->fields.end(), [](auto & field) { return field.name == "intensity"; });

  sensor_msgs::PointCloud2Iterator<float> in_it_x(*input_ptr, "x");
  sensor_msgs::PointCloud2Iterator<float> in_it_y(*input_ptr, "y");
  sensor_msgs::PointCloud2Iterator<float> in_it_z(*input_ptr, "z");

  sensor_msgs::PointCloud2Iterator<float> out_it_x(*output_ptr, "x");
  sensor_msgs::PointCloud2Iterator<float> out_it_y(*output_ptr, "y");
  sensor_msgs::PointCloud2Iterator<float> out_it_z(*output_ptr, "z");
  sensor_msgs::PointCloud2Iterator<float> out_it_intensity(*output_ptr, "intensity");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_it_index(*output_ptr, "index");

  if (has_intensity) {
    sensor_msgs::PointCloud2Iterator<uint8_t> in_it_i(*input_ptr, "intensity");
    for (; in_it_x != in_it_x.end(); ++in_it_x, ++in_it_y, ++in_it_z, ++in_it_i, ++out_it_x, ++out_it_y, ++out_it_z,
                                     ++out_it_intensity, ++out_it_index) {
      if (angle_limit_lidar_index_ >= 0 && topic_index == angle_limit_lidar_index_) {
        if (is_in_side_lidar_area(*in_it_x, *in_it_y)) {
          continue;
        }
      }
      *out_it_x = *in_it_x;
      *out_it_y = *in_it_y;
      *out_it_z = *in_it_z;
      *out_it_intensity = static_cast<float>(*in_it_i);
      *out_it_index = topic_index;
    }
  } else {
    for (; in_it_x != in_it_x.end();
         ++in_it_x, ++in_it_y, ++in_it_z, ++out_it_x, ++out_it_y, ++out_it_z, ++out_it_intensity, ++out_it_index) {
      *out_it_x = *in_it_x;
      *out_it_y = *in_it_y;
      *out_it_z = *in_it_z;
      *out_it_intensity = 0.0f;
      *out_it_index = topic_index;
    }
  }
}

void PointCloudOffsetConcatenationComponent::try_merge_point_clouds(const double stamp_msec)
{
  if (!all_points_received()) {
    return;
  }

  std::unordered_map<std::string, size_t> indices;
  for (const auto & topic : input_topics_) {
    auto idx = lookup_index(points_stamp_buff_[topic], stamp_msec);
    if (idx == -1) {
      return;
    }
    indices[topic] = idx;
    points_[topic] = points_buff_[topic][idx];
    points_stamp_[topic] = points_stamp_buff_[topic][idx];
  }

  RCLCPP_DEBUG(this->get_logger(), "merged point cloud at %f msec", stamp_msec);
  for (const auto & topic : input_topics_) {
    RCLCPP_INFO(
      this->get_logger(), "%s timestamp: %f", topic.c_str(), (points_stamp_[topic] - offset_map_msec_[topic]) * 1e-3);
    points_buff_[topic].erase(points_buff_[topic].begin(), points_buff_[topic].begin() + indices[topic] + 1);
    points_stamp_buff_[topic].erase(
      points_stamp_buff_[topic].begin(), points_stamp_buff_[topic].begin() + indices[topic] + 1);
  }

  publish();
}

bool PointCloudOffsetConcatenationComponent::all_points_received()
{
  for (const auto & topic : input_topics_) {
    if (points_stamp_buff_[topic].empty()) {
      return false;
    }
  }
  return true;
}

int PointCloudOffsetConcatenationComponent::lookup_index(const std::vector<double> & stamp_buff, const double stamp)
{
  constexpr int tolerance_msec = 50;
  int min_offset_msec = tolerance_msec;
  int min_offset_index = -1;
  for (size_t index = 0; index < stamp_buff.size(); ++index) {
    int offset_msec = std::abs(stamp_buff[index] - stamp);
    if (offset_msec <= tolerance_msec && offset_msec < min_offset_msec) {
      min_offset_msec = offset_msec;
      min_offset_index = index;
    }
  }
  return min_offset_index;
}

void PointCloudOffsetConcatenationComponent::cloud_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_ptr, const std::string & topic)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto input = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_ptr);
  sensor_msgs::msg::PointCloud2::SharedPtr xyzii_input_ptr(new sensor_msgs::msg::PointCloud2());
  auto topic_index = get_topic_index(topic);
  if (topic_index == -1) {
    return;
  }
  sensor_msgs::msg::PointCloud2::SharedPtr transformed_in_cloud_ptr(new sensor_msgs::msg::PointCloud2());
  transformPointCloud(input, transformed_in_cloud_ptr);
  convertToXYZIICloud(transformed_in_cloud_ptr, xyzii_input_ptr, static_cast<uint8_t>(topic_index));

  auto points_stamp_msec = input_ptr->header.stamp.sec * 1e3 + input_ptr->header.stamp.nanosec / 1e6;
  RCLCPP_DEBUG(
    get_logger(), "Received a pointcloud message from topic: %s at %f. Point num is %d", topic.c_str(),
    points_stamp_msec, xyzii_input_ptr->width);

  points_buff_[topic].push_back(xyzii_input_ptr);
  points_stamp_buff_[topic].push_back(points_stamp_msec + offset_map_msec_[topic]);
  // check if the buffer size exceeds the maximum buffer size
  for (size_t i = 0; i < points_buff_[topic].size(); ++i) {
    if (points_buff_[topic].size() > maximum_buffer_size_) {
      points_buff_[topic].erase(points_buff_[topic].begin());
      points_stamp_buff_[topic].erase(points_stamp_buff_[topic].begin());
    }
  }

  // chech if we can merge point clouds
  try_merge_point_clouds(points_stamp_msec + offset_map_msec_[topic]);
}

}  // namespace pointcloud_concatenate

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_concatenate::PointCloudOffsetConcatenationComponent)
