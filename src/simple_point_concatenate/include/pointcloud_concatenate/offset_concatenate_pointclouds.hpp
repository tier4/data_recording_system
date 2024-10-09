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

#ifndef POINTCLOUD_CONCATENATE__OFFSET_CONCATENATE_POINTCLOUDS_HPP_
#define POINTCLOUD_CONCATENATE__OFFSET_CONCATENATE_POINTCLOUDS_HPP_

#pragma once

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <tuple>
#include <vector>

// ROS includes
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <pcl/point_types.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace pointcloud_concatenate
{
using point_cloud_msg_wrapper::PointCloud2Modifier;

template <class T>
bool float_eq(const T a, const T b, const T eps = 10e-6)
{
  return std::fabs(a - b) < eps;
}

struct PointXYZIWithIndex
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  uint8_t index{0U};
  friend bool operator==(const PointXYZIWithIndex & p1, const PointXYZIWithIndex & p2) noexcept
  {
    return float_eq<float>(p1.x, p2.x) && float_eq<float>(p1.y, p2.y) && float_eq<float>(p1.z, p2.z) &&
           float_eq<float>(p1.intensity, p2.intensity) && p1.index == p2.index;
  }
};

class PointCloudOffsetConcatenationComponent : public rclcpp::Node
{
public:
  typedef sensor_msgs::msg::PointCloud2 PointCloud2;

  /** \brief constructor. */
  explicit PointCloudOffsetConcatenationComponent(const rclcpp::NodeOptions & node_options);

  /** \brief constructor.
   * \param queue_size the maximum queue size
   */
  PointCloudOffsetConcatenationComponent(const rclcpp::NodeOptions & node_options, int queue_size);

  /** \brief Empty destructor. */
  virtual ~PointCloudOffsetConcatenationComponent() {}

private:
  /** \brief The output PointCloud publisher. */
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;
  /** \brief Delay Compensated PointCloud publisher*/
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> transformed_raw_pc_publisher_map_;

  /** \brief The maximum number of messages that we can store in the queue. */
  int maximum_queue_size_ = 3;
  size_t maximum_buffer_size_ = 20;

  /** \brief A vector of subscriber. */
  std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> filters_;

  /** \brief Output TF frame the concatenated points should be transformed to. */
  std::string output_frame_;

  /** \brief Input point cloud topics. */
  // XmlRpc::XmlRpcValue input_topics_;
  std::vector<std::string> input_topics_;

  /** \brief TF listener object. */
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  std::map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> points_;
  std::map<std::string, double> points_stamp_;
  std::map<std::string, std::vector<sensor_msgs::msg::PointCloud2::SharedPtr>> points_buff_;
  std::map<std::string, std::vector<double>> points_stamp_buff_;

  std::mutex mutex_;

  std::vector<double> input_offset_msec_;
  std::map<std::string, double> offset_map_msec_;
  int8_t angle_limit_lidar_index_;
  std::vector<double> angle_range_;

  int8_t get_topic_index(const std::string & topic_name) const;
  void transformPointCloud(const PointCloud2::ConstSharedPtr & in, PointCloud2::SharedPtr & out);
  void transformPointCloud(
    const PointCloud2::ConstSharedPtr & in, PointCloud2::SharedPtr & out, const std::string & target_frame);
  void combineClouds(sensor_msgs::msg::PointCloud2::SharedPtr & concat_cloud_ptr);
  void publish();

  bool is_in_side_lidar_area(const float x, const float y) const;
  void convertToXYZIICloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr, sensor_msgs::msg::PointCloud2::SharedPtr & output_ptr,
    const uint8_t topic_index);
  void try_merge_point_clouds(const double stamp_msec);
  bool all_points_received();
  int lookup_index(const std::vector<double> & stamp_buff, const double stamp);
  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_ptr, const std::string & topic);
};

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(index);
using PointXYZIWithIndexGenerator = std::tuple<
  point_cloud_msg_wrapper::field_x_generator, point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator, point_cloud_msg_wrapper::field_intensity_generator,
  point_cloud_msg_wrapper::field_id_generator>;

}  // namespace pointcloud_concatenate

POINT_CLOUD_REGISTER_POINT_STRUCT(
  pointcloud_concatenate::PointXYZIWithIndex,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint8_t, index, index))
#endif  // POINTCLOUD_CONCATENATE__OFFSET_CONCATENATE_POINTCLOUDS_HPP_
