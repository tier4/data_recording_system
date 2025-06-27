#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <vector>

class MultiTfPublisher : public rclcpp::Node
{
public:
  explicit MultiTfPublisher(const rclcpp::NodeOptions & options)
    : Node("multi_transform_publisher", options)
  {
    // Declare parameters
    this->declare_parameter<std::string>("config_file", "");
    this->declare_parameter<bool>("publish_camera_optical_link", true);
    this->declare_parameter<bool>("periodic_publish", false);
    this->declare_parameter<double>("publish_period", 0.1);
    
    // Get parameters
    std::string config_file = this->get_parameter("config_file").as_string();
    publish_camera_optical_link_ = this->get_parameter("publish_camera_optical_link").as_bool();
    periodic_publish_ = this->get_parameter("periodic_publish").as_bool();
    double publish_period = this->get_parameter("publish_period").as_double();
    
    if (config_file.empty()) {
      RCLCPP_ERROR(this->get_logger(), "config_file parameter is required");
      rclcpp::shutdown();
      return;
    }
    
    // Create static transform broadcaster
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    // Create timer for periodic publishing if enabled
    if (periodic_publish_) {
      timer_ = this->create_wall_timer(
        std::chrono::duration<double>(publish_period),
        std::bind(&MultiTfPublisher::publishTransforms, this)
      );
      RCLCPP_INFO(this->get_logger(), "Periodic static publishing enabled with period: %.3f seconds", publish_period);
    } else {
      RCLCPP_INFO(this->get_logger(), "One-time static publishing enabled");
    }
    
    // Load transforms
    loadTransforms(config_file);
    
    // If static publishing, publish immediately
    if (!periodic_publish_) {
      publishTransforms();
    }
  }

private:
  void loadTransforms(const std::string& config_file)
  {
    try {
      YAML::Node config = YAML::LoadFile(config_file);
      transforms_.clear();
      
      // Process all transforms in the YAML file
      processYamlNode(config, "", transforms_);
      
      RCLCPP_INFO(this->get_logger(), "Loaded %zu transforms from config file", transforms_.size());
      
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
      rclcpp::shutdown();
    }
  }
  
  void publishTransforms()
  {
    if (transforms_.empty()) {
      return;
    }
    
    // Update timestamps
    auto now = this->get_clock()->now();
    for (auto& transform : transforms_) {
      transform.header.stamp = now;
    }
    
    // Always publish to /tf_static
    tf_static_broadcaster_->sendTransform(transforms_);
    
    if (!periodic_publish_) {
      RCLCPP_INFO(this->get_logger(), "Published %zu static transforms", transforms_.size());
    }
  }
  
  void processYamlNode(const YAML::Node& node, const std::string& parent_frame, 
                       std::vector<geometry_msgs::msg::TransformStamped>& transforms)
  {
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
      std::string frame_id = it->first.as<std::string>();
      
      // If this is a parent frame (contains child frames)
      if (it->second.IsMap() && !hasTransformFields(it->second)) {
        processYamlNode(it->second, frame_id, transforms);
      }
      // If this contains transform data
      else if (it->second.IsMap() && hasTransformFields(it->second)) {
        geometry_msgs::msg::TransformStamped transform = createTransform(parent_frame, frame_id, it->second);
        transforms.push_back(transform);
        
        // If this is a camera link and we need to publish optical link
        if (publish_camera_optical_link_ && frame_id.find("/camera_link") != std::string::npos) {
          geometry_msgs::msg::TransformStamped optical_transform = createCameraOpticalTransform(frame_id);
          transforms.push_back(optical_transform);
        }
      }
    }
  }
  
  bool hasTransformFields(const YAML::Node& node)
  {
    return node["x"] && node["y"] && node["z"] && 
           node["roll"] && node["pitch"] && node["yaw"];
  }
  
  geometry_msgs::msg::TransformStamped createTransform(const std::string& parent_frame, 
                                                       const std::string& child_frame,
                                                       const YAML::Node& transform_data)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    
    // Set translation
    transform.transform.translation.x = transform_data["x"].as<double>();
    transform.transform.translation.y = transform_data["y"].as<double>();
    transform.transform.translation.z = transform_data["z"].as<double>();
    
    // Convert Euler angles to quaternion
    double roll = transform_data["roll"].as<double>();
    double pitch = transform_data["pitch"].as<double>();
    double yaw = transform_data["yaw"].as<double>();
    
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    return transform;
  }
  
  geometry_msgs::msg::TransformStamped createCameraOpticalTransform(const std::string& camera_link_frame)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = camera_link_frame;
    
    // Extract camera name from frame_id (e.g., "camera0/camera_link" -> "camera0/camera_optical_link")
    std::string optical_frame = camera_link_frame;
    size_t pos = optical_frame.find("/camera_link");
    if (pos != std::string::npos) {
      optical_frame.replace(pos, std::string("/camera_link").length(), "/camera_optical_link");
    }
    transform.child_frame_id = optical_frame;
    
    // No translation
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    
    // Rotation: -90 degrees around Z, then -90 degrees around X
    // This is equivalent to the quaternion (0.5, -0.5, 0.5, -0.5)
    transform.transform.rotation.x = 0.5;
    transform.transform.rotation.y = -0.5;
    transform.transform.rotation.z = 0.5;
    transform.transform.rotation.w = -0.5;
    
    return transform;
  }
  
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<geometry_msgs::msg::TransformStamped> transforms_;
  bool publish_camera_optical_link_;
  bool periodic_publish_;
};

// Register as component
RCLCPP_COMPONENTS_REGISTER_NODE(MultiTfPublisher)