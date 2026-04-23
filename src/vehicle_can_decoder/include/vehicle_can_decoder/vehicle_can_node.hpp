// Copyright 2026 TIER IV, Inc.

#pragma once

#include "vehicle_can_decoder/can_reader.hpp"
#include "vehicle_can_decoder/dbc_decoder.hpp"
#include "vehicle_can_decoder/msg/signal_diagnostic.hpp"
#include "vehicle_can_decoder/msg/signal_group.hpp"
#include "vehicle_can_decoder/msg/vehicle_schema.hpp"
#include "vehicle_can_decoder/signal_router.hpp"
#include "vehicle_can_decoder/signal_transformer.hpp"
#include "vehicle_can_decoder/timeout_monitor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace vehicle_can_decoder
{

/// Configuration for a single promoted signal.
struct PromotedSignalConfig
{
  std::string dbc_name;      ///< DBC (or aliased) signal name to promote
  std::string topic_suffix;  ///< Suffix used to form the topic name
};

/// ROS2 node that:
///   1. Subscribes to a can_msgs/Frame topic
///   2. Decodes CAN frames using a DBC file (dbcppp)
///   3. Transforms signals via exprtk expressions
///   4. Routes signals to domain-grouped SignalGroup topics
///   5. Optionally promotes individual signals to std_msgs/Float64 topics
///   6. Publishes diagnostics
class VehicleCanNode : public rclcpp::Node
{
public:
  explicit VehicleCanNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~VehicleCanNode() override;

private:
  // ── Initialisation ──────────────────────────────────────────────────────────
  void declare_parameters();
  void load_parameters();
  void setup_publishers();
  void setup_diagnostics_timer();
  void setup_schema_timer();
  void publish_schema();

  // ── Runtime callbacks ───────────────────────────────────────────────────────
  void on_diagnostics_timer();
  void on_can_frame(const can_msgs::msg::Frame::SharedPtr msg);

  // ── Per-frame processing ────────────────────────────────────────────────────
  void process_frame(const CanFrame & frame, const rclcpp::Time & stamp);

  // ── Helpers ─────────────────────────────────────────────────────────────────
  uint64_t now_ms() const;

  // ── Parameters ──────────────────────────────────────────────────────────────
  std::string vehicle_id_;
  std::string can_topic_;
  std::string dbc_file_;
  uint64_t signal_timeout_ms_;
  bool publish_all_signals_;
  std::string all_signals_topic_;
  std::string diagnostics_topic_;
  double diagnostics_rate_hz_;
  bool schema_publish_per_domain_{true};
  double schema_republish_interval_s_{1.0};
  std::vector<PromotedSignalConfig> promoted_signals_;

  // ── Vehicle schema (optional) ────────────────────────────────────────────────
  // Loaded from schema_domain_names + schema.<name>.{topic,signals} parameters.
  // When non-empty, enables schema mode:
  //   - Routing is by canonical signal name (not CAN ID)
  //   - Signals not in the schema are discarded (not forwarded to any topic)
  //
  // Compound alias keys: when two DBC messages share the same signal name,
  // use "CAN{decimal_id}_{signal_name}" as the alias_names key to disambiguate.
  // Example: "CAN556_OUTPUT_VALUE" maps only STEERING_RPT's OUTPUT_VALUE.

  /// Domain name → ordered list of canonical signal names (from schema).
  std::unordered_map<std::string, std::vector<std::string>> domain_schema_;

  /// Canonical signal name → domain name (reverse index of domain_schema_).
  std::unordered_map<std::string, std::string> signal_to_domain_;

  /// Domain name → ROS2 topic (from schema params, may differ from domains.*).
  std::unordered_map<std::string, std::string> schema_domain_topics_;

  // ── Signal / unit ID tables ──────────────────────────────────────────────────
  std::string schema_version_;
  std::unordered_map<std::string, uint16_t> signal_name_to_id_;  ///< name → 1-indexed id
  std::unordered_map<uint16_t, uint16_t> signal_id_to_unit_id_;  ///< signal_id → unit_id
  std::unordered_map<std::string, uint16_t> unit_name_to_id_;    ///< unit string → 1-indexed id
  std::vector<std::string> unit_id_names_;  ///< ordered unit strings (index=id-1)

  // ── Core components ─────────────────────────────────────────────────────────
  DbcDecoder decoder_;
  SignalTransformer transformer_;
  SignalRouter router_;
  TimeoutMonitor timeout_monitor_;

  // ── Publishers ──────────────────────────────────────────────────────────────
  // Domain name → publisher
  std::unordered_map<std::string, rclcpp::Publisher<msg::SignalGroup>::SharedPtr> domain_pubs_;

  // Firehose publisher (all signals)
  rclcpp::Publisher<msg::SignalGroup>::SharedPtr all_signals_pub_;

  // Promoted signal name → publisher
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
    promoted_pubs_;

  // Diagnostics publisher
  rclcpp::Publisher<msg::SignalDiagnostic>::SharedPtr diagnostics_pub_;

  // Schema publisher (transient_local; published once at startup)
  rclcpp::Publisher<msg::VehicleSchema>::SharedPtr schema_pub_;

  // ── Subscriber ────────────────────────────────────────────────────────────
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

  // ── Timers ───────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr schema_timer_;

  // ── Diagnostic counters ───────────────────────────────────────────────────
  uint64_t frames_received_{0};
  uint64_t frames_decoded_{0};
  uint64_t frames_unknown_{0};
  uint64_t decode_errors_{0};
};

}  // namespace vehicle_can_decoder
