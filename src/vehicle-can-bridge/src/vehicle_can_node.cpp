// Copyright 2026 TIER IV, Inc.

#include "vehicle_can_bridge/vehicle_can_node.hpp"

#include "vehicle_can_bridge/msg/signal.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace vehicle_can_bridge
{

// ── Constructor ───────────────────────────────────────────────────────────────

VehicleCanNode::VehicleCanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vehicle_can_node", options)
{
  declare_parameters();
  load_parameters();
  setup_publishers();

  // Open the CAN interface; log a warning but continue if it fails so the
  // node can still be launched in simulation environments without CAN hardware.
  if (!open_can_interface()) {
    RCLCPP_WARN(
      get_logger(),
      "Could not open CAN interface '%s'. Frames will not be received. "
      "Use vcan for simulation: 'sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && "
      "sudo ip link set up vcan0'",
      can_interface_.c_str());
  }

  setup_timer();

  RCLCPP_INFO(
    get_logger(), "vehicle_can_node started [vehicle=%s, interface=%s, dbc=%s]",
    vehicle_id_.c_str(), can_interface_.c_str(), dbc_file_.c_str());
}

VehicleCanNode::~VehicleCanNode() = default;

// ── Parameter declaration ─────────────────────────────────────────────────────

void VehicleCanNode::declare_parameters()
{
  declare_parameter("vehicle_id", "unknown_vehicle");
  declare_parameter("can_interface", "can0");
  declare_parameter("dbc_file", "");
  declare_parameter("loop_rate_hz", 100.0);
  declare_parameter("signal_timeout_ms", 500);
  declare_parameter("publish_all_signals", true);
  declare_parameter("all_signals_topic", "/vehicle/signals/all");
  declare_parameter("diagnostics_topic", "/vehicle/diagnostics");
  declare_parameter("diagnostics_rate_hz", 1.0);
}

// ── Parameter loading ─────────────────────────────────────────────────────────

void VehicleCanNode::load_parameters()
{
  vehicle_id_ = get_parameter("vehicle_id").as_string();
  can_interface_ = get_parameter("can_interface").as_string();
  dbc_file_ = get_parameter("dbc_file").as_string();
  loop_rate_hz_ = get_parameter("loop_rate_hz").as_double();
  diagnostics_rate_hz_ = get_parameter("diagnostics_rate_hz").as_double();

  // Validate rates before using them in chrono arithmetic (zero/negative would
  // produce infinity or a negative period, causing UB in duration_cast).
  if (loop_rate_hz_ <= 0.0) {
    throw std::runtime_error(
      "Parameter 'loop_rate_hz' must be positive, got: " + std::to_string(loop_rate_hz_));
  }
  if (diagnostics_rate_hz_ <= 0.0) {
    throw std::runtime_error(
      "Parameter 'diagnostics_rate_hz' must be positive, got: " +
      std::to_string(diagnostics_rate_hz_));
  }

  const int64_t raw_timeout = get_parameter("signal_timeout_ms").as_int();
  if (raw_timeout <= 0) {
    throw std::runtime_error(
      "Parameter 'signal_timeout_ms' must be positive, got: " + std::to_string(raw_timeout));
  }
  signal_timeout_ms_ = static_cast<uint64_t>(raw_timeout);

  publish_all_signals_ = get_parameter("publish_all_signals").as_bool();
  all_signals_topic_ = get_parameter("all_signals_topic").as_string();
  diagnostics_topic_ = get_parameter("diagnostics_topic").as_string();

  // ── DBC file ────────────────────────────────────────────────────────────────
  if (dbc_file_.empty()) {
    throw std::runtime_error("Parameter 'dbc_file' must not be empty.");
  }
  if (!decoder_.load(dbc_file_)) {
    throw std::runtime_error("Failed to load DBC file: " + dbc_file_);
  }
  RCLCPP_INFO(
    get_logger(), "DBC loaded: %s (%zu messages)", dbc_file_.c_str(), decoder_.known_ids().size());

  // ── Domain config ────────────────────────────────────────────────────────────
  // Domains are encoded as flat YAML parameter arrays:
  //   domains.chassis.topic    : "/vehicle/chassis"
  //   domains.chassis.can_ids  : [256, 257]
  // We iterate by reading the domain names from a helper parameter.
  // The user provides domain_names as a string array:
  //   domain_names: ["chassis", "powertrain", "body"]
  declare_parameter("domain_names", std::vector<std::string>{});
  const auto domain_names = get_parameter("domain_names").as_string_array();

  std::vector<DomainConfig> domains;
  for (const auto & name : domain_names) {
    const std::string topic_param = "domains." + name + ".topic";
    const std::string ids_param = "domains." + name + ".can_ids";

    declare_parameter(topic_param, "/vehicle/" + name);
    declare_parameter(ids_param, std::vector<int64_t>{});

    const std::string topic = get_parameter(topic_param).as_string();
    const auto id_list = get_parameter(ids_param).as_integer_array();

    DomainConfig dc;
    dc.name = name;
    dc.topic = topic;
    for (const int64_t id : id_list) {
      dc.can_ids.insert(static_cast<uint32_t>(id));
    }
    domains.push_back(std::move(dc));
  }

  // ── Aliases ──────────────────────────────────────────────────────────────────
  // aliases are encoded as:
  //   alias_names: ["StrAng_Sns", "WhlSpd_FL"]
  //   aliases.StrAng_Sns: "steering_angle"
  declare_parameter("alias_names", std::vector<std::string>{});
  const auto alias_names = get_parameter("alias_names").as_string_array();

  std::unordered_map<std::string, std::string> aliases;
  for (const auto & src : alias_names) {
    const std::string param = "aliases." + src;
    declare_parameter(param, src);  // default to same name
    aliases[src] = get_parameter(param).as_string();
  }

  const auto routing_warnings = router_.configure(domains, aliases);
  for (const auto & w : routing_warnings) {
    RCLCPP_WARN(get_logger(), "Signal routing: %s", w.c_str());
  }

  // ── Transforms ───────────────────────────────────────────────────────────────
  // transform_names: ["VehicleSpeed", "SteeringAngle"]
  // transforms.VehicleSpeed.expression: "x / 3.6"
  // transforms.VehicleSpeed.unit: "m/s"
  declare_parameter("transform_names", std::vector<std::string>{});
  const auto transform_names = get_parameter("transform_names").as_string_array();

  std::unordered_map<std::string, TransformConfig> transforms;
  for (const auto & sig : transform_names) {
    const std::string expr_param = "transforms." + sig + ".expression";
    const std::string unit_param = "transforms." + sig + ".unit";
    declare_parameter(expr_param, "");
    declare_parameter(unit_param, "");

    TransformConfig cfg;
    cfg.expression = get_parameter(expr_param).as_string();
    cfg.unit = get_parameter(unit_param).as_string();
    transforms[sig] = std::move(cfg);
  }

  transformer_.configure(transforms);  // throws on bad expression

  // ── Promoted signals ─────────────────────────────────────────────────────────
  // promoted_signal_names: ["VehicleSpeed", "SteeringAngle"]
  // promoted_signals.VehicleSpeed.topic_suffix: "vehicle_speed"
  declare_parameter("promoted_signal_names", std::vector<std::string>{});
  const auto promoted_names = get_parameter("promoted_signal_names").as_string_array();

  for (const auto & sig : promoted_names) {
    const std::string suffix_param = "promoted_signals." + sig + ".topic_suffix";
    declare_parameter(suffix_param, sig);
    PromotedSignalConfig pc;
    pc.dbc_name = sig;
    pc.topic_suffix = get_parameter(suffix_param).as_string();
    promoted_signals_.push_back(std::move(pc));
  }
}

// ── Publisher setup ───────────────────────────────────────────────────────────

void VehicleCanNode::setup_publishers()
{
  // Firehose
  if (publish_all_signals_) {
    all_signals_pub_ = create_publisher<msg::SignalGroup>(all_signals_topic_, 10);
  }

  // Per-domain publishers — create one for every domain the router knows about
  const auto domain_names = get_parameter("domain_names").as_string_array();
  for (const auto & name : domain_names) {
    const std::string topic = router_.topic_for_domain(name);
    if (!topic.empty()) {
      domain_pubs_[name] = create_publisher<msg::SignalGroup>(topic, 10);
    }
  }

  // Promoted signal publishers
  for (const auto & pc : promoted_signals_) {
    const std::string topic = "/vehicle/signals/" + pc.topic_suffix;
    promoted_pubs_[pc.dbc_name] = create_publisher<std_msgs::msg::Float64>(topic, 10);
  }

  // Diagnostics
  diagnostics_pub_ = create_publisher<msg::SignalDiagnostic>(diagnostics_topic_, 10);
}

// ── Timer setup ──────────────────────────────────────────────────────────────

void VehicleCanNode::setup_timer()
{
  const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
  spin_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&VehicleCanNode::on_timer, this));

  const auto diag_period = std::chrono::duration<double>(1.0 / diagnostics_rate_hz_);
  diagnostics_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(diag_period),
    std::bind(&VehicleCanNode::on_diagnostics_timer, this));
}

// ── CAN interface open ────────────────────────────────────────────────────────

bool VehicleCanNode::open_can_interface() { return can_reader_.open(can_interface_); }

// ── Timer callback ────────────────────────────────────────────────────────────

void VehicleCanNode::on_timer()
{
  // Drain all available CAN frames in this tick
  while (true) {
    const auto frame = can_reader_.read_frame();
    if (!frame.has_value()) {
      // Distinguish real socket errors from an empty buffer (CRITICAL-2 fix).
      if (can_reader_.has_error()) {
        RCLCPP_ERROR(
          get_logger(),
          "SocketCAN read error on '%s': %s (errno=%d). "
          "CAN data loss until the interface recovers.",
          can_interface_.c_str(), std::strerror(can_reader_.last_error()),
          can_reader_.last_error());
        ++decode_errors_;
        can_reader_.clear_error();
      }
      break;
    }
    process_frame(*frame);
  }

  // Publish accumulated per-domain signal groups
  flush_pending_groups();

  // Check for signal timeouts (newly timed-out names available if needed)
  (void)timeout_monitor_.check_timeouts(now_ms(), signal_timeout_ms_);
}

// ── Process a single CAN frame ────────────────────────────────────────────────

void VehicleCanNode::process_frame(const CanFrame & frame)
{
  ++frames_received_;

  // Fast pre-filter: skip if ID is not in any domain (still try decode for
  // the "unassigned" domain to capture unknowns in the firehose)
  const auto decoded = decoder_.decode(frame.id, frame.data, frame.dlc);
  if (!decoded.has_value()) {
    ++frames_unknown_;
    return;
  }

  ++frames_decoded_;

  const std::string & domain = router_.domain_for_id(frame.id);
  const rclcpp::Time stamp = now();

  for (const RawSignal & raw_sig : *decoded) {
    // Apply alias
    const std::string & name = router_.apply_alias(raw_sig.name);

    // Apply transform; catch expression evaluation errors (MEDIUM-3)
    TransformResult tr;
    try {
      tr = transformer_.transform(name, raw_sig.value);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Transform error for signal '%s': %s", name.c_str(), e.what());
      ++decode_errors_;
      continue;
    }

    // Update timeout monitor
    timeout_monitor_.signal_received(name, now_ms());

    // Build Signal message
    msg::Signal sig_msg;
    sig_msg.name = name;
    sig_msg.value = tr.value;
    sig_msg.raw_value = tr.raw_value;
    sig_msg.unit = tr.unit;
    sig_msg.can_id = frame.id;
    sig_msg.status = msg::Signal::STATUS_OK;
    sig_msg.timestamp_can = frame.timestamp;

    // Accumulate into domain batch (skip unassigned to avoid silent accumulation
    // of signals that have no publisher — they still go to the firehose below)
    if (domain != SignalRouter::kUnassignedDomain) {
      pending_signals_[domain].push_back(sig_msg);
    }

    // Publish promoted signal if configured
    {
      auto it = promoted_pubs_.find(name);
      if (it != promoted_pubs_.end()) {
        std_msgs::msg::Float64 f64;
        f64.data = tr.value;
        it->second->publish(f64);
      }
    }

    // Accumulate into firehose batch as well (domain "all")
    if (publish_all_signals_) {
      pending_signals_["__all__"].push_back(sig_msg);
    }
  }
}

// ── Flush pending domain groups ───────────────────────────────────────────────

void VehicleCanNode::flush_pending_groups()
{
  const rclcpp::Time stamp = now();

  for (auto & [domain, signals] : pending_signals_) {
    if (signals.empty()) {
      continue;
    }

    msg::SignalGroup group;
    group.header.stamp = stamp;
    group.header.frame_id = "";
    group.vehicle_id = vehicle_id_;
    group.signals = signals;

    if (domain == "__all__") {
      group.domain = "all";
      if (all_signals_pub_) {
        all_signals_pub_->publish(group);
      }
    } else {
      group.domain = domain;
      auto pub_it = domain_pubs_.find(domain);
      if (pub_it != domain_pubs_.end()) {
        pub_it->second->publish(group);
      } else if (domain == SignalRouter::kUnassignedDomain) {
        // Unassigned signals: only publish on firehose, not a separate topic
      }
    }

    signals.clear();
  }
}

// ── Diagnostics timer callback ────────────────────────────────────────────────

void VehicleCanNode::on_diagnostics_timer()
{
  msg::SignalDiagnostic diag;
  diag.header.stamp = now();
  diag.vehicle_id = vehicle_id_;
  diag.can_interface = can_interface_;
  diag.frames_received = frames_received_;
  diag.frames_decoded = frames_decoded_;
  diag.frames_unknown = frames_unknown_;
  diag.decode_errors = decode_errors_;

  const auto ts = timeout_monitor_.timed_out_signals();
  diag.timed_out_signals = ts;

  diagnostics_pub_->publish(diag);
}

// ── now_ms helper ─────────────────────────────────────────────────────────────

uint64_t VehicleCanNode::now_ms() const
{
  const int64_t ns = now().nanoseconds();
  // nanoseconds() is int64_t; cast to uint64_t before division to avoid the
  // implicit signed→unsigned conversion that would wrap negative values.
  return (ns >= 0) ? (static_cast<uint64_t>(ns) / 1'000'000ULL) : 0ULL;
}

}  // namespace vehicle_can_bridge
