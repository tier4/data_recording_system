// Copyright 2026 TIER IV, Inc.

#include "vehicle_can_decoder/vehicle_can_node.hpp"

#include "vehicle_can_decoder/msg/signal.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace vehicle_can_decoder
{

// ── Constructor ───────────────────────────────────────────────────────────────

VehicleCanNode::VehicleCanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vehicle_can_node", options)
{
  declare_parameters();
  load_parameters();
  setup_publishers();
  publish_schema();

  if (use_can_topic_) {
    can_sub_ = create_subscription<can_msgs::msg::Frame>(
      can_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VehicleCanNode::on_can_frame, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "vehicle_can_node started in topic mode [vehicle=%s, topic=%s, dbc=%s]",
      vehicle_id_.c_str(), can_topic_.c_str(), dbc_file_.c_str());
  } else {
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
    RCLCPP_INFO(
      get_logger(), "vehicle_can_node started [vehicle=%s, interface=%s, dbc=%s]",
      vehicle_id_.c_str(), can_interface_.c_str(), dbc_file_.c_str());
  }

  setup_timer();
}

VehicleCanNode::~VehicleCanNode() = default;

// ── Parameter declaration ─────────────────────────────────────────────────────

void VehicleCanNode::declare_parameters()
{
  declare_parameter("vehicle_id", "unknown_vehicle");
  declare_parameter("can_interface", "can0");
  declare_parameter("use_can_topic", true);
  declare_parameter("can_topic", "/vehicle/from_can_bus");
  declare_parameter("dbc_file", "");
  declare_parameter("loop_rate_hz", 20.0);
  declare_parameter("signal_timeout_ms", 500);
  declare_parameter("publish_all_signals", true);
  declare_parameter("all_signals_topic", "/vehicle/decoded_can");
  declare_parameter("diagnostics_topic", "/vehicle/diagnostics");
  declare_parameter("diagnostics_rate_hz", 1.0);
  declare_parameter("schema_domain_names", std::vector<std::string>{});
  declare_parameter("schema_publish_per_domain", true);
  declare_parameter("schema_version", std::string(""));
  declare_parameter("signal_id_names", std::vector<std::string>{});
  declare_parameter("signal_id_unit_names", std::vector<std::string>{});
  declare_parameter("unit_id_names", std::vector<std::string>{});
}

// ── Parameter loading ─────────────────────────────────────────────────────────

void VehicleCanNode::load_parameters()
{
  vehicle_id_ = get_parameter("vehicle_id").as_string();
  can_interface_ = get_parameter("can_interface").as_string();
  use_can_topic_ = get_parameter("use_can_topic").as_bool();
  can_topic_ = get_parameter("can_topic").as_string();
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
  schema_publish_per_domain_ = get_parameter("schema_publish_per_domain").as_bool();

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

  // ── Vehicle schema (optional) ─────────────────────────────────────────────────
  // schema_domain_names: [operation, dynamics, ...]
  // schema.operation.topic: /vehicle/operation
  // schema.operation.signals: [operation.steering.command.angle, ...]
  //
  // When schema_domain_names is non-empty, schema mode is active:
  //   - Signals are routed by canonical name (signal_to_domain_) instead of CAN ID.
  //   - Each domain always publishes a full SignalGroup; absent signals get STATUS_INITIAL.
  const auto schema_names = get_parameter("schema_domain_names").as_string_array();
  for (const auto & name : schema_names) {
    const std::string topic_param = "schema." + name + ".topic";
    const std::string sigs_param = "schema." + name + ".signals";
    declare_parameter(topic_param, "/vehicle/" + name);
    declare_parameter(sigs_param, std::vector<std::string>{});

    schema_domain_topics_[name] = get_parameter(topic_param).as_string();
    domain_schema_[name] = get_parameter(sigs_param).as_string_array();
    for (const auto & sig : domain_schema_.at(name)) {
      signal_to_domain_[sig] = name;
    }
  }
  if (!schema_names.empty()) {
    RCLCPP_INFO(
      get_logger(), "Schema mode active: %zu domains, %zu canonical signals", schema_names.size(),
      signal_to_domain_.size());
  }

  // ── Signal / unit ID tables ───────────────────────────────────────────────────
  schema_version_ = get_parameter("schema_version").as_string();

  const auto signal_id_names_list = get_parameter("signal_id_names").as_string_array();
  const auto signal_id_unit_names_list = get_parameter("signal_id_unit_names").as_string_array();
  unit_id_names_ = get_parameter("unit_id_names").as_string_array();

  if (
    !signal_id_names_list.empty() &&
    signal_id_unit_names_list.size() != signal_id_names_list.size()) {
    throw std::runtime_error("signal_id_unit_names must have the same length as signal_id_names");
  }

  for (uint16_t i = 0; i < static_cast<uint16_t>(unit_id_names_.size()); ++i) {
    unit_name_to_id_[unit_id_names_[i]] = static_cast<uint16_t>(i + 1);
  }

  for (uint16_t i = 0; i < static_cast<uint16_t>(signal_id_names_list.size()); ++i) {
    const uint16_t signal_id = static_cast<uint16_t>(i + 1);
    signal_name_to_id_[signal_id_names_list[i]] = signal_id;

    if (!signal_id_unit_names_list.empty()) {
      const auto unit_it = unit_name_to_id_.find(signal_id_unit_names_list[i]);
      if (unit_it != unit_name_to_id_.end()) {
        signal_id_to_unit_id_[signal_id] = unit_it->second;
      } else {
        RCLCPP_WARN(
          get_logger(), "Unit '%s' for signal '%s' not in unit_id_names",
          signal_id_unit_names_list[i].c_str(), signal_id_names_list[i].c_str());
      }
    }
  }

  if (!signal_id_names_list.empty()) {
    RCLCPP_INFO(
      get_logger(), "Signal ID table loaded: %zu signals, %zu units [schema %s]",
      signal_name_to_id_.size(), unit_name_to_id_.size(), schema_version_.c_str());
  }
}

// ── Publisher setup ───────────────────────────────────────────────────────────

void VehicleCanNode::setup_publishers()
{
  // Firehose
  if (publish_all_signals_) {
    all_signals_pub_ = create_publisher<msg::SignalGroup>(all_signals_topic_, 10);
  }

  // Per-domain publishers — legacy CAN-ID-routed domains
  const auto domain_names = get_parameter("domain_names").as_string_array();
  for (const auto & name : domain_names) {
    const std::string topic = router_.topic_for_domain(name);
    if (!topic.empty()) {
      domain_pubs_[name] = create_publisher<msg::SignalGroup>(topic, 10);
    }
  }

  // Schema domain publishers — skipped when schema_publish_per_domain is false
  // (all schema signals are instead published on the firehose topic)
  if (schema_publish_per_domain_) {
    for (const auto & [name, topic] : schema_domain_topics_) {
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

  // Schema (transient_local: late subscribers always receive the current schema)
  schema_pub_ =
    create_publisher<msg::VehicleSchema>("/vehicle/schema", rclcpp::QoS(1).transient_local());
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
  if (!use_can_topic_) {
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

  const rclcpp::Time stamp = now();

  for (const RawSignal & raw_sig : *decoded) {
    // Apply alias: try compound key "CAN{id}_{signal}" first to disambiguate
    // signals that share the same name across different CAN messages (e.g.,
    // PACMod's OUTPUT_VALUE appearing in ACCEL_RPT, BRAKE_RPT, STEERING_RPT).
    const std::string compound_key = "CAN" + std::to_string(frame.id) + "_" + raw_sig.name;
    const std::string & compound_alias = router_.apply_alias(compound_key);
    const std::string & name =
      (compound_alias != compound_key) ? compound_alias : router_.apply_alias(raw_sig.name);

    // Determine domain: schema-based routing by canonical signal name takes
    // precedence over legacy CAN-ID-based routing.
    std::string domain;
    if (!signal_to_domain_.empty()) {
      const auto it = signal_to_domain_.find(name);
      domain =
        (it != signal_to_domain_.end()) ? it->second : std::string(SignalRouter::kUnassignedDomain);
    } else {
      domain = router_.domain_for_id(frame.id);
    }

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
    {
      const auto id_it = signal_name_to_id_.find(name);
      sig_msg.name_id = (id_it != signal_name_to_id_.end()) ? id_it->second : 0;
    }
    sig_msg.value = static_cast<float>(tr.value);
    sig_msg.status = msg::Signal::STATUS_OK;
    sig_msg.timestamp_can = frame.timestamp;

    // Accumulate domain-assigned signals into the domain batch and firehose.
    // Unassigned signals (not in the schema) are discarded — they are not
    // forwarded to the firehose so that /vehicle/decoded_can only contains
    // the canonical signals defined in vehicle_schema.yaml.
    if (domain != SignalRouter::kUnassignedDomain) {
      pending_signals_[domain].push_back(sig_msg);

      if (publish_all_signals_) {
        pending_signals_["__all__"].push_back(sig_msg);
      }
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
  }
}

// ── Flush pending domain groups ───────────────────────────────────────────────

void VehicleCanNode::flush_pending_groups()
{
  const rclcpp::Time stamp = now();

  if (!domain_schema_.empty()) {
    // ── Schema mode ─────────────────────────────────────────────────────────────
    // Every schema-defined domain is published every tick. Signals not received
    // from the DBC are pre-filled with STATUS_INITIAL so downstream nodes always
    // see the same SignalGroup structure regardless of which DBC is loaded.

    for (const auto & [domain, expected] : domain_schema_) {
      // Index received signals by name_id for O(1) overlay
      std::unordered_map<uint16_t, msg::Signal> rx;
      if (auto it = pending_signals_.find(domain); it != pending_signals_.end()) {
        for (auto & s : it->second) {
          rx[s.name_id] = std::move(s);
        }
        it->second.clear();
      }

      msg::SignalGroup group;
      group.header.stamp = stamp;
      group.header.frame_id = "";
      group.vehicle_id = vehicle_id_;
      group.domain = domain;
      group.signals.reserve(expected.size());

      for (const auto & sig_name : expected) {
        const auto id_it = signal_name_to_id_.find(sig_name);
        if (id_it == signal_name_to_id_.end()) {
          RCLCPP_WARN_ONCE(get_logger(), "Schema signal '%s' has no ID entry", sig_name.c_str());
          continue;
        }
        const uint16_t id = id_it->second;
        auto rx_it = rx.find(id);
        if (rx_it != rx.end()) {
          group.signals.push_back(std::move(rx_it->second));
        } else {
          // Signal not present in this DBC: publish as STATUS_INITIAL
          msg::Signal init;
          init.name_id = id;
          init.value = 0.0f;
          init.status = msg::Signal::STATUS_INITIAL;
          init.timestamp_can = 0.0;
          group.signals.push_back(std::move(init));
        }
      }

      if (auto pub_it = domain_pubs_.find(domain); pub_it != domain_pubs_.end()) {
        pub_it->second->publish(group);
      }
    }

    // Firehose: publish all received signals (only actually-received ones)
    auto & all_batch = pending_signals_["__all__"];
    if (publish_all_signals_ && all_signals_pub_ && !all_batch.empty()) {
      msg::SignalGroup all_group;
      all_group.header.stamp = stamp;
      all_group.header.frame_id = "";
      all_group.vehicle_id = vehicle_id_;
      all_group.domain = "all";
      all_group.signals = std::move(all_batch);
      all_signals_pub_->publish(all_group);
    }
    all_batch.clear();
    return;
  }

  // ── Legacy mode (CAN-ID-routed domains) ─────────────────────────────────────
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

// ── can_msgs topic callback ───────────────────────────────────────────────────

void VehicleCanNode::on_can_frame(const can_msgs::msg::Frame::SharedPtr msg)
{
  CanFrame frame{};
  frame.id = msg->id;
  frame.dlc = msg->dlc;
  std::copy(msg->data.begin(), msg->data.end(), frame.data.begin());
  frame.timestamp = static_cast<double>(msg->header.stamp.sec) +
                    static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
  process_frame(frame);
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

// ── Schema publisher ──────────────────────────────────────────────────────────

void VehicleCanNode::publish_schema()
{
  msg::VehicleSchema schema;
  schema.header.stamp = now();
  schema.vehicle_id = vehicle_id_;
  schema.schema_version = schema_version_;

  // Build signal_table sorted by id so signal_table[name_id-1] is valid.
  std::vector<std::pair<uint16_t, std::string>> id_name_pairs;
  id_name_pairs.reserve(signal_name_to_id_.size());
  for (const auto & [name, id] : signal_name_to_id_) {
    id_name_pairs.emplace_back(id, name);
  }
  std::sort(id_name_pairs.begin(), id_name_pairs.end());

  schema.signal_table.reserve(id_name_pairs.size());
  for (const auto & [id, name] : id_name_pairs) {
    msg::SignalEntry entry;
    entry.id = id;
    entry.name = name;
    const auto unit_it = signal_id_to_unit_id_.find(id);
    entry.unit_id = (unit_it != signal_id_to_unit_id_.end()) ? unit_it->second : 0;
    schema.signal_table.push_back(std::move(entry));
  }

  // Build unit_table: 1-indexed, so unit_table[unit_id-1] = unit string.
  schema.unit_table = unit_id_names_;

  schema_pub_->publish(schema);
}

// ── now_ms helper ─────────────────────────────────────────────────────────────

uint64_t VehicleCanNode::now_ms() const
{
  const int64_t ns = now().nanoseconds();
  // nanoseconds() is int64_t; cast to uint64_t before division to avoid the
  // implicit signed→unsigned conversion that would wrap negative values.
  return (ns >= 0) ? (static_cast<uint64_t>(ns) / 1'000'000ULL) : 0ULL;
}

}  // namespace vehicle_can_decoder
