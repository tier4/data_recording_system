// Copyright 2026 TIER IV, Inc.

#include "vehicle_can_decoder/vehicle_can_node.hpp"

#include "vehicle_can_decoder/msg/signal.hpp"

#include <algorithm>
#include <chrono>
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

  can_sub_ = create_subscription<can_msgs::msg::Frame>(
    can_topic_, rclcpp::SensorDataQoS(),
    std::bind(&VehicleCanNode::on_can_frame, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "vehicle_can_node started [vehicle=%s, topic=%s, dbc=%s]", vehicle_id_.c_str(),
    can_topic_.c_str(), dbc_file_.c_str());

  setup_diagnostics_timer();
}

VehicleCanNode::~VehicleCanNode() = default;

// ── Parameter declaration ─────────────────────────────────────────────────────

void VehicleCanNode::declare_parameters()
{
  declare_parameter("vehicle_id", "unknown_vehicle");
  declare_parameter("can_topic", "/vehicle/from_can_bus");
  declare_parameter("dbc_file", "");
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
  can_topic_ = get_parameter("can_topic").as_string();
  dbc_file_ = get_parameter("dbc_file").as_string();
  diagnostics_rate_hz_ = get_parameter("diagnostics_rate_hz").as_double();

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
  declare_parameter("alias_names", std::vector<std::string>{});
  const auto alias_names = get_parameter("alias_names").as_string_array();

  std::unordered_map<std::string, std::string> aliases;
  for (const auto & src : alias_names) {
    const std::string param = "aliases." + src;
    declare_parameter(param, src);
    aliases[src] = get_parameter(param).as_string();
  }

  const auto routing_warnings = router_.configure(domains, aliases);
  for (const auto & w : routing_warnings) {
    RCLCPP_WARN(get_logger(), "Signal routing: %s", w.c_str());
  }

  // ── Transforms ───────────────────────────────────────────────────────────────
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

  transformer_.configure(transforms);

  // ── Promoted signals ─────────────────────────────────────────────────────────
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

// ── Diagnostics timer setup ───────────────────────────────────────────────────

void VehicleCanNode::setup_diagnostics_timer()
{
  const auto diag_period = std::chrono::duration<double>(1.0 / diagnostics_rate_hz_);
  diagnostics_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(diag_period),
    std::bind(&VehicleCanNode::on_diagnostics_timer, this));
}

// ── CAN frame callback ────────────────────────────────────────────────────────

void VehicleCanNode::on_can_frame(const can_msgs::msg::Frame::SharedPtr msg)
{
  CanFrame frame{};
  frame.id = msg->id;
  frame.dlc = msg->dlc;
  std::copy(msg->data.begin(), msg->data.end(), frame.data.begin());
  process_frame(frame, rclcpp::Time(msg->header.stamp));
}

// ── Process a single CAN frame ────────────────────────────────────────────────

void VehicleCanNode::process_frame(const CanFrame & frame, const rclcpp::Time & stamp)
{
  ++frames_received_;

  const auto decoded = decoder_.decode(frame.id, frame.data, frame.dlc);
  if (!decoded.has_value()) {
    ++frames_unknown_;
    return;
  }

  ++frames_decoded_;

  std::unordered_map<std::string, std::vector<msg::Signal>> frame_signals;
  std::vector<msg::Signal> all_sigs;

  for (const RawSignal & raw_sig : *decoded) {
    // Apply alias: try compound key "CAN{id}_{signal}" first to disambiguate
    // signals that share the same name across different CAN messages.
    const std::string compound_key = "CAN" + std::to_string(frame.id) + "_" + raw_sig.name;
    const std::string & compound_alias = router_.apply_alias(compound_key);
    const std::string & name =
      (compound_alias != compound_key) ? compound_alias : router_.apply_alias(raw_sig.name);

    // Determine domain: schema-based routing takes precedence over CAN-ID-based.
    std::string domain;
    if (!signal_to_domain_.empty()) {
      const auto it = signal_to_domain_.find(name);
      domain =
        (it != signal_to_domain_.end()) ? it->second : std::string(SignalRouter::kUnassignedDomain);
    } else {
      domain = router_.domain_for_id(frame.id);
    }

    TransformResult tr;
    try {
      tr = transformer_.transform(name, raw_sig.value);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Transform error for signal '%s': %s", name.c_str(), e.what());
      ++decode_errors_;
      continue;
    }

    timeout_monitor_.signal_received(name, now_ms());

    msg::Signal sig_msg;
    {
      const auto id_it = signal_name_to_id_.find(name);
      sig_msg.name_id = (id_it != signal_name_to_id_.end()) ? id_it->second : 0;
    }
    sig_msg.value = static_cast<float>(tr.value);

    // Unassigned signals (not in the schema) are discarded.
    if (domain != SignalRouter::kUnassignedDomain) {
      frame_signals[domain].push_back(sig_msg);
      if (publish_all_signals_) {
        all_sigs.push_back(sig_msg);
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

  // Publish one SignalGroup per domain immediately
  for (auto & [domain, signals] : frame_signals) {
    msg::SignalGroup group;
    group.header.stamp = stamp;
    group.header.frame_id = "";
    group.vehicle_id = vehicle_id_;
    group.domain = domain;
    group.signals = std::move(signals);

    if (auto pub_it = domain_pubs_.find(domain); pub_it != domain_pubs_.end()) {
      pub_it->second->publish(group);
    }
  }

  // Firehose: all received signals from this frame
  if (publish_all_signals_ && all_signals_pub_ && !all_sigs.empty()) {
    msg::SignalGroup all_group;
    all_group.header.stamp = stamp;
    all_group.header.frame_id = "";
    all_group.vehicle_id = vehicle_id_;
    all_group.domain = "all";
    all_group.signals = std::move(all_sigs);
    all_signals_pub_->publish(all_group);
  }

  // Check for signal timeouts and log newly-timed-out signals
  for (const auto & name : timeout_monitor_.check_timeouts(now_ms(), signal_timeout_ms_)) {
    RCLCPP_WARN(get_logger(), "Signal timeout: '%s'", name.c_str());
  }
}

// ── Diagnostics timer callback ────────────────────────────────────────────────

void VehicleCanNode::on_diagnostics_timer()
{
  msg::SignalDiagnostic diag;
  diag.header.stamp = now();
  diag.vehicle_id = vehicle_id_;
  diag.can_topic = can_topic_;
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

  schema.unit_table = unit_id_names_;

  schema_pub_->publish(schema);
}

// ── now_ms helper ─────────────────────────────────────────────────────────────

uint64_t VehicleCanNode::now_ms() const
{
  const int64_t ns = now().nanoseconds();
  return (ns >= 0) ? (static_cast<uint64_t>(ns) / 1'000'000ULL) : 0ULL;
}

}  // namespace vehicle_can_decoder
