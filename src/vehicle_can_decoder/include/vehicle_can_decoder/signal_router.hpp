// Copyright 2026 TIER IV, Inc.

#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace vehicle_can_decoder
{

/// Domain definition: a named group of CAN IDs.
struct DomainConfig
{
  std::string name;                      ///< Domain name (e.g. "chassis")
  std::string topic;                     ///< ROS2 topic to publish on
  std::unordered_set<uint32_t> can_ids;  ///< CAN IDs belonging to this domain
};

/// Routes decoded signals to their configured domain group, and applies
/// signal name aliases.
///
/// Configuration is loaded once at startup. Routing is O(1) via hash maps.
class SignalRouter
{
public:
  /// Domain name used for signals whose CAN ID does not match any domain.
  static constexpr const char * kUnassignedDomain = "unassigned";

  /// Configure the router.
  /// @param domains   List of domain definitions (from YAML config).
  /// @param aliases   Map of DBC signal name → semantic alias name.
  /// @returns List of warning strings for duplicate CAN IDs across domains.
  ///          Empty on clean configuration. Log these at WARN level.
  [[nodiscard]] std::vector<std::string> configure(
    const std::vector<DomainConfig> & domains,
    const std::unordered_map<std::string, std::string> & aliases);

  /// Return the domain name for the given CAN ID.
  /// Returns kUnassignedDomain if the CAN ID is not in any domain.
  const std::string & domain_for_id(uint32_t can_id) const;

  /// Return the topic string for the given domain name.
  /// Returns an empty string if the domain is not configured (should not happen
  /// in practice since domain_for_id() always returns a valid domain).
  std::string topic_for_domain(const std::string & domain) const;

  /// Apply alias substitution to a signal name.
  /// Returns the alias if one exists, otherwise returns the original name.
  const std::string & apply_alias(const std::string & signal_name) const;

  /// Return the set of all CAN IDs known to the router (across all domains).
  std::unordered_set<uint32_t> all_known_ids() const;

private:
  /// CAN ID → domain name
  std::unordered_map<uint32_t, std::string> id_to_domain_;

  /// Domain name → topic string
  std::unordered_map<std::string, std::string> domain_to_topic_;

  /// DBC signal name → alias
  std::unordered_map<std::string, std::string> aliases_;

  const std::string unassigned_{kUnassignedDomain};
};

}  // namespace vehicle_can_decoder
