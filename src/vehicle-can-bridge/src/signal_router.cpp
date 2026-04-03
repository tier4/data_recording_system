// Copyright 2026 TIER IV, Inc.

#include "vehicle_can_bridge/signal_router.hpp"

#include <cstdio>
#include <string>

namespace vehicle_can_bridge
{

std::vector<std::string> SignalRouter::configure(
  const std::vector<DomainConfig> & domains,
  const std::unordered_map<std::string, std::string> & aliases)
{
  id_to_domain_.clear();
  domain_to_topic_.clear();
  aliases_ = aliases;

  std::vector<std::string> warnings;

  for (const DomainConfig & domain : domains) {
    domain_to_topic_[domain.name] = domain.topic;
    for (uint32_t id : domain.can_ids) {
      auto [it, inserted] = id_to_domain_.emplace(id, domain.name);
      if (!inserted) {
        // CAN ID already assigned to another domain; last one wins.
        warnings.push_back(
          "Duplicate CAN ID 0x" +
          [](uint32_t v) {
            char buf[12];
            std::snprintf(buf, sizeof(buf), "%08X", v);
            return std::string(buf);
          }(id) +
          " in domain '" + domain.name + "' (already assigned to '" + it->second +
          "'). "
          "Reassigning to '" +
          domain.name + "'.");
        it->second = domain.name;
      }
    }
  }

  return warnings;
}

const std::string & SignalRouter::domain_for_id(uint32_t can_id) const
{
  const auto it = id_to_domain_.find(can_id);
  if (it == id_to_domain_.end()) {
    return unassigned_;
  }
  return it->second;
}

std::string SignalRouter::topic_for_domain(const std::string & domain) const
{
  const auto it = domain_to_topic_.find(domain);
  if (it == domain_to_topic_.end()) {
    return "";
  }
  return it->second;
}

const std::string & SignalRouter::apply_alias(const std::string & signal_name) const
{
  const auto it = aliases_.find(signal_name);
  if (it == aliases_.end()) {
    return signal_name;
  }
  return it->second;
}

std::unordered_set<uint32_t> SignalRouter::all_known_ids() const
{
  std::unordered_set<uint32_t> ids;
  ids.reserve(id_to_domain_.size());
  for (const auto & kv : id_to_domain_) {
    ids.insert(kv.first);
  }
  return ids;
}

}  // namespace vehicle_can_bridge
