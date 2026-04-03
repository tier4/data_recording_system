// Copyright 2026 TIER IV, Inc.

#include "vehicle_can_bridge/timeout_monitor.hpp"

namespace vehicle_can_bridge
{

void TimeoutMonitor::register_signal(const std::string & signal_name)
{
  // Only insert if not already present; preserves existing state on re-configure.
  states_.emplace(signal_name, SignalState{});
}

void TimeoutMonitor::signal_received(const std::string & signal_name, uint64_t now_ms)
{
  auto it = states_.find(signal_name);
  if (it == states_.end()) {
    // Auto-register on first receipt for convenience
    auto [ins_it, _] = states_.emplace(signal_name, SignalState{});
    it = ins_it;
  }
  it->second.last_seen_ms = now_ms;
  it->second.status = STATUS_OK;
}

uint8_t TimeoutMonitor::get_status(const std::string & signal_name) const
{
  const auto it = states_.find(signal_name);
  if (it == states_.end()) {
    return STATUS_INITIAL;
  }
  return it->second.status;
}

std::vector<std::string> TimeoutMonitor::check_timeouts(uint64_t now_ms, uint64_t timeout_ms)
{
  std::vector<std::string> newly_timed_out;

  for (auto & [name, state] : states_) {
    if (state.status == STATUS_INITIAL) {
      // Signal has never been received; do not flag as timed-out yet.
      continue;
    }
    if (state.status == STATUS_OK && (now_ms - state.last_seen_ms) > timeout_ms) {
      state.status = STATUS_TIMEOUT;
      newly_timed_out.push_back(name);
    }
  }

  return newly_timed_out;
}

std::vector<std::string> TimeoutMonitor::timed_out_signals() const
{
  std::vector<std::string> result;
  for (const auto & [name, state] : states_) {
    if (state.status == STATUS_TIMEOUT) {
      result.push_back(name);
    }
  }
  return result;
}

}  // namespace vehicle_can_bridge
