// Copyright 2026 TIER IV, Inc.

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace vehicle_can_decoder
{

/// Tracks whether CAN signals are arriving within an expected interval.
///
/// Each signal starts in STATUS_INITIAL. On first receipt it transitions to
/// STATUS_OK. If not refreshed within timeout_ms it transitions to
/// STATUS_TIMEOUT. The status returns to STATUS_OK on the next receipt.
class TimeoutMonitor
{
public:
  /// Signal status values (match Signal.msg constants).
  static constexpr uint8_t STATUS_OK = 0;
  static constexpr uint8_t STATUS_TIMEOUT = 1;
  static constexpr uint8_t STATUS_ERROR = 2;
  static constexpr uint8_t STATUS_INITIAL = 3;

  /// Register a signal to be monitored.
  /// Must be called before signal_received() or get_status().
  void register_signal(const std::string & signal_name);

  /// Record that a signal was just received (or updated).
  /// @param signal_name  Name of the signal.
  /// @param now_ms       Current time in milliseconds (monotonic).
  void signal_received(const std::string & signal_name, uint64_t now_ms);

  /// Get the current status of a signal.
  /// Returns STATUS_INITIAL if the signal has never been received or
  /// is not registered.
  [[nodiscard]] uint8_t get_status(const std::string & signal_name) const;

  /// Check all registered signals for timeout.
  /// Transitions STATUS_OK → STATUS_TIMEOUT for signals not seen within
  /// timeout_ms. Returns list of newly-timed-out signal names.
  /// @param now_ms       Current time in milliseconds.
  /// @param timeout_ms   Timeout threshold in milliseconds.
  [[nodiscard]] std::vector<std::string> check_timeouts(uint64_t now_ms, uint64_t timeout_ms);

  /// Returns the names of all signals currently in STATUS_TIMEOUT.
  [[nodiscard]] std::vector<std::string> timed_out_signals() const;

private:
  struct SignalState
  {
    uint64_t last_seen_ms{0};
    uint8_t status{STATUS_INITIAL};
  };

  std::unordered_map<std::string, SignalState> states_;
};

}  // namespace vehicle_can_decoder
