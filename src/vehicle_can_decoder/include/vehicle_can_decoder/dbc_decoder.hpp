// Copyright 2026 TIER IV, Inc.

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace vehicle_can_decoder
{

/// A single signal decoded from a CAN frame (before any transforms).
struct RawSignal
{
  std::string name;  ///< Signal name as defined in the DBC file
  double value;      ///< Physical value (DBC scale/offset already applied)
  uint32_t can_id;   ///< Source CAN message ID
};

/// Wraps dbcppp to load a DBC file and decode raw CAN frames into
/// named signal-value pairs. Thread-safe for concurrent decode() calls
/// after construction (the DBC is immutable once loaded).
class DbcDecoder
{
public:
  DbcDecoder();
  ~DbcDecoder();

  DbcDecoder(const DbcDecoder &) = delete;
  DbcDecoder & operator=(const DbcDecoder &) = delete;
  // Explicit move operations reset loaded_ on the source to prevent
  // null-deref when decode() is called on a moved-from object.
  DbcDecoder(DbcDecoder &&) noexcept;
  DbcDecoder & operator=(DbcDecoder &&) noexcept;

  /// Load a DBC file from the given path.
  /// @returns true on success, false if the file could not be parsed.
  bool load(const std::string & dbc_file_path);

  /// Decode a CAN frame.
  /// @param can_id   29-bit or 11-bit CAN message ID (raw, no flags).
  /// @param data     Frame payload (up to 8 bytes).
  /// @param dlc      Data length code (number of valid bytes in data).
  /// @returns Decoded signals, or empty if the CAN ID is not in the DBC.
  [[nodiscard]] std::optional<std::vector<RawSignal>> decode(
    uint32_t can_id, const std::array<uint8_t, 8> & data, uint8_t dlc) const;

  /// Returns all CAN IDs defined in the loaded DBC.
  /// Useful for pre-filtering: skip decode() for unknown IDs immediately.
  [[nodiscard]] std::unordered_set<uint32_t> known_ids() const;

  /// Returns true if a DBC has been successfully loaded.
  [[nodiscard]] bool is_loaded() const noexcept { return loaded_; }

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  bool loaded_{false};
};

}  // namespace vehicle_can_decoder
