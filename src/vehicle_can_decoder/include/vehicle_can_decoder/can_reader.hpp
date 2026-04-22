// Copyright 2026 TIER IV, Inc.

#pragma once

#include <array>
#include <cerrno>
#include <cstdint>
#include <optional>
#include <string>

namespace vehicle_can_decoder
{

/// A raw CAN frame read from the SocketCAN socket.
struct CanFrame
{
  uint32_t id;                  ///< CAN message ID (11-bit or 29-bit, no flags)
  std::array<uint8_t, 8> data;  ///< Payload (bytes beyond dlc are zeroed)
  uint8_t dlc;                  ///< Data length code (0-8)
  double timestamp;             ///< Hardware timestamp (seconds since epoch), or
                                ///  wall clock time if hardware TS is unavailable
};

/// Opens a SocketCAN interface and reads raw CAN frames non-blockingly.
///
/// Usage:
///   CanReader reader;
///   if (!reader.open("can0")) { /* handle error */ }
///   while (true) {
///     while (auto frame = reader.read_frame()) { /* process */ }
///     if (reader.has_error()) { /* handle socket error */ }
///   }
class CanReader
{
public:
  CanReader();
  ~CanReader();

  CanReader(const CanReader &) = delete;
  CanReader & operator=(const CanReader &) = delete;

  /// Move constructor: transfers ownership of the socket fd.
  CanReader(CanReader && other) noexcept;
  CanReader & operator=(CanReader && other) noexcept;

  /// Open the named SocketCAN interface (e.g. "can0", "vcan0").
  /// The socket is set to non-blocking mode.
  /// @returns true on success.
  bool open(const std::string & interface_name);

  /// Close the socket. Safe to call multiple times or when not open.
  void close();

  /// Read one CAN frame without blocking.
  /// Returns std::nullopt when the socket buffer is empty (normal) OR when a
  /// real socket error occurs. Check has_error() / last_error() to distinguish
  /// the two cases after read_frame() returns nullopt.
  [[nodiscard]] std::optional<CanFrame> read_frame();

  [[nodiscard]] bool is_open() const noexcept { return fd_ >= 0; }

  /// True if the last read_frame() call encountered a real socket error
  /// (i.e. not just an empty buffer).
  [[nodiscard]] bool has_error() const noexcept { return last_errno_ != 0; }

  /// The errno value from the last socket error, or 0 if no error.
  [[nodiscard]] int last_error() const noexcept { return last_errno_; }

  /// Clear the error flag. Call after handling the error.
  void clear_error() noexcept { last_errno_ = 0; }

  const std::string & interface_name() const noexcept { return interface_name_; }

private:
  int fd_{-1};
  int last_errno_{0};
  std::string interface_name_;
};

}  // namespace vehicle_can_decoder
