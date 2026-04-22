// Copyright 2026 TIER IV, Inc.

#include "vehicle_can_decoder/can_reader.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <ctime>
#include <utility>

namespace vehicle_can_decoder
{

CanReader::CanReader() = default;

CanReader::~CanReader() { close(); }

// ── Move semantics ────────────────────────────────────────────────────────────

CanReader::CanReader(CanReader && other) noexcept
: fd_(other.fd_), last_errno_(other.last_errno_), interface_name_(std::move(other.interface_name_))
{
  other.fd_ = -1;
  other.last_errno_ = 0;
}

CanReader & CanReader::operator=(CanReader && other) noexcept
{
  if (this != &other) {
    close();
    fd_ = other.fd_;
    last_errno_ = other.last_errno_;
    interface_name_ = std::move(other.interface_name_);
    other.fd_ = -1;
    other.last_errno_ = 0;
  }
  return *this;
}

// ── open ──────────────────────────────────────────────────────────────────────

bool CanReader::open(const std::string & interface_name)
{
  close();
  last_errno_ = 0;
  interface_name_.clear();  // cleared on both success and failure paths

  // Create CAN_RAW socket
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    return false;
  }

  // Bind to the named CAN interface
  struct ifreq ifr
  {
  };
  std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  struct sockaddr_can addr
  {
  };
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Request hardware/software timestamps. Not fatal if unsupported.
  int enable = 1;
  const int ts_result = ::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMP, &enable, sizeof(enable));
  const bool hw_timestamp = (ts_result == 0);
  (void)hw_timestamp;  // used below to decide fallback

  // Set non-blocking
  const int flags = ::fcntl(fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  interface_name_ = interface_name;
  return true;
}

// ── close ─────────────────────────────────────────────────────────────────────

void CanReader::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

// ── read_frame ────────────────────────────────────────────────────────────────

std::optional<CanFrame> CanReader::read_frame()
{
  if (fd_ < 0) {
    return std::nullopt;
  }

  last_errno_ = 0;

  // Use recvmsg to also receive SO_TIMESTAMP ancillary data
  struct can_frame raw_frame
  {
  };
  struct iovec iov
  {
  };
  iov.iov_base = &raw_frame;
  iov.iov_len = sizeof(raw_frame);

  char cmsg_buf[CMSG_SPACE(sizeof(struct timeval))] = {};

  struct msghdr msg
  {
  };
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = cmsg_buf;
  msg.msg_controllen = sizeof(cmsg_buf);

  const ssize_t nbytes = ::recvmsg(fd_, &msg, MSG_DONTWAIT);
  if (nbytes < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // Normal: socket buffer is empty
      return std::nullopt;
    }
    // Real socket error (e.g. ENETDOWN, EBADF, EIO).
    // Record errno so the caller can detect and log it via has_error().
    last_errno_ = errno;
    return std::nullopt;
  }

  if (static_cast<size_t>(nbytes) < sizeof(struct can_frame)) {
    return std::nullopt;
  }

  // Extract hardware timestamp if present; fall back to CLOCK_REALTIME
  double timestamp = 0.0;
  bool got_hw_ts = false;
  for (struct cmsghdr * cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr;
       cmsg = CMSG_NXTHDR(&msg, cmsg)) {
    if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
      struct timeval tv
      {
      };
      std::memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));
      timestamp = static_cast<double>(tv.tv_sec) + static_cast<double>(tv.tv_usec) * 1e-6;
      got_hw_ts = true;
      break;
    }
  }
  if (!got_hw_ts) {
    // Fallback: wall clock time
    struct timespec ts
    {
    };
    ::clock_gettime(CLOCK_REALTIME, &ts);
    timestamp = static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
  }

  CanFrame frame{};
  // Strip extended/error/remote-frame flag bits from the CAN ID
  if (raw_frame.can_id & CAN_EFF_FLAG) {
    frame.id = raw_frame.can_id & CAN_EFF_MASK;
  } else {
    frame.id = raw_frame.can_id & CAN_SFF_MASK;
  }
  frame.dlc = raw_frame.can_dlc;
  frame.timestamp = timestamp;

  const uint8_t valid = (frame.dlc <= 8u) ? frame.dlc : 8u;
  std::memcpy(frame.data.data(), raw_frame.data, valid);
  if (valid < 8u) {
    std::memset(frame.data.data() + valid, 0, 8u - valid);
  }

  return frame;
}

}  // namespace vehicle_can_decoder
