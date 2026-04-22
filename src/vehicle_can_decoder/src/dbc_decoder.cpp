// Copyright 2026 TIER IV, Inc.

#include "vehicle_can_decoder/dbc_decoder.hpp"

#include <dbcppp/Network.h>

#include <fstream>
#include <stdexcept>
#include <unordered_map>

namespace vehicle_can_decoder
{

// ── Impl ──────────────────────────────────────────────────────────────────────

struct DbcDecoder::Impl
{
  std::unique_ptr<dbcppp::INetwork> network;

  // Pre-built lookup: CAN ID → IMessage pointer (owned by network)
  std::unordered_map<uint64_t, const dbcppp::IMessage *> msg_map;

  void build_map()
  {
    msg_map.clear();
    if (!network) {
      return;
    }
    for (const dbcppp::IMessage & msg : network->Messages()) {
      msg_map[msg.Id()] = &msg;
    }
  }
};

// ── Constructors / destructor ─────────────────────────────────────────────────

DbcDecoder::DbcDecoder() : impl_(std::make_unique<Impl>()) {}

DbcDecoder::~DbcDecoder() = default;

DbcDecoder::DbcDecoder(DbcDecoder && other) noexcept
: impl_(std::move(other.impl_)), loaded_(other.loaded_)
{
  // Reset source so that calling decode() on a moved-from DbcDecoder does not
  // pass the loaded_ guard and then dereference a null impl_.
  other.loaded_ = false;
}

DbcDecoder & DbcDecoder::operator=(DbcDecoder && other) noexcept
{
  if (this != &other) {
    impl_ = std::move(other.impl_);
    loaded_ = other.loaded_;
    other.loaded_ = false;
  }
  return *this;
}

// ── load ──────────────────────────────────────────────────────────────────────

bool DbcDecoder::load(const std::string & dbc_file_path)
{
  loaded_ = false;

  std::ifstream file(dbc_file_path);
  if (!file.is_open()) {
    return false;
  }

  try {
    impl_->network = dbcppp::INetwork::LoadDBCFromIs(file);
  } catch (const std::exception &) {
    return false;
  }

  if (!impl_->network) {
    return false;
  }

  impl_->build_map();
  loaded_ = true;
  return true;
}

// ── decode ────────────────────────────────────────────────────────────────────

std::optional<std::vector<RawSignal>> DbcDecoder::decode(
  uint32_t can_id, const std::array<uint8_t, 8> & data, uint8_t dlc) const
{
  if (!loaded_) {
    return std::nullopt;
  }

  auto it = impl_->msg_map.find(static_cast<uint64_t>(can_id));
  if (it == impl_->msg_map.end()) {
    return std::nullopt;
  }

  const dbcppp::IMessage * msg = it->second;

  std::vector<RawSignal> results;
  results.reserve(8);

  // dbcppp decode: raw bytes → physical value per signal
  for (const dbcppp::ISignal & sig : msg->Signals()) {
    // dbcppp expects the full 8-byte payload regardless of DLC.
    // Bytes beyond DLC are zeroed in the array (guaranteed by caller).
    const uint64_t raw = sig.Decode(data.data());
    const double physical = sig.RawToPhys(raw);

    results.push_back(RawSignal{
      .name = std::string(sig.Name()),
      .value = physical,
      .can_id = can_id,
    });
  }

  return results;
}

// ── known_ids ─────────────────────────────────────────────────────────────────

std::unordered_set<uint32_t> DbcDecoder::known_ids() const
{
  std::unordered_set<uint32_t> ids;
  if (!loaded_) {
    return ids;
  }
  ids.reserve(impl_->msg_map.size());
  for (const auto & kv : impl_->msg_map) {
    ids.insert(static_cast<uint32_t>(kv.first));
  }
  return ids;
}

}  // namespace vehicle_can_decoder
