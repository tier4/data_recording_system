// Copyright 2026 TIER IV, Inc.

// Unit tests for the schema-based STATUS_INITIAL pre-population logic.
//
// These tests verify the invariant:
//   "Every schema-defined signal is always present in the published SignalGroup,
//    with STATUS_OK when received from the DBC and STATUS_INITIAL otherwise."
//
// The tests operate on the merge helper logic directly (no ROS2 node required)
// and on the SignalRouter compound-alias mechanism.

#include "vehicle_can_decoder/msg/signal.hpp"
#include "vehicle_can_decoder/signal_router.hpp"

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace vehicle_can_decoder
{

// ── Helper: mirrors flush_pending_groups() schema-mode merge logic ─────────────
//
// Given the ordered list of expected signal IDs (name_ids) and a (possibly
// partial) vector of received signals, produce the merged output that the node
// would publish.  Absent signals are filled with STATUS_INITIAL.

static std::vector<msg::Signal> merge_with_schema(
  const std::vector<uint16_t> & expected_ids, const std::vector<msg::Signal> & received)
{
  std::unordered_map<uint16_t, const msg::Signal *> rx;
  for (const auto & s : received) {
    rx[s.name_id] = &s;
  }

  std::vector<msg::Signal> result;
  result.reserve(expected_ids.size());
  for (const uint16_t id : expected_ids) {
    auto it = rx.find(id);
    if (it != rx.end()) {
      result.push_back(*it->second);
    } else {
      msg::Signal init;
      init.name_id = id;
      init.value = 0.0;
      init.status = msg::Signal::STATUS_INITIAL;
      init.timestamp_can = 0.0;
      result.push_back(init);
    }
  }
  return result;
}

// ── Test fixture ──────────────────────────────────────────────────────────────

class SchemaMergeTest : public ::testing::Test
{
protected:
  // IDs for the hypothetical "dynamics" domain used in these tests.
  // Values chosen to be non-contiguous to verify ID-based (not index-based) lookup.
  static constexpr uint16_t kIdSpeedLongitudinal = 1;
  static constexpr uint16_t kIdAngularVelYaw = 7;
  static constexpr uint16_t kIdSteeringAngle = 10;
  static constexpr uint16_t kIdSteeringTorque = 20;  // hypothetical extra signal

  const std::vector<uint16_t> kDynamicsSchemaIds = {
    kIdSpeedLongitudinal,
    kIdAngularVelYaw,
    kIdSteeringAngle,
    kIdSteeringTorque,
  };

  msg::Signal make_signal(uint16_t name_id, double value, uint8_t status = msg::Signal::STATUS_OK)
  {
    msg::Signal s;
    s.name_id = name_id;
    s.value = static_cast<float>(value);
    s.status = status;
    s.timestamp_can = 0.0;
    return s;
  }
};

// ── All signals absent → all STATUS_INITIAL ───────────────────────────────────

TEST_F(SchemaMergeTest, AllAbsentSignalsAreStatusInitial)
{
  const auto result = merge_with_schema(kDynamicsSchemaIds, {});

  ASSERT_EQ(result.size(), kDynamicsSchemaIds.size());
  for (std::size_t i = 0; i < result.size(); ++i) {
    EXPECT_EQ(result[i].name_id, kDynamicsSchemaIds[i]) << "wrong name_id at index " << i;
    EXPECT_EQ(result[i].status, msg::Signal::STATUS_INITIAL) << "name_id=" << result[i].name_id;
    EXPECT_DOUBLE_EQ(result[i].value, 0.0);
  }
}

// ── All signals received → all STATUS_OK ─────────────────────────────────────

TEST_F(SchemaMergeTest, AllReceivedSignalsAreStatusOk)
{
  std::vector<msg::Signal> received;
  received.push_back(make_signal(kIdSpeedLongitudinal, 10.0));
  received.push_back(make_signal(kIdAngularVelYaw, 0.1));
  received.push_back(make_signal(kIdSteeringAngle, -0.3));
  received.push_back(make_signal(kIdSteeringTorque, 2.5));

  const auto result = merge_with_schema(kDynamicsSchemaIds, received);

  ASSERT_EQ(result.size(), kDynamicsSchemaIds.size());
  for (const auto & sig : result) {
    EXPECT_EQ(sig.status, msg::Signal::STATUS_OK) << "name_id=" << sig.name_id;
  }
}

// ── Partial receive → mixed STATUS_OK and STATUS_INITIAL ─────────────────────

TEST_F(SchemaMergeTest, PartialReceiveProducesCorrectStatuses)
{
  // Only speed and yaw are received (steering not available in this DBC)
  std::vector<msg::Signal> received;
  received.push_back(make_signal(kIdSpeedLongitudinal, 15.0));
  received.push_back(make_signal(kIdAngularVelYaw, -0.2));

  const auto result = merge_with_schema(kDynamicsSchemaIds, received);

  ASSERT_EQ(result.size(), kDynamicsSchemaIds.size());

  for (const auto & sig : result) {
    if (sig.name_id == kIdSpeedLongitudinal) {
      EXPECT_EQ(sig.status, msg::Signal::STATUS_OK);
      EXPECT_FLOAT_EQ(sig.value, 15.0f);
    } else if (sig.name_id == kIdAngularVelYaw) {
      EXPECT_EQ(sig.status, msg::Signal::STATUS_OK);
      EXPECT_FLOAT_EQ(sig.value, -0.2f);
    } else {
      EXPECT_EQ(sig.status, msg::Signal::STATUS_INITIAL) << "name_id=" << sig.name_id;
      EXPECT_FLOAT_EQ(sig.value, 0.0f) << "name_id=" << sig.name_id;
    }
  }
}

// ── Output order matches schema definition ────────────────────────────────────

TEST_F(SchemaMergeTest, OutputOrderMatchesSchemaDefinition)
{
  std::vector<msg::Signal> received;
  // Insert in reverse order to verify ordering is driven by schema, not received order
  received.push_back(make_signal(kIdSteeringAngle, 1.0));
  received.push_back(make_signal(kIdSpeedLongitudinal, 2.0));

  const auto result = merge_with_schema(kDynamicsSchemaIds, received);

  ASSERT_EQ(result.size(), kDynamicsSchemaIds.size());
  for (std::size_t i = 0; i < kDynamicsSchemaIds.size(); ++i) {
    EXPECT_EQ(result[i].name_id, kDynamicsSchemaIds[i])
      << "Position " << i << " should have name_id=" << kDynamicsSchemaIds[i]
      << " but got name_id=" << result[i].name_id;
  }
}

// ── Received signal not in schema is ignored (no extra entries) ───────────────

TEST_F(SchemaMergeTest, ExtraSignalsNotInSchemaAreIgnored)
{
  constexpr uint16_t kUnknownId = 99;

  std::vector<msg::Signal> received;
  received.push_back(make_signal(kIdSpeedLongitudinal, 5.0));
  received.push_back(make_signal(kUnknownId, 99.0));  // not in schema → should be dropped

  const auto result = merge_with_schema(kDynamicsSchemaIds, received);

  ASSERT_EQ(result.size(), kDynamicsSchemaIds.size());
  for (const auto & sig : result) {
    EXPECT_NE(sig.name_id, kUnknownId);
  }
}

// ── Compound alias key format verification ────────────────────────────────────
// Tests that the "CAN{id}_{signal}" compound key correctly disambiguates
// signals that share the same DBC name across different CAN messages.

class CompoundAliasTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Simulate PACMod OUTPUT_VALUE appearing in three messages:
    //   ACCEL_RPT  (CAN ID 512): → operation.throttle.report.position
    //   BRAKE_RPT  (CAN ID 516): → operation.brake.report.position
    //   STEERING_RPT (CAN ID 556): → operation.steering.report.angle
    std::vector<DomainConfig> domains;  // empty — schema mode handles routing
    std::unordered_map<std::string, std::string> aliases = {
      {"CAN512_OUTPUT_VALUE", "operation.throttle.report.position"},
      {"CAN516_OUTPUT_VALUE", "operation.brake.report.position"},
      {"CAN556_OUTPUT_VALUE", "operation.steering.report.angle"},
      {"VEHICLE_SPEED", "dynamics.speed.longitudinal"},  // simple (no compound needed)
    };
    router_.configure(domains, aliases);
  }

  SignalRouter router_;

  // Simulates the lookup in VehicleCanNode::process_frame()
  std::string resolve_signal(uint32_t can_id, const std::string & dbc_signal_name) const
  {
    const std::string compound = "CAN" + std::to_string(can_id) + "_" + dbc_signal_name;
    const std::string & compound_result = router_.apply_alias(compound);
    if (compound_result != compound) {
      return compound_result;
    }
    return router_.apply_alias(dbc_signal_name);
  }
};

TEST_F(CompoundAliasTest, OutputValueDisambiguatedByCanId)
{
  EXPECT_EQ(resolve_signal(512, "OUTPUT_VALUE"), "operation.throttle.report.position");
  EXPECT_EQ(resolve_signal(516, "OUTPUT_VALUE"), "operation.brake.report.position");
  EXPECT_EQ(resolve_signal(556, "OUTPUT_VALUE"), "operation.steering.report.angle");
}

TEST_F(CompoundAliasTest, SimpleAliasStillWorksWithoutCompoundKey)
{
  // VEHICLE_SPEED has no compound alias; should fall back to simple alias
  EXPECT_EQ(resolve_signal(1024, "VEHICLE_SPEED"), "dynamics.speed.longitudinal");
}

TEST_F(CompoundAliasTest, UnknownSignalReturnsOriginalName)
{
  // No alias configured → returns the raw DBC signal name unchanged
  EXPECT_EQ(resolve_signal(256, "SOME_UNKNOWN_SIGNAL"), "SOME_UNKNOWN_SIGNAL");
}

TEST_F(CompoundAliasTest, SameNameDifferentIdWithOneAliasOnly)
{
  // Only CAN512_OUTPUT_VALUE is aliased; other IDs with OUTPUT_VALUE
  // should fall back to simple lookup (no alias → return original name)
  EXPECT_EQ(resolve_signal(999, "OUTPUT_VALUE"), "OUTPUT_VALUE");
}

// ── STATUS_INITIAL has correct constant value ──────────────────────────────────

TEST(SignalStatusTest, StatusInitialConstantValue)
{
  EXPECT_EQ(msg::Signal::STATUS_OK, 0);
  EXPECT_EQ(msg::Signal::STATUS_TIMEOUT, 1);
  EXPECT_EQ(msg::Signal::STATUS_ERROR, 2);
  EXPECT_EQ(msg::Signal::STATUS_INITIAL, 3);
}

}  // namespace vehicle_can_decoder
