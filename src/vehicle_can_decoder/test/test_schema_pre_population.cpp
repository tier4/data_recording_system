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
// Given the ordered list of expected canonical signal names and a (possibly
// partial) vector of received signals, produce the merged output that the node
// would publish.  Absent signals are filled with STATUS_INITIAL.

static std::vector<msg::Signal> merge_with_schema(
  const std::vector<std::string> & expected, const std::vector<msg::Signal> & received)
{
  std::unordered_map<std::string, const msg::Signal *> rx;
  for (const auto & s : received) {
    rx[s.name] = &s;
  }

  std::vector<msg::Signal> result;
  result.reserve(expected.size());
  for (const auto & name : expected) {
    auto it = rx.find(name);
    if (it != rx.end()) {
      result.push_back(*it->second);
    } else {
      msg::Signal init;
      init.name = name;
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
  // Canonical signal list for a hypothetical "dynamics" domain
  const std::vector<std::string> kDynamicsSchema = {
    "dynamics.speed.longitudinal",
    "dynamics.angular_vel.yaw",
    "dynamics.steering.angle",
    "dynamics.steering.torque",
  };

  msg::Signal make_signal(
    const std::string & name, double value, uint8_t status = msg::Signal::STATUS_OK)
  {
    msg::Signal s;
    s.name = name;
    s.value = value;
    s.status = status;
    s.timestamp_can = 0.0;
    return s;
  }
};

// ── All signals absent → all STATUS_INITIAL ───────────────────────────────────

TEST_F(SchemaMergeTest, AllAbsentSignalsAreStatusInitial)
{
  const auto result = merge_with_schema(kDynamicsSchema, {});

  ASSERT_EQ(result.size(), kDynamicsSchema.size());
  for (std::size_t i = 0; i < result.size(); ++i) {
    EXPECT_EQ(result[i].name, kDynamicsSchema[i]) << "wrong name at index " << i;
    EXPECT_EQ(result[i].status, msg::Signal::STATUS_INITIAL) << "signal: " << result[i].name;
    EXPECT_DOUBLE_EQ(result[i].value, 0.0);
  }
}

// ── All signals received → all STATUS_OK ─────────────────────────────────────

TEST_F(SchemaMergeTest, AllReceivedSignalsAreStatusOk)
{
  std::vector<msg::Signal> received;
  received.push_back(make_signal("dynamics.speed.longitudinal", 10.0));
  received.push_back(make_signal("dynamics.angular_vel.yaw", 0.1));
  received.push_back(make_signal("dynamics.steering.angle", -0.3));
  received.push_back(make_signal("dynamics.steering.torque", 2.5));

  const auto result = merge_with_schema(kDynamicsSchema, received);

  ASSERT_EQ(result.size(), kDynamicsSchema.size());
  for (const auto & sig : result) {
    EXPECT_EQ(sig.status, msg::Signal::STATUS_OK) << "signal: " << sig.name;
  }
}

// ── Partial receive → mixed STATUS_OK and STATUS_INITIAL ─────────────────────

TEST_F(SchemaMergeTest, PartialReceiveProducesCorrectStatuses)
{
  // Only speed and yaw are received (steering not available in this DBC)
  std::vector<msg::Signal> received;
  received.push_back(make_signal("dynamics.speed.longitudinal", 15.0));
  received.push_back(make_signal("dynamics.angular_vel.yaw", -0.2));

  const auto result = merge_with_schema(kDynamicsSchema, received);

  ASSERT_EQ(result.size(), kDynamicsSchema.size());

  for (const auto & sig : result) {
    if (sig.name == "dynamics.speed.longitudinal") {
      EXPECT_EQ(sig.status, msg::Signal::STATUS_OK);
      EXPECT_DOUBLE_EQ(sig.value, 15.0);
    } else if (sig.name == "dynamics.angular_vel.yaw") {
      EXPECT_EQ(sig.status, msg::Signal::STATUS_OK);
      EXPECT_DOUBLE_EQ(sig.value, -0.2);
    } else {
      EXPECT_EQ(sig.status, msg::Signal::STATUS_INITIAL) << "signal: " << sig.name;
      EXPECT_DOUBLE_EQ(sig.value, 0.0) << "signal: " << sig.name;
    }
  }
}

// ── Output order matches schema definition ────────────────────────────────────

TEST_F(SchemaMergeTest, OutputOrderMatchesSchemaDefinition)
{
  std::vector<msg::Signal> received;
  // Insert in reverse order to verify ordering is driven by schema, not received order
  received.push_back(make_signal("dynamics.steering.angle", 1.0));
  received.push_back(make_signal("dynamics.speed.longitudinal", 2.0));

  const auto result = merge_with_schema(kDynamicsSchema, received);

  ASSERT_EQ(result.size(), kDynamicsSchema.size());
  for (std::size_t i = 0; i < kDynamicsSchema.size(); ++i) {
    EXPECT_EQ(result[i].name, kDynamicsSchema[i])
      << "Position " << i << " should be '" << kDynamicsSchema[i] << "' but got '" << result[i].name
      << "'";
  }
}

// ── Received signal not in schema is ignored (no extra entries) ───────────────

TEST_F(SchemaMergeTest, ExtraSignalsNotInSchemaAreIgnored)
{
  std::vector<msg::Signal> received;
  received.push_back(make_signal("dynamics.speed.longitudinal", 5.0));
  received.push_back(make_signal("unknown.signal.not_in_schema", 99.0));  // should be dropped

  const auto result = merge_with_schema(kDynamicsSchema, received);

  ASSERT_EQ(result.size(), kDynamicsSchema.size());
  for (const auto & sig : result) {
    EXPECT_NE(sig.name, "unknown.signal.not_in_schema");
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
