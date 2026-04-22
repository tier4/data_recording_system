// Copyright 2026 TIER IV, Inc.

// Unit tests for schema-related helpers used by VehicleCanNode.
//
// - CompoundAliasTest: verifies "CAN{id}_{signal}" compound key disambiguation
// - SignalStatusTest:  verifies STATUS_* constant values in Signal.msg

#include "vehicle_can_decoder/signal_router.hpp"

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace vehicle_can_decoder
{

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

}  // namespace vehicle_can_decoder
