// Copyright 2026 TIER IV, Inc.

#include "vehicle_can_bridge/signal_router.hpp"

#include <gtest/gtest.h>

namespace vehicle_can_bridge
{

// ── Fixture ───────────────────────────────────────────────────────────────────

class SignalRouterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::vector<DomainConfig> domains = {
      {"chassis", "/vehicle/chassis", {0x100, 0x101, 0x102}},
      {"powertrain", "/vehicle/powertrain", {0x200, 0x201}},
      {"body", "/vehicle/body", {0x300}},
    };

    std::unordered_map<std::string, std::string> aliases = {
      {"StrAng_Sns", "steering_angle"},
      {"WhlSpd_FL", "wheel_speed_front_left"},
      {"VEHICLE_SPEED", "vehicle_speed"},
    };

    const auto warnings = router_.configure(domains, aliases);
    EXPECT_TRUE(warnings.empty()) << "Unexpected configuration warnings";
  }

  SignalRouter router_;
};

// ── domain_for_id ─────────────────────────────────────────────────────────────

TEST_F(SignalRouterTest, KnownIdMappedToDomain)
{
  EXPECT_EQ(router_.domain_for_id(0x100), "chassis");
  EXPECT_EQ(router_.domain_for_id(0x101), "chassis");
  EXPECT_EQ(router_.domain_for_id(0x200), "powertrain");
  EXPECT_EQ(router_.domain_for_id(0x300), "body");
}

TEST_F(SignalRouterTest, UnknownIdReturnsUnassigned)
{
  EXPECT_EQ(router_.domain_for_id(0x999), std::string(SignalRouter::kUnassignedDomain));
  EXPECT_EQ(router_.domain_for_id(0x000), std::string(SignalRouter::kUnassignedDomain));
}

// ── topic_for_domain ──────────────────────────────────────────────────────────

TEST_F(SignalRouterTest, TopicForKnownDomain)
{
  EXPECT_EQ(router_.topic_for_domain("chassis"), "/vehicle/chassis");
  EXPECT_EQ(router_.topic_for_domain("powertrain"), "/vehicle/powertrain");
  EXPECT_EQ(router_.topic_for_domain("body"), "/vehicle/body");
}

TEST_F(SignalRouterTest, TopicForUnknownDomainEmpty)
{
  EXPECT_EQ(router_.topic_for_domain("nonexistent"), "");
}

// ── apply_alias ───────────────────────────────────────────────────────────────

TEST_F(SignalRouterTest, AliasSubstituted)
{
  EXPECT_EQ(router_.apply_alias("StrAng_Sns"), "steering_angle");
  EXPECT_EQ(router_.apply_alias("WhlSpd_FL"), "wheel_speed_front_left");
  EXPECT_EQ(router_.apply_alias("VEHICLE_SPEED"), "vehicle_speed");
}

TEST_F(SignalRouterTest, NoAliasReturnsOriginalName)
{
  EXPECT_EQ(router_.apply_alias("UNKNOWN_SIGNAL"), "UNKNOWN_SIGNAL");
  EXPECT_EQ(router_.apply_alias("BRAKE_PEDAL"), "BRAKE_PEDAL");
}

// ── all_known_ids ─────────────────────────────────────────────────────────────

TEST_F(SignalRouterTest, AllKnownIdsContainsAllConfiguredIds)
{
  const auto ids = router_.all_known_ids();
  EXPECT_EQ(ids.size(), 6u);  // 3 + 2 + 1
  EXPECT_TRUE(ids.count(0x100));
  EXPECT_TRUE(ids.count(0x201));
  EXPECT_TRUE(ids.count(0x300));
  EXPECT_FALSE(ids.count(0x999));
}

// ── Reconfigure ───────────────────────────────────────────────────────────────

TEST_F(SignalRouterTest, ReconfigureUpdatesMapping)
{
  std::vector<DomainConfig> new_domains = {
    {"adas", "/vehicle/adas", {0xABC}},
  };
  EXPECT_TRUE(router_.configure(new_domains, {}).empty());

  EXPECT_EQ(router_.domain_for_id(0xABC), "adas");
  // Old IDs no longer in the mapping
  EXPECT_EQ(router_.domain_for_id(0x100), std::string(SignalRouter::kUnassignedDomain));
}

// ── Empty config ──────────────────────────────────────────────────────────────

TEST(SignalRouterEmptyTest, EmptyConfigAllUnassigned)
{
  SignalRouter r;
  EXPECT_TRUE(r.configure({}, {}).empty());
  EXPECT_EQ(r.domain_for_id(0x100), std::string(SignalRouter::kUnassignedDomain));
  EXPECT_TRUE(r.all_known_ids().empty());
}

// ── Duplicate CAN ID warning ──────────────────────────────────────────────────

TEST(SignalRouterDuplicateTest, DuplicateCanIdProducesWarning)
{
  SignalRouter r;
  std::vector<DomainConfig> domains = {
    {"chassis", "/vehicle/chassis", {0x100, 0x101}},
    {"adas", "/vehicle/adas", {0x100}},  // duplicate!
  };
  const auto warnings = r.configure(domains, {});
  EXPECT_EQ(warnings.size(), 1u);
  // After duplicate, the second domain wins
  EXPECT_EQ(r.domain_for_id(0x100), "adas");
}

TEST(SignalRouterDuplicateTest, NoDuplicatesProducesNoWarning)
{
  SignalRouter r;
  std::vector<DomainConfig> domains = {
    {"chassis", "/vehicle/chassis", {0x100}},
    {"brake", "/vehicle/brake", {0x200}},
  };
  EXPECT_TRUE(r.configure(domains, {}).empty());
}

}  // namespace vehicle_can_bridge
