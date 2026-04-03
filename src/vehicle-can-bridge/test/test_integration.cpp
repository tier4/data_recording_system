// Copyright 2026 TIER IV, Inc.

#include "vehicle_can_bridge/dbc_decoder.hpp"
#include "vehicle_can_bridge/signal_router.hpp"
#include "vehicle_can_bridge/signal_transformer.hpp"
#include "vehicle_can_bridge/timeout_monitor.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace
{
constexpr const char * kTestDbc = TEST_DATA_DIR "/test_vehicle.dbc";
}

namespace vehicle_can_bridge
{

/// Integration test: exercises the full decode → transform → route pipeline
/// without a real ROS2 node or CAN hardware.
class PipelineIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Load DBC
    ASSERT_TRUE(decoder_.load(kTestDbc));

    // Configure router matching test DBC CAN IDs
    std::vector<DomainConfig> domains = {
      {"chassis", "/vehicle/chassis", {256, 257}},  // CHASSIS_RPT, STEERING_RPT
      {"brake", "/vehicle/brake", {512}},           // BRAKE_RPT
      {"status", "/vehicle/status", {768}},         // VEHICLE_STATUS
    };
    std::unordered_map<std::string, std::string> aliases = {
      {"VEHICLE_SPEED", "vehicle_speed"},
      {"STEER_ANGLE", "steering_angle"},
      {"BRAKE_PEDAL", "brake_pedal"},
    };
    const auto warnings = router_.configure(domains, aliases);
    EXPECT_TRUE(warnings.empty());

    // Configure transforms
    transformer_.configure({
      {"vehicle_speed", {"x / 3.6", "m/s"}},
      {"steering_angle", {"x * pi / 180.0", "rad"}},
      {"brake_pedal", {"x / 100.0", ""}},
    });
  }

  DbcDecoder decoder_;
  SignalRouter router_;
  SignalTransformer transformer_;
  TimeoutMonitor monitor_;

  struct ProcessedSignal
  {
    std::string name;
    double value;
    std::string unit;
    std::string domain;
  };

  /// Run one "frame" through the full pipeline.
  std::vector<ProcessedSignal> process(
    uint32_t can_id, const std::array<uint8_t, 8> & data, uint8_t dlc)
  {
    std::vector<ProcessedSignal> out;
    const auto decoded = decoder_.decode(can_id, data, dlc);
    if (!decoded.has_value()) {
      return out;
    }
    const std::string & domain = router_.domain_for_id(can_id);
    for (const auto & raw : *decoded) {
      const std::string & name = router_.apply_alias(raw.name);
      const TransformResult tr = transformer_.transform(name, raw.value);
      monitor_.signal_received(name, 0);
      out.push_back({name, tr.value, tr.unit, domain});
    }
    return out;
  }
};

// ── Vehicle speed decode → transform → route ─────────────────────────────────

TEST_F(PipelineIntegrationTest, VehicleSpeedEndToEnd)
{
  // CHASSIS_RPT: VEHICLE_SPEED raw=10000 → 100.0 km/h → 27.78 m/s
  // YAW_RATE raw=32768 → 0.0 deg/s (no transform configured, passthrough)
  std::array<uint8_t, 8> data = {0x10, 0x27, 0x00, 0x80, 0, 0, 0, 0};

  const auto signals = process(256, data, 8);
  ASSERT_FALSE(signals.empty());

  const ProcessedSignal * spd = nullptr;
  for (const auto & s : signals) {
    if (s.name == "vehicle_speed") {
      spd = &s;
    }
  }
  ASSERT_NE(spd, nullptr) << "vehicle_speed not found";
  EXPECT_NEAR(spd->value, 100.0 / 3.6, 0.01);
  EXPECT_EQ(spd->unit, "m/s");
  EXPECT_EQ(spd->domain, "chassis");
}

// ── Steering angle decode → transform → route ────────────────────────────────

TEST_F(PipelineIntegrationTest, SteeringAngleEndToEnd)
{
  // STEERING_RPT: STEER_ANGLE raw=32768 → 0.0 deg → 0.0 rad
  std::array<uint8_t, 8> data = {0x00, 0x80, 0x80, 0, 0, 0, 0, 0};

  const auto signals = process(257, data, 8);
  const ProcessedSignal * steer = nullptr;
  for (const auto & s : signals) {
    if (s.name == "steering_angle") {
      steer = &s;
    }
  }
  ASSERT_NE(steer, nullptr);
  EXPECT_NEAR(steer->value, 0.0, 0.001);
  EXPECT_EQ(steer->unit, "rad");
  EXPECT_EQ(steer->domain, "chassis");
}

// ── Brake pedal decode → transform → route ───────────────────────────────────

TEST_F(PipelineIntegrationTest, BrakePedalFullPressEndToEnd)
{
  // BRAKE_RPT: BRAKE_PEDAL raw=255 → ~100% → ~1.0
  std::array<uint8_t, 8> data = {0xFF, 0x01, 0, 0, 0, 0, 0, 0};

  const auto signals = process(512, data, 8);
  const ProcessedSignal * pedal = nullptr;
  for (const auto & s : signals) {
    if (s.name == "brake_pedal") {
      pedal = &s;
    }
  }
  ASSERT_NE(pedal, nullptr);
  EXPECT_NEAR(pedal->value, 1.0, 0.01);
  EXPECT_EQ(pedal->domain, "brake");
}

// ── Unknown CAN ID → empty result ────────────────────────────────────────────

TEST_F(PipelineIntegrationTest, UnknownCanIdProducesNoSignals)
{
  std::array<uint8_t, 8> data{};
  const auto signals = process(0xDEAD, data, 8);
  EXPECT_TRUE(signals.empty());
}

// ── Domain routing for all test DBC messages ─────────────────────────────────

TEST_F(PipelineIntegrationTest, AllMessagesRoutedToCorrectDomain)
{
  std::array<uint8_t, 8> data{};
  EXPECT_EQ(router_.domain_for_id(256), "chassis");
  EXPECT_EQ(router_.domain_for_id(257), "chassis");
  EXPECT_EQ(router_.domain_for_id(512), "brake");
  EXPECT_EQ(router_.domain_for_id(768), "status");
  EXPECT_EQ(router_.domain_for_id(999), std::string(SignalRouter::kUnassignedDomain));
}

// ── Timeout integration ───────────────────────────────────────────────────────

TEST_F(PipelineIntegrationTest, TimeoutStatusAfterMissingFrames)
{
  // Receive speed at t=0
  monitor_.signal_received("vehicle_speed", 0);
  EXPECT_EQ(monitor_.get_status("vehicle_speed"), TimeoutMonitor::STATUS_OK);

  // 600 ms later, timeout=500 ms → should timeout
  monitor_.check_timeouts(600, 500);
  EXPECT_EQ(monitor_.get_status("vehicle_speed"), TimeoutMonitor::STATUS_TIMEOUT);

  // Receive again → back to OK
  monitor_.signal_received("vehicle_speed", 601);
  EXPECT_EQ(monitor_.get_status("vehicle_speed"), TimeoutMonitor::STATUS_OK);
}

// ── Alias applied in pipeline ─────────────────────────────────────────────────

TEST_F(PipelineIntegrationTest, AliasAppliedBeforeTransform)
{
  // VEHICLE_SPEED DBC name → aliased to "vehicle_speed" → transform applied
  std::array<uint8_t, 8> data = {0x10, 0x27, 0x00, 0x80, 0, 0, 0, 0};
  const auto signals = process(256, data, 8);

  // Must appear under aliased name, not DBC name
  bool found_alias = false;
  bool found_dbc_name = false;
  for (const auto & s : signals) {
    if (s.name == "vehicle_speed") {
      found_alias = true;
    }
    if (s.name == "VEHICLE_SPEED") {
      found_dbc_name = true;
    }
  }
  EXPECT_TRUE(found_alias);
  EXPECT_FALSE(found_dbc_name);
}

}  // namespace vehicle_can_bridge
