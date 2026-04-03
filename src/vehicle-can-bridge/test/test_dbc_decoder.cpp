// Copyright 2026 TIER IV, Inc.

#include "vehicle_can_bridge/dbc_decoder.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>

namespace
{
constexpr const char * kTestDbc = TEST_DATA_DIR "/test_vehicle.dbc";
}

namespace vehicle_can_bridge
{

// ── Fixture ───────────────────────────────────────────────────────────────────

class DbcDecoderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(decoder_.load(kTestDbc)) << "Failed to load test DBC: " << kTestDbc;
  }

  DbcDecoder decoder_;
};

// ── Load tests ────────────────────────────────────────────────────────────────

TEST(DbcDecoderLoadTest, LoadValidFile)
{
  DbcDecoder dec;
  EXPECT_FALSE(dec.is_loaded());
  EXPECT_TRUE(dec.load(kTestDbc));
  EXPECT_TRUE(dec.is_loaded());
}

TEST(DbcDecoderLoadTest, LoadNonexistentFile)
{
  DbcDecoder dec;
  EXPECT_FALSE(dec.load("/nonexistent/path/to/file.dbc"));
  EXPECT_FALSE(dec.is_loaded());
}

TEST(DbcDecoderLoadTest, KnownIdsBeforeLoad)
{
  DbcDecoder dec;
  EXPECT_TRUE(dec.known_ids().empty());
}

// ── known_ids tests ───────────────────────────────────────────────────────────

TEST_F(DbcDecoderTest, KnownIdsContainsExpectedMessages)
{
  const auto ids = decoder_.known_ids();
  EXPECT_EQ(ids.size(), 4u);
  EXPECT_TRUE(ids.count(256));  // CHASSIS_RPT
  EXPECT_TRUE(ids.count(257));  // STEERING_RPT
  EXPECT_TRUE(ids.count(512));  // BRAKE_RPT
  EXPECT_TRUE(ids.count(768));  // VEHICLE_STATUS
}

// ── decode: unknown CAN ID ────────────────────────────────────────────────────

TEST_F(DbcDecoderTest, DecodeUnknownIdReturnsNullopt)
{
  const std::array<uint8_t, 8> data{};
  EXPECT_EQ(decoder_.decode(0xDEAD, data, 8), std::nullopt);
}

TEST_F(DbcDecoderTest, DecodeBeforeLoadReturnsNullopt)
{
  DbcDecoder dec;
  const std::array<uint8_t, 8> data{};
  EXPECT_EQ(dec.decode(256, data, 8), std::nullopt);
}

// ── decode: CHASSIS_RPT (ID 256) ─────────────────────────────────────────────
//
// VEHICLE_SPEED: bits 0-15 little-endian unsigned, scale=0.01, offset=0
//   raw = 10000 → physical = 100.00 km/h
//   encode: bytes[0] = 0x10, bytes[1] = 0x27
//
// YAW_RATE: bits 16-31 little-endian unsigned, scale=0.01, offset=-327.68
//   raw = 32768 → physical = 0.00 deg/s  (32768 * 0.01 - 327.68 = 0)
//   encode: bytes[2] = 0x00, bytes[3] = 0x80

TEST_F(DbcDecoderTest, DecodeChassisRpt_VehicleSpeed100)
{
  // raw = 10000 = 0x2710, little-endian: byte0=0x10, byte1=0x27
  // yaw raw = 32768 = 0x8000: byte2=0x00, byte3=0x80
  std::array<uint8_t, 8> data = {0x10, 0x27, 0x00, 0x80, 0, 0, 0, 0};

  const auto result = decoder_.decode(256, data, 8);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 2u);

  // Find VEHICLE_SPEED
  const RawSignal * speed_sig = nullptr;
  const RawSignal * yaw_sig = nullptr;
  for (const auto & s : *result) {
    if (s.name == "VEHICLE_SPEED") {
      speed_sig = &s;
    } else if (s.name == "YAW_RATE") {
      yaw_sig = &s;
    }
  }

  ASSERT_NE(speed_sig, nullptr) << "VEHICLE_SPEED signal not found";
  EXPECT_NEAR(speed_sig->value, 100.0, 0.01);
  EXPECT_EQ(speed_sig->can_id, 256u);

  ASSERT_NE(yaw_sig, nullptr) << "YAW_RATE signal not found";
  EXPECT_NEAR(yaw_sig->value, 0.0, 0.01);
}

// ── decode: STEERING_RPT (ID 257) ────────────────────────────────────────────
//
// STEER_ANGLE: bits 0-15 LE unsigned, scale=0.1, offset=-3276.8
//   raw = 32768 → physical = 32768 * 0.1 - 3276.8 = 0.0 deg
//   encode: byte0=0x00, byte1=0x80

TEST_F(DbcDecoderTest, DecodeSteeringRpt_ZeroAngle)
{
  std::array<uint8_t, 8> data = {0x00, 0x80, 0x80, 0, 0, 0, 0, 0};
  const auto result = decoder_.decode(257, data, 8);
  ASSERT_TRUE(result.has_value());

  const RawSignal * steer_sig = nullptr;
  for (const auto & s : *result) {
    if (s.name == "STEER_ANGLE") {
      steer_sig = &s;
    }
  }
  ASSERT_NE(steer_sig, nullptr);
  EXPECT_NEAR(steer_sig->value, 0.0, 0.1);
}

// ── decode: BRAKE_RPT (ID 512) ───────────────────────────────────────────────
//
// BRAKE_PEDAL: bits 0-7 LE unsigned, scale=0.392157, offset=0
//   raw = 255 → physical ≈ 100.0 %
//
// BRAKE_ACTIVE: bit 8, scale=1, offset=0

TEST_F(DbcDecoderTest, DecodeBrakeRpt_FullPedal)
{
  std::array<uint8_t, 8> data = {0xFF, 0x01, 0, 0, 0, 0, 0, 0};
  const auto result = decoder_.decode(512, data, 8);
  ASSERT_TRUE(result.has_value());

  const RawSignal * pedal = nullptr;
  const RawSignal * active = nullptr;
  for (const auto & s : *result) {
    if (s.name == "BRAKE_PEDAL") {
      pedal = &s;
    }
    if (s.name == "BRAKE_ACTIVE") {
      active = &s;
    }
  }
  ASSERT_NE(pedal, nullptr);
  EXPECT_NEAR(pedal->value, 100.0, 0.5);

  ASSERT_NE(active, nullptr);
  EXPECT_NEAR(active->value, 1.0, 0.01);
}

// ── decode: VEHICLE_STATUS (ID 768) ──────────────────────────────────────────
//
// GEAR: bits 0-3 LE unsigned, raw=4 → physical=4 (Drive)
// TURN_SIGNAL: bits 4-6 LE unsigned, raw=1 → physical=1 (Left)

TEST_F(DbcDecoderTest, DecodeVehicleStatus_DriveWithLeftSignal)
{
  // GEAR=4 (0b0100), TURN_SIGNAL=1 (0b001) → nibble: 0b001_0100 = 0x14
  std::array<uint8_t, 8> data = {0x14, 0, 0, 0, 0, 0, 0, 0};
  const auto result = decoder_.decode(768, data, 8);
  ASSERT_TRUE(result.has_value());

  const RawSignal * gear = nullptr;
  const RawSignal * turn = nullptr;
  for (const auto & s : *result) {
    if (s.name == "GEAR") {
      gear = &s;
    }
    if (s.name == "TURN_SIGNAL") {
      turn = &s;
    }
  }
  ASSERT_NE(gear, nullptr);
  EXPECT_NEAR(gear->value, 4.0, 0.01);

  ASSERT_NE(turn, nullptr);
  EXPECT_NEAR(turn->value, 1.0, 0.01);
}

// ── Move semantics ────────────────────────────────────────────────────────────

TEST(DbcDecoderMoveTest, MoveConstructor)
{
  DbcDecoder dec;
  ASSERT_TRUE(dec.load(kTestDbc));
  EXPECT_TRUE(dec.is_loaded());

  DbcDecoder moved(std::move(dec));
  EXPECT_TRUE(moved.is_loaded());
  // Original is in valid but unspecified state
}

}  // namespace vehicle_can_bridge
