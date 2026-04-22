// Copyright 2026 TIER IV, Inc.

#include "vehicle_can_decoder/signal_transformer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace vehicle_can_decoder
{

// ── No-transform passthrough ──────────────────────────────────────────────────

TEST(SignalTransformerTest, EmptyConfigPassthrough)
{
  SignalTransformer t;
  t.configure({});
  const auto r = t.transform("any_signal", 42.0);
  EXPECT_DOUBLE_EQ(r.value, 42.0);
  EXPECT_DOUBLE_EQ(r.raw_value, 42.0);
  EXPECT_TRUE(r.unit.empty());
}

TEST(SignalTransformerTest, UnknownSignalPassthrough)
{
  SignalTransformer t;
  t.configure({{"known", {"x * 2.0", "unit"}}});
  const auto r = t.transform("unknown", 5.0);
  EXPECT_DOUBLE_EQ(r.value, 5.0);
  EXPECT_DOUBLE_EQ(r.raw_value, 5.0);
}

// ── Linear scaling ────────────────────────────────────────────────────────────

TEST(SignalTransformerTest, KmhToMs)
{
  SignalTransformer t;
  t.configure({{"VehicleSpeed", {"x / 3.6", "m/s"}}});

  const auto r = t.transform("VehicleSpeed", 36.0);
  EXPECT_NEAR(r.value, 10.0, 1e-9);
  EXPECT_DOUBLE_EQ(r.raw_value, 36.0);
  EXPECT_EQ(r.unit, "m/s");
}

TEST(SignalTransformerTest, DegToRad)
{
  SignalTransformer t;
  t.configure({{"SteeringAngle", {"x * 0.017453292519943", "rad"}}});

  const auto r = t.transform("SteeringAngle", 180.0);
  EXPECT_NEAR(r.value, M_PI, 1e-6);
  EXPECT_EQ(r.unit, "rad");
}

TEST(SignalTransformerTest, PercentToNormalized)
{
  SignalTransformer t;
  t.configure({{"BrakePedal", {"x / 100.0", ""}}});

  const auto r = t.transform("BrakePedal", 50.0);
  EXPECT_DOUBLE_EQ(r.value, 0.5);
}

// ── Constant expression ───────────────────────────────────────────────────────

TEST(SignalTransformerTest, ConstantExpression)
{
  SignalTransformer t;
  t.configure({{"StaticField", {"0.0", ""}}});
  const auto r = t.transform("StaticField", 999.0);
  EXPECT_DOUBLE_EQ(r.value, 0.0);
  EXPECT_DOUBLE_EQ(r.raw_value, 999.0);
}

// ── Complex expressions ───────────────────────────────────────────────────────

TEST(SignalTransformerTest, ComplexExpressionWithPiConstant)
{
  // exprtk provides pi as a constant
  SignalTransformer t;
  t.configure({{"Angle", {"x * pi / 180.0", "rad"}}});

  const auto r = t.transform("Angle", 90.0);
  EXPECT_NEAR(r.value, M_PI / 2.0, 1e-9);
}

TEST(SignalTransformerTest, MultipleSignals)
{
  SignalTransformer t;
  t.configure({
    {"Speed", {"x / 3.6", "m/s"}},
    {"Steer", {"x * pi / 180.0", "rad"}},
    {"Brake", {"x / 100.0", ""}},
  });

  EXPECT_NEAR(t.transform("Speed", 72.0).value, 20.0, 1e-9);
  EXPECT_NEAR(t.transform("Steer", 180.0).value, M_PI, 1e-6);
  EXPECT_NEAR(t.transform("Brake", 75.0).value, 0.75, 1e-9);
}

// ── Empty expression → passthrough (not an error) ────────────────────────────

TEST(SignalTransformerTest, EmptyExpressionString_Passthrough)
{
  SignalTransformer t;
  t.configure({{"Gear", {"", ""}}});  // expression is empty
  const auto r = t.transform("Gear", 4.0);
  EXPECT_DOUBLE_EQ(r.value, 4.0);
}

// ── Invalid expression → exception at configure ───────────────────────────────

TEST(SignalTransformerTest, InvalidExpressionThrowsOnConfigure)
{
  SignalTransformer t;
  EXPECT_THROW(t.configure({{"Bad", {"x + + + ", ""}}}), std::invalid_argument);
}

// ── Zero input ────────────────────────────────────────────────────────────────

TEST(SignalTransformerTest, ZeroInput)
{
  SignalTransformer t;
  t.configure({{"Speed", {"x / 3.6", "m/s"}}});
  const auto r = t.transform("Speed", 0.0);
  EXPECT_DOUBLE_EQ(r.value, 0.0);
}

// ── Negative input ────────────────────────────────────────────────────────────

TEST(SignalTransformerTest, NegativeInput)
{
  SignalTransformer t;
  t.configure({{"YawRate", {"x * pi / 180.0", "rad/s"}}});
  const auto r = t.transform("YawRate", -90.0);
  EXPECT_NEAR(r.value, -M_PI / 2.0, 1e-9);
}

}  // namespace vehicle_can_decoder
