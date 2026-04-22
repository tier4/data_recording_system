// Copyright 2026 TIER IV, Inc.

#pragma once
#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>

namespace vehicle_can_decoder
{

/// Configuration for a single signal transform.
struct TransformConfig
{
  /// exprtk expression to evaluate. The variable "x" holds the raw value.
  /// Example: "x / 3.6"  (km/h → m/s)
  /// If empty, the raw value is passed through unchanged.
  std::string expression;

  /// Physical unit of the output value (informational, written into Signal.unit).
  std::string unit;
};

/// Result of applying a transform to a raw signal value.
struct TransformResult
{
  double value;      ///< Value after transform (= raw_value if no expression)
  double raw_value;  ///< Original raw value before transform
  std::string unit;  ///< Unit string from TransformConfig (empty if none)
};

/// Evaluates exprtk expressions to transform raw decoded signal values.
///
/// Usage:
///   SignalTransformer t;
///   t.configure({{"VehicleSpeed", {"x / 3.6", "m/s"}}});
///   auto r = t.transform("VehicleSpeed", 100.0);  // r.value ≈ 27.78
///
/// Thread safety: configure() is not thread-safe. transform() is safe to
/// call concurrently after configure() completes.
class SignalTransformer
{
public:
  SignalTransformer();
  ~SignalTransformer();

  SignalTransformer(const SignalTransformer &) = delete;
  SignalTransformer & operator=(const SignalTransformer &) = delete;
  SignalTransformer(SignalTransformer &&) noexcept;
  SignalTransformer & operator=(SignalTransformer &&) noexcept;

  /// Compile all expressions from the config map.
  /// @throws std::invalid_argument if any expression fails to compile.
  /// @param configs  Map of signal name → transform config.
  void configure(const std::unordered_map<std::string, TransformConfig> & configs);

  /// Apply the transform for the named signal.
  /// If no transform is configured for this name, returns raw_value as value.
  TransformResult transform(const std::string & signal_name, double raw_value) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vehicle_can_decoder
