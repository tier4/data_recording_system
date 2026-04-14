// Copyright 2026 TIER IV, Inc.

// exprtk is a very large header (~40k lines). Including it in exactly one
// translation unit prevents bloating other compilation units.
#include "vehicle_can_bridge/signal_transformer.hpp"

#include "exprtk.hpp"

#include <stdexcept>
#include <string>
#include <unordered_map>

namespace vehicle_can_bridge
{

// ── Impl ──────────────────────────────────────────────────────────────────────

struct CompiledExpr
{
  // x and expression are mutable because expression evaluation writes to the
  // symbol table variable (x) even though the overall transform operation is
  // logically const (same inputs → same outputs).
  mutable double x{0.0};
  exprtk::symbol_table<double> symbol_table;
  mutable exprtk::expression<double> expression;
  exprtk::parser<double> parser;
  std::string unit;

  /// Compile the given expression string. Binds 'x' as the input variable.
  /// @throws std::invalid_argument on compile error.
  void compile(const std::string & expr_str, const std::string & unit_str)
  {
    unit = unit_str;
    symbol_table.add_variable("x", x);
    symbol_table.add_constants();
    expression.register_symbol_table(symbol_table);

    if (!parser.compile(expr_str, expression)) {
      throw std::invalid_argument(
        "Failed to compile expression \"" + expr_str + "\": " + parser.error());
    }
  }

  double evaluate(double raw) const
  {
    x = raw;
    return expression.value();
  }
};

struct SignalTransformer::Impl
{
  std::unordered_map<std::string, std::unique_ptr<CompiledExpr>> exprs;
};

// ── Constructors / destructor ─────────────────────────────────────────────────

SignalTransformer::SignalTransformer() : impl_(std::make_unique<Impl>()) {}

SignalTransformer::~SignalTransformer() = default;

SignalTransformer::SignalTransformer(SignalTransformer &&) noexcept = default;
SignalTransformer & SignalTransformer::operator=(SignalTransformer &&) noexcept = default;

// ── configure ─────────────────────────────────────────────────────────────────

void SignalTransformer::configure(const std::unordered_map<std::string, TransformConfig> & configs)
{
  impl_->exprs.clear();

  for (const auto & [signal_name, cfg] : configs) {
    if (cfg.expression.empty()) {
      // No expression → passthrough; store just the unit
      // We represent this as an absent entry: transform() handles it.
      continue;
    }

    auto compiled = std::make_unique<CompiledExpr>();
    // compile() throws std::invalid_argument on error
    compiled->compile(cfg.expression, cfg.unit);
    impl_->exprs.emplace(signal_name, std::move(compiled));
  }
}

// ── transform ─────────────────────────────────────────────────────────────────

TransformResult SignalTransformer::transform(
  const std::string & signal_name, double raw_value) const
{
  auto it = impl_->exprs.find(signal_name);
  if (it == impl_->exprs.end()) {
    // No configured expression: passthrough
    return TransformResult{raw_value, raw_value, ""};
  }

  // x and expression are mutable so this const method can update the
  // symbol table variable before calling expression.value().
  const double transformed = it->second->evaluate(raw_value);

  return TransformResult{transformed, raw_value, it->second->unit};
}

}  // namespace vehicle_can_bridge
