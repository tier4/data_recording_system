// Copyright 2026 TIER IV, Inc.

#include "vehicle_can_decoder/timeout_monitor.hpp"

#include <gtest/gtest.h>

namespace vehicle_can_decoder
{

// ── Initial state ─────────────────────────────────────────────────────────────

TEST(TimeoutMonitorTest, InitialStatusIsInitial)
{
  TimeoutMonitor m;
  m.register_signal("speed");
  EXPECT_EQ(m.get_status("speed"), TimeoutMonitor::STATUS_INITIAL);
}

TEST(TimeoutMonitorTest, UnregisteredSignalReturnsInitial)
{
  TimeoutMonitor m;
  EXPECT_EQ(m.get_status("nonexistent"), TimeoutMonitor::STATUS_INITIAL);
}

// ── First receipt ─────────────────────────────────────────────────────────────

TEST(TimeoutMonitorTest, ReceiptChangesStatusToOk)
{
  TimeoutMonitor m;
  m.register_signal("speed");
  m.signal_received("speed", 1000);
  EXPECT_EQ(m.get_status("speed"), TimeoutMonitor::STATUS_OK);
}

TEST(TimeoutMonitorTest, AutoRegisterOnFirstReceipt)
{
  TimeoutMonitor m;
  // Do NOT register first; signal_received should auto-register
  m.signal_received("speed", 1000);
  EXPECT_EQ(m.get_status("speed"), TimeoutMonitor::STATUS_OK);
}

// ── No timeout when within window ────────────────────────────────────────────

TEST(TimeoutMonitorTest, NoTimeoutWithinWindow)
{
  TimeoutMonitor m;
  m.signal_received("speed", 1000);

  // 400 ms later, timeout = 500 ms → no timeout
  const auto timed_out = m.check_timeouts(1400, 500);
  EXPECT_TRUE(timed_out.empty());
  EXPECT_EQ(m.get_status("speed"), TimeoutMonitor::STATUS_OK);
}

// ── Timeout detection ─────────────────────────────────────────────────────────

TEST(TimeoutMonitorTest, TimeoutAfterThreshold)
{
  TimeoutMonitor m;
  m.signal_received("speed", 1000);

  // 600 ms later, timeout = 500 ms → should timeout
  const auto timed_out = m.check_timeouts(1600, 500);
  ASSERT_EQ(timed_out.size(), 1u);
  EXPECT_EQ(timed_out[0], "speed");
  EXPECT_EQ(m.get_status("speed"), TimeoutMonitor::STATUS_TIMEOUT);
}

// ── Recovery from timeout ─────────────────────────────────────────────────────

TEST(TimeoutMonitorTest, RecoveryFromTimeout)
{
  TimeoutMonitor m;
  m.signal_received("speed", 1000);
  m.check_timeouts(2000, 500);  // triggers timeout
  EXPECT_EQ(m.get_status("speed"), TimeoutMonitor::STATUS_TIMEOUT);

  m.signal_received("speed", 2001);
  EXPECT_EQ(m.get_status("speed"), TimeoutMonitor::STATUS_OK);
}

// ── No timeout for INITIAL state ─────────────────────────────────────────────

TEST(TimeoutMonitorTest, InitialSignalNotFlaggedAsTimeout)
{
  TimeoutMonitor m;
  m.register_signal("speed");
  // Never received; check_timeouts should not flag it
  const auto timed_out = m.check_timeouts(99999, 500);
  EXPECT_TRUE(timed_out.empty());
  EXPECT_EQ(m.get_status("speed"), TimeoutMonitor::STATUS_INITIAL);
}

// ── Multiple signals ──────────────────────────────────────────────────────────

TEST(TimeoutMonitorTest, MultipleSignalsSomeTimeout)
{
  TimeoutMonitor m;
  m.signal_received("speed", 1000);
  m.signal_received("steer", 1000);
  m.signal_received("brake", 1000);

  // Only speed is refreshed before check
  m.signal_received("speed", 1400);

  // 600 ms after initial receipt; speed refreshed at 1400 so still OK
  const auto timed_out = m.check_timeouts(1600, 500);

  // steer and brake should be timed out; speed should be OK
  EXPECT_EQ(m.get_status("speed"), TimeoutMonitor::STATUS_OK);
  EXPECT_EQ(m.get_status("steer"), TimeoutMonitor::STATUS_TIMEOUT);
  EXPECT_EQ(m.get_status("brake"), TimeoutMonitor::STATUS_TIMEOUT);
  EXPECT_EQ(timed_out.size(), 2u);
}

// ── timed_out_signals ─────────────────────────────────────────────────────────

TEST(TimeoutMonitorTest, TimedOutSignalsReturnsAll)
{
  TimeoutMonitor m;
  m.signal_received("a", 0);
  m.signal_received("b", 0);
  m.signal_received("c", 0);

  m.check_timeouts(1000, 500);  // all three timeout

  const auto ts = m.timed_out_signals();
  EXPECT_EQ(ts.size(), 3u);
}

TEST(TimeoutMonitorTest, TimedOutSignalsEmptyWhenAllOk)
{
  TimeoutMonitor m;
  m.signal_received("a", 0);
  m.check_timeouts(100, 500);  // within window
  EXPECT_TRUE(m.timed_out_signals().empty());
}

}  // namespace vehicle_can_decoder
