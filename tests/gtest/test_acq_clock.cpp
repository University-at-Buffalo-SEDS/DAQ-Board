// The analog reporter ages auxiliary MCP3564R samples on the acquisition (HAL) millisecond clock, wrap-safe.
#include <gtest/gtest.h>
#include <cstdint>

extern "C" {
#include "daq_acq_clock.h"
}

namespace {
constexpr uint32_t kStale = 300U;
constexpr uint32_t kAhead = 100U;

// The old check: (uint32_t)(now - sample) <= stale on any clock pair.
bool old_fresh(uint32_t now, uint32_t sample) { return static_cast<uint32_t>(now - sample) <= kStale; }
}  // namespace

TEST(AcqSampleFresh, SamplesUpToTheStaleLimitAreFresh) {
  EXPECT_TRUE(daq_acq_sample_fresh(10000U, 10000U, kStale, kAhead));
  EXPECT_TRUE(daq_acq_sample_fresh(10000U, 9700U, kStale, kAhead));
  EXPECT_FALSE(daq_acq_sample_fresh(10000U, 9699U, kStale, kAhead));
}

// The reporter's clock read 50 ms behind the samples' clock (ThreadX ticks trailing the HAL tick in the native sim):
// with the old check every recent sample looked 4294967246 ms old and published NaN.
TEST(AcqSampleFresh, ASampleStampedAfterNowIsCurrentNotWrapped) {
  const uint32_t sample = 500050U, now = 500000U;
  EXPECT_FALSE(old_fresh(now, sample));
  EXPECT_TRUE(daq_acq_sample_fresh(now, sample, kStale, kAhead));
  EXPECT_FALSE(daq_acq_sample_fresh(now, now + kAhead + 1U, kStale, kAhead));
}

// The opposite skew (the reporter 50 ms ahead): an old sample must still age out.
TEST(AcqSampleFresh, AClockAheadStillAgesOldSamples) {
  const uint32_t now = 500050U;
  EXPECT_TRUE(daq_acq_sample_fresh(now, 500000U, kStale, kAhead));
  EXPECT_FALSE(daq_acq_sample_fresh(now, 499700U, kStale, kAhead));
}

TEST(AcqSampleFresh, WrapsAtTheThirtyTwoBitBoundary) {
  EXPECT_TRUE(daq_acq_sample_fresh(20U, 0xFFFFFFF0U, kStale, kAhead));         // 36 ms old across the wrap
  EXPECT_TRUE(daq_acq_sample_fresh(0xFFFFFFF0U, 20U, kStale, kAhead));         // 36 ms ahead across the wrap
  EXPECT_FALSE(daq_acq_sample_fresh(0x80000000U, 0U, kStale, kAhead));         // half a wrap: stale
}
