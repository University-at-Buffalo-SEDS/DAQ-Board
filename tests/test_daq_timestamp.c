#include <assert.h>
#include <stdint.h>
#include "daq_timestamp.h"
#include "board_uptime.h"

int main(void)
{
  const uint64_t unix_base = UINT64_C(1800000000000);
  /* Startup and a later unsynchronized sample advance without a master. */
  assert(daq_timestamp_ms(0, 0) == 0);
  assert(daq_timestamp_ms(0, 1234) == 1234);
  assert(daq_timestamp_ms(0, 5678) == 5678);
  assert(daq_sample_network_ms(0, 6000, 5678) == 0);
  /* A queued sample is dated at acquisition, not at batch-drain time. */
  const uint64_t synced = daq_sample_network_ms(unix_base, 6000, 5678);
  assert(synced == unix_base - 322);
  assert(daq_timestamp_ms(synced, 5678) == unix_base - 322);
  /* Already queued unsynchronized rows retain local time after sync arrives. */
  assert(daq_timestamp_ms(0, 5678) == 5678);
  /* Losing synchronization goes back to uptime, not zero. */
  assert(daq_timestamp_ms(daq_sample_network_ms(0, 9000, 8900), 8900) == 8900);
  assert(daq_sample_network_ms(10, 9000, 8900) == 0);
  /* A sample drained after the clock pair was read is dated after it. */
  assert(daq_sample_network_ms(unix_base, 6000, 6001) == unix_base + 1);
  /* Such a sample's 32-bit capture tick is newer than the reference, not a
   * wrapped 49-day-old capture: it must never project to uptime zero. */
  assert(board_uptime_project(168891U, 168892U) == 168892U);
  assert(board_uptime_project(UINT64_C(0x1fffffffe), 1U) == UINT64_C(0x200000001));
  assert(board_uptime_project(168892U, 168891U) == 168891U);
  assert(board_uptime_project(UINT64_C(0x100000001), 0xfffffffeU) == UINT64_C(0xfffffffe));
  assert(daq_timestamp_ms(0, UINT64_C(0x100000001)) == UINT64_C(0x100000001));
  return 0;
}
