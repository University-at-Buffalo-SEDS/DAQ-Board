#include <assert.h>
#include <stdint.h>
#include "daq_timestamp.h"

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
  assert(daq_sample_network_ms(unix_base, 6000, 6001) == unix_base);
  assert(daq_timestamp_ms(0, UINT64_C(0x100000001)) == UINT64_C(0x100000001));
  return 0;
}
