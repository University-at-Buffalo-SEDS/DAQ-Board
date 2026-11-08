#ifndef DAQ_TIMESTAMP_H
#define DAQ_TIMESTAMP_H
#include <stdint.h>

/* Preserve uptime until network time is available. */
static inline uint64_t daq_timestamp_ms(uint64_t network_ms, uint64_t local_ms)
{
  return network_ms != 0U ? network_ms : local_ms;
}

/* Project network time back to acquisition, not the later drain time. */
static inline uint64_t daq_sample_network_ms(uint64_t network_now,
                                            uint64_t local_now,
                                            uint64_t sample_ms)
{
  if (network_now == 0U) return 0U;
  /* A sample drained after the clock pair was read is newer, not older. */
  if (sample_ms > local_now) return network_now + (sample_ms - local_now);
  const uint64_t age = local_now - sample_ms;
  return network_now >= age ? network_now - age : 0U;
}
#endif
