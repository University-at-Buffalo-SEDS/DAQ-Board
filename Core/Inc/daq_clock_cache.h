#pragma once
#include <stdint.h>

/* Network workers publish a UTC/uptime pair. Acquisition only extrapolates
 * this bounded-age snapshot; it must never wait on a router mutex. */
typedef struct {
    uint64_t unix_ms;
    uint32_t monotonic_ms;
} daq_clock_cache_t;
static inline daq_clock_cache_t daq_clock_cache_make(uint64_t unix_ms, uint32_t now)
{
    daq_clock_cache_t result = {0U, now};
    if (unix_ms >= 315532800000ULL && unix_ms < 4354819200000ULL)
        result.unix_ms = unix_ms;
    return result;
}
static inline uint64_t daq_clock_cache_read(daq_clock_cache_t value, uint32_t now)
{
    const uint32_t age = now - value.monotonic_ms;
    return value.unix_ms != 0U && age <= 5000U ? value.unix_ms + age : 0U;
}
