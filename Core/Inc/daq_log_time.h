#pragma once
#include <stdint.h>
#include "board_uptime.h"
#ifdef TELEMETRY_ENABLED
#include "telemetry.h"
#endif
/* Local recording remains valid without a router. Zero means UTC is unknown. */
static inline uint64_t daq_log_monotonic_ms(void) {
#ifdef TELEMETRY_ENABLED
 return telemetry_now_ms();
#else
 return board_uptime_ticks();
#endif
}
static inline uint64_t daq_log_unix_ms(void) {
#ifdef TELEMETRY_ENABLED
 return telemetry_unix_ms();
#else
 return 0U;
#endif
}
static inline uint64_t daq_log_unix_ms_cached(void) {
#ifdef TELEMETRY_ENABLED
 return telemetry_unix_ms_cached();
#else
 return 0U;
#endif
}
