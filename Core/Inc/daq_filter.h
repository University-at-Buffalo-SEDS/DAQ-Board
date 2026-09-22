#ifndef DAQ_FILTER_H
#define DAQ_FILTER_H
#include <stdint.h>
#include <math.h>

typedef struct { float value; uint32_t timestamp_ms; uint8_t valid; } daq_filter_t;
/* First-order low-pass, backward-Euler discretization. Keep identical to GS.
 * No deadband: small real loads and DC values are preserved. */
static inline float daq_filter_add(daq_filter_t *state, float raw, uint32_t now, float tau_ms)
{
  const uint32_t dt = now - state->timestamp_ms;
  if (!isfinite(raw)) { state->valid = 0U; return raw; }
  if (!state->valid || dt == 0U || dt > 2000U || tau_ms <= 0.0f)
    state->value = raw;
  else
    state->value += ((float)dt / (tau_ms + (float)dt)) * (raw - state->value);
  state->timestamp_ms = now;
  state->valid = 1U;
  return state->value;
}
#endif
