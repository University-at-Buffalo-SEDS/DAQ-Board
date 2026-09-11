#ifndef DAQ_DOWNSAMPLE_H
#define DAQ_DOWNSAMPLE_H
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Sample-weighted boxcar before decimation: averages every acquired sample,
 * not only the latest value. Bounded state; no allocation. */
typedef struct { double sum; uint32_t count; uint32_t started_ms; bool started; } daq_downsample_t;
static inline bool daq_downsample_add(daq_downsample_t *state, float mean,
    uint32_t count, uint32_t now_ms, uint32_t period_ms, float *output)
{
    if (!state->started) { state->started = true; state->started_ms = now_ms; }
    if (count && isfinite(mean)) { state->sum += (double)mean * count; state->count += count; }
    if ((uint32_t)(now_ms - state->started_ms) < period_ms || !state->count) return false;
    *output = (float)(state->sum / state->count);
    state->sum = 0; state->count = 0; state->started_ms = now_ms;
    return true;
}
#endif
