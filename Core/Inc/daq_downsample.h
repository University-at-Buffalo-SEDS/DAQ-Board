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
    state->sum = 0; state->count = 0;
    /* Retain the reporting phase across scheduler jitter. Starting a new
     * period at each late observation progressively lowers the report rate.
     * Skip missed windows rather than emitting stale catch-up bursts. */
    const uint32_t elapsed = now_ms - state->started_ms;
    state->started_ms += period_ms ? (elapsed / period_ms) * period_ms : elapsed;
    return true;
}

/* Load-cell network boxcar, one call per ADC conversion. Bins are fixed on the
 * board uptime clock (bin = sample_ms / period_ms), so they depend only on what
 * each SD raw record holds: raw_value and its uptime ms. A bin's mean is
 * complete once a conversion of a later bin arrives; the result carries the
 * uptime of the bin's last conversion. An SD reader repeating this on the raw
 * records reproduces each network value bit for bit (double sum in record
 * order, one binary32 rounding of sum/count). Non-finite inputs are skipped. */
typedef struct { double sum; uint32_t count; uint64_t bin; uint64_t last_ms; } daq_bin_average_t;
static inline bool daq_bin_average_add(daq_bin_average_t *state, float value,
    uint64_t sample_ms, uint32_t period_ms, float *output, uint64_t *output_ms)
{
    if (!isfinite(value) || period_ms == 0U) return false;
    const uint64_t bin = sample_ms / period_ms;
    const bool complete = state->count != 0U && bin != state->bin;
    if (complete) {
        *output = (float)(state->sum / state->count);
        *output_ms = state->last_ms;
        state->sum = 0; state->count = 0;
    }
    state->bin = bin; state->sum += (double)value; ++state->count; state->last_ms = sample_ms;
    return complete;
}
#endif
