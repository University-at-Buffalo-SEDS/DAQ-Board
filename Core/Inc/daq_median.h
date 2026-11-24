#ifndef DAQ_MEDIAN_H
#define DAQ_MEDIAN_H
/* Spike filters and report bins for the published/display streams. SD keeps
 * every raw conversion; nothing here touches it. Fixed state, no allocation.
 *
 * Running median of the last N samples (N odd, at most 7). An isolated
 * outlier (any single sample when N >= 3, up to (N-1)/2 consecutive samples)
 * never reaches the output. A genuine step reaches the output after
 * (N+1)/2 samples, i.e. (N-1)/2 samples of delay, with no overshoot or
 * smearing: the output equals an input value, never an average across the
 * step. Non-finite inputs are skipped and do not enter the window. */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define DAQ_MEDIAN_MAX 7U
typedef struct {
  float window[DAQ_MEDIAN_MAX];
  uint8_t size, count, next;
} daq_median_t;

static inline void daq_median_init(daq_median_t *m, uint8_t size)
{
  *m = (daq_median_t){0};
  m->size = size == 0U ? 1U : size > DAQ_MEDIAN_MAX ? DAQ_MEDIAN_MAX : size;
  if ((m->size & 1U) == 0U) --m->size;
}

/* Adds `value`; returns the median of the samples in the window. Until the
 * window has filled, the median of the samples so far (the lower middle for
 * an even count), so the first output is not delayed. */
static inline float daq_median_add(daq_median_t *m, float value)
{
  if (m->size == 0U) daq_median_init(m, 1U);
  if (isfinite(value)) {
    m->window[m->next] = value;
    m->next = (uint8_t)((m->next + 1U) % m->size);
    if (m->count < m->size) ++m->count;
  }
  if (m->count == 0U) return NAN;
  float sorted[DAQ_MEDIAN_MAX];
  for (unsigned i = 0; i < m->count; ++i) {
    const float v = m->window[i];
    unsigned j = i;
    while (j > 0U && sorted[j - 1U] > v) { sorted[j] = sorted[j - 1U]; --j; }
    sorted[j] = v;
  }
  return sorted[(m->count - 1U) / 2U];
}

/* Report bin on the microsecond sample clock: bin = t_us / period_us. A
 * bin's mean is complete when a sample of a later bin arrives; it carries the
 * time of the bin's centre. Mean in double precision, rounded once. Empty
 * bins (an input that stopped) produce no report rather than a stale one.
 *
 * ADC over-range conversions (daq_over_range.h) join a bin through
 * daq_bin_us_add_over: they place the bin in time but never enter its mean.
 * A bin with at least one in-range sample reports the mean of those; a bin
 * whose every conversion was over range reports +INFINITY or -INFINITY (the
 * direction of the last one), the published over-range marker. */
typedef struct { double sum; uint32_t count; uint64_t bin; float min, max; uint32_t over; int8_t over_dir; } daq_bin_us_t;
typedef struct { float mean, min, max; uint64_t centre_us; uint32_t count, over; } daq_bin_out_t;

/* Closes the current bin when `t_us` falls in a later one. */
static inline bool daq_bin_us_advance(daq_bin_us_t *b, uint64_t t_us, uint32_t period_us, daq_bin_out_t *out)
{
  const uint64_t bin = t_us / period_us;
  const bool complete = (b->count != 0U || b->over != 0U) && bin != b->bin;
  if (complete) {
    const float marker = b->over_dir < 0 ? -INFINITY : INFINITY;
    *out = (daq_bin_out_t){b->count ? (float)(b->sum / b->count) : marker, b->count ? b->min : marker,
                           b->count ? b->max : marker, b->bin * period_us + period_us / 2U, b->count, b->over};
    b->count = 0U;
    b->over = 0U;
  }
  b->bin = bin;
  return complete;
}

static inline bool daq_bin_us_add(daq_bin_us_t *b, float value, uint64_t t_us, uint32_t period_us,
                                  daq_bin_out_t *out)
{
  if (!isfinite(value) || period_us == 0U) return false;
  const bool complete = daq_bin_us_advance(b, t_us, period_us, out);
  if (b->count == 0U) { b->sum = 0.0; b->min = b->max = value; }
  b->sum += (double)value;
  if (value < b->min) b->min = value;
  if (value > b->max) b->max = value;
  ++b->count;
  return complete;
}

/* One over-range conversion (`direction` +1 or -1) at `t_us`. */
static inline bool daq_bin_us_add_over(daq_bin_us_t *b, int direction, uint64_t t_us, uint32_t period_us,
                                       daq_bin_out_t *out)
{
  if (period_us == 0U || direction == 0) return false;
  const bool complete = daq_bin_us_advance(b, t_us, period_us, out);
  ++b->over;
  b->over_dir = direction < 0 ? -1 : 1;
  return complete;
}
#endif
