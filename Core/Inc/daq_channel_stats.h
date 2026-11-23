#ifndef DAQ_CHANNEL_STATS_H
#define DAQ_CHANNEL_STATS_H
/* Per-channel noise and drift for DAQ_INPUT_HEALTH v2. Pure: no HAL.
 *
 * Noise: RMS of consecutive-sample differences divided by sqrt(2), over a
 * 1 s window. For white noise this equals the sample standard deviation; a
 * slow load change or drift does not inflate it. Units are the channel's
 * own volts (ADC-pin volts for KG1000/AMP2, input volts for P6, connector
 * volts for the chamber PT and the excitation monitor).
 * Drift: change of the 10 s mean over the last 60 s, per minute. */
#include <math.h>
#include <stdint.h>
#include "daq_chamber_pressure.h"

/* P4/4 HVAIN2 (CH7) through the 10k:1k divider, when +9V6A_LC (the load-cell
 * excitation) is wired to it (GS loadcell investigation, hardware item 5).
 * -1: not wired; CH7 stays an ordinary auxiliary input. */
#ifndef DAQ_EXCITATION_ADC_CHANNEL
#define DAQ_EXCITATION_ADC_CHANNEL -1
#endif
#if DAQ_EXCITATION_ADC_CHANNEL != -1 && DAQ_EXCITATION_ADC_CHANNEL != 7
#error "The excitation monitor is HVAIN2 (7) or unassigned (-1)"
#endif
#if DAQ_EXCITATION_ADC_CHANNEL == 7 && DAQ_CHAMBER_ADC_CHANNEL == 7
#error "HVAIN2 cannot be both the chamber PT and the excitation monitor"
#endif

#define DAQ_STATS_KG1000 0U
#define DAQ_STATS_KG50 1U
#define DAQ_STATS_CHAMBER 2U
#define DAQ_STATS_EXCITATION 3U
#define DAQ_STATS_CHANNELS 4U
#define DAQ_STATS_NOISE_WINDOW_MS 1000U
#define DAQ_STATS_BLOCK_MS 10000U
#define DAQ_STATS_BLOCKS 7U /* 6 block steps = 60 s */

static inline int daq_stats_index(unsigned channel, unsigned kg50_channel)
{
  if (channel == 0U) return DAQ_STATS_KG1000;
  if (channel == kg50_channel) return DAQ_STATS_KG50;
  if (daq_chamber_is_channel(channel)) return DAQ_STATS_CHAMBER;
#if DAQ_EXCITATION_ADC_CHANNEL >= 0
  if (channel == (unsigned)DAQ_EXCITATION_ADC_CHANNEL) return DAQ_STATS_EXCITATION;
#endif
  return -1;
}

typedef struct {
  float last, noise_v, drift_v_per_min, now_v, min_v;
  double diff_sq, block_sum;
  uint32_t diff_n, block_n, window_start_ms, block_start_ms;
  float blocks[DAQ_STATS_BLOCKS];
  uint8_t has_last, started, block_count, block_next, noise_valid, drift_valid;
} daq_channel_stats_t;

static inline void daq_channel_stats_add(daq_channel_stats_t *s, float value, uint32_t now_ms)
{
  if (!isfinite(value)) { s->has_last = 0U; return; }
  if (!s->started) {
    s->started = 1U;
    s->window_start_ms = s->block_start_ms = now_ms;
    s->min_v = value;
  }
  s->now_v = value;
  if (value < s->min_v) s->min_v = value;
  if (s->has_last) {
    const double d = (double)value - (double)s->last;
    s->diff_sq += d * d;
    ++s->diff_n;
  }
  s->last = value;
  s->has_last = 1U;
  if ((uint32_t)(now_ms - s->window_start_ms) >= DAQ_STATS_NOISE_WINDOW_MS) {
    if (s->diff_n != 0U) {
      s->noise_v = (float)sqrt(s->diff_sq / (2.0 * s->diff_n));
      s->noise_valid = 1U;
    }
    s->diff_sq = 0.0;
    s->diff_n = 0U;
    s->window_start_ms = now_ms;
  }
  s->block_sum += value;
  ++s->block_n;
  if ((uint32_t)(now_ms - s->block_start_ms) >= DAQ_STATS_BLOCK_MS) {
    s->blocks[s->block_next] = (float)(s->block_sum / s->block_n);
    s->block_next = (uint8_t)((s->block_next + 1U) % DAQ_STATS_BLOCKS);
    if (s->block_count < DAQ_STATS_BLOCKS) ++s->block_count;
    if (s->block_count == DAQ_STATS_BLOCKS) {
      const float newest = s->blocks[(s->block_next + DAQ_STATS_BLOCKS - 1U) % DAQ_STATS_BLOCKS];
      const float oldest = s->blocks[s->block_next];
      s->drift_v_per_min = (newest - oldest) * (60000.0f / (float)(DAQ_STATS_BLOCK_MS * (DAQ_STATS_BLOCKS - 1U)));
      s->drift_valid = 1U;
    }
    s->block_sum = 0.0;
    s->block_n = 0U;
    s->block_start_ms = now_ms;
  }
}

/* One health word: noise in uV (low 16 bits, 0xFFFF none, 0xFFFE saturated)
 * | drift in uV/min (high 16 bits, signed, 0x8000 none, saturating). */
static inline uint32_t daq_channel_stats_word(const daq_channel_stats_t *s)
{
  uint32_t noise = 0xFFFFU;
  if (s->noise_valid) {
    const float uv = s->noise_v * 1e6f;
    noise = uv >= 65534.0f ? 0xFFFEU : (uint32_t)(uv + 0.5f);
  }
  int32_t drift = -32768;
  if (s->drift_valid) {
    const float uv = s->drift_v_per_min * 1e6f;
    drift = uv >= 32767.0f ? 32767 : uv <= -32767.0f ? -32767 : (int32_t)lrintf(uv);
  }
  return noise | ((uint32_t)(uint16_t)(int16_t)drift << 16U);
}
#endif
