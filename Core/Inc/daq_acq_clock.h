#ifndef DAQ_ACQ_CLOCK_H
#define DAQ_ACQ_CLOCK_H
/* Acquisition microsecond clock: TIM2, a free-running 32-bit counter at
 * 1 MHz from the same APB1 timer clock as the HAL tick (TIM6, 1 MHz counter,
 * 1 ms update). The offset to the HAL tick is measured once, so
 * (TIM2 - offset) / 1000 equals HAL_GetTick() and every sample keeps the
 * board's millisecond uptime base with microsecond resolution. Pure helpers:
 * no HAL, so the host tests run them directly. */
#include <stdint.h>

/* HAL microseconds from one consistent reading of the tick and TIM6:
 * `pending` is TIM6's update flag (the counter wrapped but the tick
 * interrupt has not run yet), in which case `count` was re-read after it. */
static inline uint64_t daq_acq_hal_us(uint32_t tick_ms, uint32_t count, int pending)
{
  return ((uint64_t)tick_ms + (pending ? 1U : 0U)) * 1000ULL + count;
}

/* A 32-bit capture taken at most 2^31 us (35 min) before or after `now64`,
 * placed on the 64-bit timeline. */
static inline uint64_t daq_acq_project_us(uint64_t now64, uint32_t captured)
{
  const int32_t delta = (int32_t)(captured - (uint32_t)now64);
  if (delta < 0 && (uint64_t)(-(int64_t)delta) > now64) return 0U;
  return (uint64_t)((int64_t)now64 + delta);
}

/* Extend a free-running 32-bit counter; call at least once per wrap
 * (71.6 minutes at 1 MHz). */
static inline uint64_t daq_acq_extend_us(uint64_t *state, uint32_t raw)
{
  *state += (uint32_t)(raw - (uint32_t)*state);
  return *state;
}
#endif
