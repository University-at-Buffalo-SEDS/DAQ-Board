#ifndef DAQ_INPUT_FAULTS_H
#define DAQ_INPUT_FAULTS_H
/* Live input and sensor fault detection, reported in DAQ_INPUT_HEALTH.
 * Pure state machine: no HAL, no RTOS; the caller serializes observations
 * against daq_input_faults_report(). Thresholds: daq_input_fault_config.h.
 *
 * DAQ_INPUT_HEALTH, 10 little-endian UInt32 words:
 *   0  version (bits 0-7) | KG50 input is P6 (bit 8) | chamber reading
 *      seen (bit 9)
 *   1  active fault mask (bits 0-15) | latched mask (bits 16-31): any fault
 *      seen, even for one sample, since the previous report
 *   2-5 episode counters since boot, u16 each, saturating: fault 2n in the
 *      low half and 2n+1 in the high half of word 2+n
 *   6  VMAIN minimum since the previous report | VMAIN now (mV)
 *   7  chamber connector minimum since the previous report | now (mV)
 *   8  P6 |CH2 - CH3| maximum since the previous report | P6 leg mean now (mV)
 *   9  supply undervoltage level | chamber no-signal level (mV)
 * 0xFFFF in a mV half means no reading in that interval. */
#include <math.h>
#include <stdint.h>
#include "daq_input_fault_config.h"

enum {
  DAQ_FAULT_SUPPLY_LOW = 0,
  DAQ_FAULT_KG1000_HIGH_RAIL,
  DAQ_FAULT_KG1000_LOW_RAIL,
  DAQ_FAULT_KG50_HIGH_RAIL,
  DAQ_FAULT_KG50_LOW_RAIL,
  DAQ_FAULT_P6_LEG_IMBALANCE,
  DAQ_FAULT_P6_COMMON_MODE,
  DAQ_FAULT_CHAMBER_NO_SIGNAL,
  DAQ_FAULT_COUNT
};
#define DAQ_INPUT_HEALTH_VERSION 1U
#define DAQ_INPUT_HEALTH_WORDS 10U
#define DAQ_INPUT_HEALTH_NO_MV 0xFFFFU

typedef struct {
  uint32_t changed_ms; /* last entry into or exit from the raw fault level */
  uint16_t episodes;
  uint8_t raw;         /* past the set level, not yet back inside the clear level */
  uint8_t active;      /* debounced */
} daq_fault_t;

typedef struct {
  daq_fault_t fault[DAQ_FAULT_COUNT];
  uint16_t latched, onset;
  uint16_t vmain_min_mv, vmain_mv, chamber_min_mv, chamber_mv;
  uint16_t p6_imbalance_max_mv, p6_common_mv;
  uint8_t kg50_input, chamber_seen;
} daq_input_faults_t;

static inline uint16_t daq_input_faults_mv(float volts)
{
  if (!isfinite(volts)) return DAQ_INPUT_HEALTH_NO_MV;
  if (volts <= 0.0f) return 0U;
  if (volts >= 65.534f) return 65534U;
  return (uint16_t)(volts * 1000.0f + 0.5f);
}

static inline void daq_input_faults_reset(daq_input_faults_t *s)
{
  *s = (daq_input_faults_t){0};
  s->vmain_min_mv = s->vmain_mv = s->chamber_min_mv = s->chamber_mv = DAQ_INPUT_HEALTH_NO_MV;
  s->p6_common_mv = s->p6_imbalance_max_mv = DAQ_INPUT_HEALTH_NO_MV;
}

/* One observation of fault `id`: `beyond_set` past the set level,
 * `within_clear` back inside the clear level (neither in the band between). */
static inline void daq_input_fault_step(daq_input_faults_t *s, unsigned id, int beyond_set,
                                        int within_clear, uint32_t now_ms)
{
  static const uint16_t set_ms[DAQ_FAULT_COUNT] = {
    DAQ_FAULT_SUPPLY_SET_MS, DAQ_FAULT_RAIL_SET_MS, DAQ_FAULT_RAIL_SET_MS,
    DAQ_FAULT_RAIL_SET_MS, DAQ_FAULT_RAIL_SET_MS, DAQ_FAULT_P6_IMBALANCE_SET_MS,
    DAQ_FAULT_P6_COMMON_SET_MS, DAQ_FAULT_CHAMBER_SET_MS};
  static const uint16_t clear_ms[DAQ_FAULT_COUNT] = {
    DAQ_FAULT_SUPPLY_CLEAR_MS, DAQ_FAULT_RAIL_CLEAR_MS, DAQ_FAULT_RAIL_CLEAR_MS,
    DAQ_FAULT_RAIL_CLEAR_MS, DAQ_FAULT_RAIL_CLEAR_MS, DAQ_FAULT_P6_IMBALANCE_CLEAR_MS,
    DAQ_FAULT_P6_COMMON_CLEAR_MS, DAQ_FAULT_CHAMBER_CLEAR_MS};
  daq_fault_t *f = &s->fault[id];
  const uint16_t bit = (uint16_t)(1U << id);
  if (!f->raw && beyond_set) {
    f->raw = 1U;
    f->changed_ms = now_ms;
    if (f->episodes != UINT16_MAX) ++f->episodes;
    s->onset |= bit;
  } else if (f->raw && within_clear) {
    f->raw = 0U;
    f->changed_ms = now_ms;
  }
  const uint32_t held = now_ms - f->changed_ms;
  if (f->raw && !f->active && held >= set_ms[id]) f->active = 1U;
  else if (!f->raw && f->active && held >= clear_ms[id]) f->active = 0U;
  if (f->raw || f->active) s->latched |= bit;
}

/* An input no longer selected or assigned cannot be faulted. Episodes stay. */
static inline void daq_input_fault_idle(daq_input_faults_t *s, unsigned id)
{
  s->fault[id].raw = 0U;
  s->fault[id].active = 0U;
}

static inline void daq_input_faults_supply(daq_input_faults_t *s, float vmain_v, uint32_t now_ms)
{
  if (!isfinite(vmain_v)) return;
  const uint16_t mv = daq_input_faults_mv(vmain_v);
  s->vmain_mv = mv;
  if (s->vmain_min_mv == DAQ_INPUT_HEALTH_NO_MV || mv < s->vmain_min_mv) s->vmain_min_mv = mv;
  daq_input_fault_step(s, DAQ_FAULT_SUPPLY_LOW, vmain_v < DAQ_FAULT_SUPPLY_LOW_V,
                       vmain_v >= DAQ_FAULT_SUPPLY_RECOVERED_V, now_ms);
}

/* KG1000 (CH0) mean ADC-pin volts of one acquisition window. */
static inline void daq_input_faults_kg1000(daq_input_faults_t *s, float pin_v, uint32_t now_ms)
{
  if (isnan(pin_v)) return;
  daq_input_fault_step(s, DAQ_FAULT_KG1000_HIGH_RAIL, pin_v >= DAQ_FAULT_ADC_FULL_SCALE_V,
                       pin_v < DAQ_FAULT_ADC_FULL_SCALE_CLEAR_V, now_ms);
  daq_input_fault_step(s, DAQ_FAULT_KG1000_LOW_RAIL, pin_v <= DAQ_FAULT_KG1000_LOW_V,
                       pin_v > DAQ_FAULT_KG1000_LOW_CLEAR_V, now_ms);
}

/* Changing the KG50 input ends that input's faults. */
static inline void daq_input_faults_kg50_input(daq_input_faults_t *s, uint8_t input)
{
  if (input == s->kg50_input) return;
  s->kg50_input = input;
  daq_input_fault_idle(s, DAQ_FAULT_KG50_HIGH_RAIL);
  daq_input_fault_idle(s, DAQ_FAULT_KG50_LOW_RAIL);
  daq_input_fault_idle(s, DAQ_FAULT_P6_LEG_IMBALANCE);
  daq_input_fault_idle(s, DAQ_FAULT_P6_COMMON_MODE);
  s->p6_common_mv = DAQ_INPUT_HEALTH_NO_MV;
}

/* Selected KG50 input: AMP2/CH1 ADC-pin volts, or P6 differential volts. */
static inline void daq_input_faults_kg50(daq_input_faults_t *s, float volts, uint32_t now_ms)
{
  if (isnan(volts)) return;
  const float full = s->kg50_input == 1U ? DAQ_FAULT_P6_FULL_SCALE_V : DAQ_FAULT_ADC_FULL_SCALE_V;
  const float clear = s->kg50_input == 1U ? DAQ_FAULT_P6_FULL_SCALE_CLEAR_V : DAQ_FAULT_ADC_FULL_SCALE_CLEAR_V;
  daq_input_fault_step(s, DAQ_FAULT_KG50_HIGH_RAIL, volts >= full, volts < clear, now_ms);
  daq_input_fault_step(s, DAQ_FAULT_KG50_LOW_RAIL, volts <= -full, volts > -clear, now_ms);
}

/* P6 single-ended legs, CH2 (P6/3, S+) and CH3 (S-), ADC-pin volts. */
static inline void daq_input_faults_p6_legs(daq_input_faults_t *s, float plus_v, float minus_v,
                                            uint32_t now_ms)
{
  if (s->kg50_input != 1U || !isfinite(plus_v) || !isfinite(minus_v)) return;
  const float imbalance = fabsf(plus_v - minus_v);
  const float common = 0.5f * (plus_v + minus_v);
  const uint16_t mv = daq_input_faults_mv(imbalance);
  if (s->p6_imbalance_max_mv == DAQ_INPUT_HEALTH_NO_MV || mv > s->p6_imbalance_max_mv)
    s->p6_imbalance_max_mv = mv;
  s->p6_common_mv = daq_input_faults_mv(common);
  daq_input_fault_step(s, DAQ_FAULT_P6_LEG_IMBALANCE, imbalance > DAQ_FAULT_P6_IMBALANCE_V,
                       imbalance < DAQ_FAULT_P6_IMBALANCE_CLEAR_V, now_ms);
  daq_input_fault_step(s, DAQ_FAULT_P6_COMMON_MODE,
                       common < DAQ_FAULT_P6_COMMON_LOW_V || common > DAQ_FAULT_P6_COMMON_HIGH_V,
                       common > DAQ_FAULT_P6_COMMON_LOW_V + DAQ_FAULT_P6_COMMON_HYSTERESIS_V &&
                       common < DAQ_FAULT_P6_COMMON_HIGH_V - DAQ_FAULT_P6_COMMON_HYSTERESIS_V,
                       now_ms);
}

/* Assigned chamber transducer, connector volts. */
static inline void daq_input_faults_chamber(daq_input_faults_t *s, float connector_v, uint32_t now_ms)
{
  if (!isfinite(connector_v)) return;
  s->chamber_seen = 1U;
  const uint16_t mv = daq_input_faults_mv(connector_v);
  s->chamber_mv = mv;
  if (s->chamber_min_mv == DAQ_INPUT_HEALTH_NO_MV || mv < s->chamber_min_mv) s->chamber_min_mv = mv;
  daq_input_fault_step(s, DAQ_FAULT_CHAMBER_NO_SIGNAL, connector_v < DAQ_FAULT_CHAMBER_NO_SIGNAL_V,
                       connector_v >= DAQ_FAULT_CHAMBER_CLEAR_V, now_ms);
}

static inline uint16_t daq_input_faults_active(const daq_input_faults_t *s)
{
  uint16_t mask = 0U;
  for (unsigned i = 0; i < DAQ_FAULT_COUNT; ++i)
    if (s->fault[i].active) mask |= (uint16_t)(1U << i);
  return mask;
}

/* Fill one report and start the next interval: latched bits, onsets and the
 * per-interval minimum/maximum restart from the faults still present. */
static inline void daq_input_faults_report(daq_input_faults_t *s, uint32_t words[DAQ_INPUT_HEALTH_WORDS])
{
  for (unsigned i = 0; i < DAQ_INPUT_HEALTH_WORDS; ++i) words[i] = 0U;
  words[0] = DAQ_INPUT_HEALTH_VERSION | ((uint32_t)(s->kg50_input == 1U) << 8U) |
             ((uint32_t)(s->chamber_seen != 0U) << 9U);
  words[1] = (uint32_t)daq_input_faults_active(s) | ((uint32_t)s->latched << 16U);
  for (unsigned i = 0; i < DAQ_FAULT_COUNT; ++i)
    words[2U + i / 2U] |= (uint32_t)s->fault[i].episodes << (16U * (i & 1U));
  words[6] = s->vmain_min_mv | ((uint32_t)s->vmain_mv << 16U);
  words[7] = s->chamber_min_mv | ((uint32_t)s->chamber_mv << 16U);
  words[8] = (s->kg50_input == 1U ? s->p6_imbalance_max_mv : DAQ_INPUT_HEALTH_NO_MV) |
             ((uint32_t)(s->kg50_input == 1U ? s->p6_common_mv : DAQ_INPUT_HEALTH_NO_MV) << 16U);
  words[9] = daq_input_faults_mv(DAQ_FAULT_SUPPLY_LOW_V) |
             ((uint32_t)daq_input_faults_mv(DAQ_FAULT_CHAMBER_NO_SIGNAL_V) << 16U);
  s->latched = 0U;
  for (unsigned i = 0; i < DAQ_FAULT_COUNT; ++i)
    if (s->fault[i].raw || s->fault[i].active) s->latched |= (uint16_t)(1U << i);
  s->onset = 0U;
  s->vmain_min_mv = s->vmain_mv;
  s->chamber_min_mv = s->chamber_mv;
  s->p6_imbalance_max_mv = DAQ_INPUT_HEALTH_NO_MV;
}

/* Firmware owner (daq_thread.c): copies the next report into `words` when
 * one is due at `now_ms` (the period has passed, or a new episode started
 * and the minimum spacing has passed). Returns 0 when none is due. */
int daq_input_health_take(uint32_t now_ms, uint32_t words[DAQ_INPUT_HEALTH_WORDS]);
#endif
