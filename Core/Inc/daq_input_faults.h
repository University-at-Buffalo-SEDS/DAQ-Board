#ifndef DAQ_INPUT_FAULTS_H
#define DAQ_INPUT_FAULTS_H
/* Live input and sensor fault detection, reported in DAQ_INPUT_HEALTH.
 * Pure state machine: no HAL, no RTOS; the caller serializes observations
 * against daq_input_faults_report(). Thresholds: daq_input_fault_config.h.
 *
 * DAQ_INPUT_HEALTH, 28 little-endian UInt32 words (version 3; version 2 sent
 * words 0-17 and version 1 words 0-9, with the same meaning):
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
 *  10-13 noise | drift of KG1000, selected KG50, chamber PT, excitation
 *      (daq_channel_stats_word: noise uV u16, drift uV/min s16; 0xFFFF /
 *      0x8000 none). Load cells in ADC-pin volts (P6 input volts), chamber
 *      and excitation in connector volts.
 *  14  excitation (HVAIN2, connector) minimum since the previous report |
 *      now (mV); 0xFFFF when DAQ_EXCITATION_ADC_CHANNEL is unassigned
 *  15  MCP3564R frames skipped | slot items skipped (u16 each, saturating)
 *  16  scans restarted after a lost DRDY | device recoveries + STATUS faults
 *  17  latest fast-scan end | latest slot end, us from the frame start
 *  18  ADC over range now (bits 0-9) | over range latched since the previous
 *      report (bits 16-25). Bit n = MCP3564R CH n (0..7), bit 8 = die
 *      temperature, bit 9 = P6 differential (daq_over_range.h). "Now": the
 *      channel's latest conversion was over range.
 *  19  direction of the now bits, set = below -full scale (bits 0-9) | 0
 *  20-24 over-range conversions since boot, u16 each, wrapping modulo 2^16
 *      (readers take differences): CH0|CH1, CH2|CH3, CH4|CH5, CH6|CH7,
 *      TEMP|P6
 *  25  VDDA (+3V3A, from VREFINT) maximum since the previous report | now (mV)
 *  26  episode counters of faults 8 | 9 (u16 each, saturating)
 *  27  applied MCP3564R reference, mV (bits 0-15) | set by the station this
 *      boot (bit 16), restored from flash (bit 17); neither: nominal 2.4 V
 * 0xFFFF in a mV half means no reading in that interval. Faults 0-7 keep
 * their version 2 bits and episode words; 8 and 9 are new in version 3. */
#include <math.h>
#include <stdint.h>
#include "daq_input_fault_config.h"
#include "daq_over_range.h"

enum {
  DAQ_FAULT_SUPPLY_LOW = 0,
  DAQ_FAULT_KG1000_HIGH_RAIL,
  DAQ_FAULT_KG1000_LOW_RAIL,
  DAQ_FAULT_KG50_HIGH_RAIL,
  DAQ_FAULT_KG50_LOW_RAIL,
  DAQ_FAULT_P6_LEG_IMBALANCE,
  DAQ_FAULT_P6_COMMON_MODE,
  DAQ_FAULT_CHAMBER_NO_SIGNAL,
  DAQ_FAULT_VDDA_HIGH,
  DAQ_FAULT_VDDA_LOW,
  DAQ_FAULT_COUNT
};
#define DAQ_INPUT_HEALTH_VERSION 3U
#define DAQ_INPUT_HEALTH_WORDS 28U
#define DAQ_INPUT_HEALTH_STATS_WORD 10U
#define DAQ_INPUT_HEALTH_EXCITATION_WORD 14U
#define DAQ_INPUT_HEALTH_FRAME_WORD 15U
#define DAQ_INPUT_HEALTH_OVR_WORD 18U
#define DAQ_INPUT_HEALTH_OVR_COUNT_WORD 20U
#define DAQ_INPUT_HEALTH_VDDA_WORD 25U
#define DAQ_INPUT_HEALTH_EPISODES_V3_WORD 26U
#define DAQ_INPUT_HEALTH_VREF_WORD 27U
#define DAQ_FAULT_V2_COUNT 8U
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
  uint16_t excitation_min_mv, excitation_mv;
  uint16_t vdda_max_mv, vdda_mv;
  uint16_t ovr_now, ovr_low, ovr_latched;
  uint16_t ovr_count[DAQ_OVR_CHANNELS]; /* wrapping */
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
  s->excitation_min_mv = s->excitation_mv = DAQ_INPUT_HEALTH_NO_MV;
  s->vdda_max_mv = s->vdda_mv = DAQ_INPUT_HEALTH_NO_MV;
}

/* Two counters in one word, each saturating at 0xFFFF. */
static inline uint32_t daq_input_health_pair(uint32_t low, uint32_t high)
{
  return (low > 0xFFFFU ? 0xFFFFU : low) | ((high > 0xFFFFU ? 0xFFFFU : high) << 16U);
}

/* Load-cell excitation monitor (HVAIN2, connector volts). Reported only. */
static inline void daq_input_faults_excitation(daq_input_faults_t *s, float connector_v)
{
  if (!isfinite(connector_v)) return;
  const uint16_t mv = daq_input_faults_mv(connector_v);
  s->excitation_mv = mv;
  if (s->excitation_min_mv == DAQ_INPUT_HEALTH_NO_MV || mv < s->excitation_min_mv) s->excitation_min_mv = mv;
}

/* One observation of fault `id`: `beyond_set` past the set level,
 * `within_clear` back inside the clear level (neither in the band between). */
static inline void daq_input_fault_step(daq_input_faults_t *s, unsigned id, int beyond_set,
                                        int within_clear, uint32_t now_ms)
{
  static const uint16_t set_ms[DAQ_FAULT_COUNT] = {
    DAQ_FAULT_SUPPLY_SET_MS, DAQ_FAULT_RAIL_SET_MS, DAQ_FAULT_RAIL_SET_MS,
    DAQ_FAULT_RAIL_SET_MS, DAQ_FAULT_RAIL_SET_MS, DAQ_FAULT_P6_IMBALANCE_SET_MS,
    DAQ_FAULT_P6_COMMON_SET_MS, DAQ_FAULT_CHAMBER_SET_MS, DAQ_FAULT_VDDA_SET_MS, DAQ_FAULT_VDDA_SET_MS};
  static const uint16_t clear_ms[DAQ_FAULT_COUNT] = {
    DAQ_FAULT_SUPPLY_CLEAR_MS, DAQ_FAULT_RAIL_CLEAR_MS, DAQ_FAULT_RAIL_CLEAR_MS,
    DAQ_FAULT_RAIL_CLEAR_MS, DAQ_FAULT_RAIL_CLEAR_MS, DAQ_FAULT_P6_IMBALANCE_CLEAR_MS,
    DAQ_FAULT_P6_COMMON_CLEAR_MS, DAQ_FAULT_CHAMBER_CLEAR_MS, DAQ_FAULT_VDDA_CLEAR_MS, DAQ_FAULT_VDDA_CLEAR_MS};
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

/* `since_t0_ms` for daq_input_faults_chamber when no launch window is open. */
#define DAQ_CHAMBER_NO_LAUNCH UINT32_MAX

/* Nonzero while the chamber no-signal check runs: always for a transducer
 * with a live zero, otherwise only in the burn window (config header). */
static inline int daq_input_faults_chamber_armed(uint32_t since_t0_ms)
{
  return DAQ_FAULT_CHAMBER_ALWAYS_ON ||
         (since_t0_ms >= DAQ_FAULT_CHAMBER_BURN_START_MS && since_t0_ms < DAQ_FAULT_CHAMBER_BURN_END_MS);
}

/* Assigned chamber transducer, connector volts; `since_t0_ms` is the time
 * since launch T0, or DAQ_CHAMBER_NO_LAUNCH. The reading is always reported;
 * the fault is inactive while the check is not armed. Episodes stay. */
static inline void daq_input_faults_chamber(daq_input_faults_t *s, float connector_v, uint32_t now_ms,
                                            uint32_t since_t0_ms)
{
  if (!isfinite(connector_v)) return;
  s->chamber_seen = 1U;
  const uint16_t mv = daq_input_faults_mv(connector_v);
  s->chamber_mv = mv;
  if (s->chamber_min_mv == DAQ_INPUT_HEALTH_NO_MV || mv < s->chamber_min_mv) s->chamber_min_mv = mv;
  if (!daq_input_faults_chamber_armed(since_t0_ms)) {
    daq_input_fault_idle(s, DAQ_FAULT_CHAMBER_NO_SIGNAL);
    return;
  }
  daq_input_fault_step(s, DAQ_FAULT_CHAMBER_NO_SIGNAL, connector_v < DAQ_FAULT_CHAMBER_NO_SIGNAL_V,
                       connector_v >= DAQ_FAULT_CHAMBER_CLEAR_V, now_ms);
}

/* The chamber channel's conversions in one window were all over range
 * (`direction` +1/-1). A 0-5 V transducer behind the 11:1 divider can only
 * reach the ADC's +/-26.4 V connector limit through a wiring fault (e.g. its
 * output shorted to a supply) or a damaged sensor; it is never "no signal".
 * The no-signal check therefore sees it as a present signal, and the
 * over-range bits (word 18) carry the condition; the station treats chamber
 * over range during the burn as overpressure or a sensor fault. */
static inline void daq_input_faults_chamber_over_range(daq_input_faults_t *s, uint32_t now_ms,
                                                       uint32_t since_t0_ms)
{
  s->chamber_seen = 1U;
  if (!daq_input_faults_chamber_armed(since_t0_ms)) {
    daq_input_fault_idle(s, DAQ_FAULT_CHAMBER_NO_SIGNAL);
    return;
  }
  daq_input_fault_step(s, DAQ_FAULT_CHAMBER_NO_SIGNAL, 0, 1, now_ms);
}

/* STM32 VDDA = +3V3A, measured from VREFINT (daq_vdda.h). */
static inline void daq_input_faults_vdda(daq_input_faults_t *s, float vdda_v, uint32_t now_ms)
{
  if (!isfinite(vdda_v)) return;
  const uint16_t mv = daq_input_faults_mv(vdda_v);
  s->vdda_mv = mv;
  if (s->vdda_max_mv == DAQ_INPUT_HEALTH_NO_MV || mv > s->vdda_max_mv) s->vdda_max_mv = mv;
  daq_input_fault_step(s, DAQ_FAULT_VDDA_HIGH, vdda_v > DAQ_FAULT_VDDA_HIGH_V,
                       vdda_v < DAQ_FAULT_VDDA_HIGH_CLEAR_V, now_ms);
  daq_input_fault_step(s, DAQ_FAULT_VDDA_LOW, vdda_v < DAQ_FAULT_VDDA_LOW_V,
                       vdda_v > DAQ_FAULT_VDDA_LOW_CLEAR_V, now_ms);
}

/* One acquisition window's over-range observations (daq_ovr_window_t). */
static inline void daq_input_faults_over_range(daq_input_faults_t *s, const daq_ovr_window_t *w)
{
  for (unsigned i = 0U; i < DAQ_OVR_CHANNELS; ++i) {
    const uint16_t bit = (uint16_t)(1U << i);
    s->ovr_count[i] = (uint16_t)(s->ovr_count[i] + w->count[i]);
    if (w->count[i] != 0U) s->ovr_latched |= bit;
    if (w->last[i] == 2) continue;
    if (w->last[i] != 0) s->ovr_now |= bit; else s->ovr_now &= (uint16_t)~bit;
    if (w->last[i] < 0) s->ovr_low |= bit; else s->ovr_low &= (uint16_t)~bit;
  }
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
  for (unsigned i = 0; i < DAQ_FAULT_V2_COUNT; ++i)
    words[2U + i / 2U] |= (uint32_t)s->fault[i].episodes << (16U * (i & 1U));
  words[DAQ_INPUT_HEALTH_EPISODES_V3_WORD] = s->fault[DAQ_FAULT_VDDA_HIGH].episodes |
      ((uint32_t)s->fault[DAQ_FAULT_VDDA_LOW].episodes << 16U);
  words[DAQ_INPUT_HEALTH_OVR_WORD] = (uint32_t)s->ovr_now | ((uint32_t)(s->ovr_latched | s->ovr_now) << 16U);
  words[DAQ_INPUT_HEALTH_OVR_WORD + 1U] = (uint32_t)(s->ovr_low & s->ovr_now);
  for (unsigned i = 0; i < DAQ_OVR_CHANNELS; ++i)
    words[DAQ_INPUT_HEALTH_OVR_COUNT_WORD + i / 2U] |= (uint32_t)s->ovr_count[i] << (16U * (i & 1U));
  words[DAQ_INPUT_HEALTH_VDDA_WORD] = s->vdda_max_mv | ((uint32_t)s->vdda_mv << 16U);
  words[6] = s->vmain_min_mv | ((uint32_t)s->vmain_mv << 16U);
  words[7] = s->chamber_min_mv | ((uint32_t)s->chamber_mv << 16U);
  words[8] = (s->kg50_input == 1U ? s->p6_imbalance_max_mv : DAQ_INPUT_HEALTH_NO_MV) |
             ((uint32_t)(s->kg50_input == 1U ? s->p6_common_mv : DAQ_INPUT_HEALTH_NO_MV) << 16U);
  words[9] = daq_input_faults_mv(DAQ_FAULT_SUPPLY_LOW_V) |
             ((uint32_t)daq_input_faults_mv(DAQ_FAULT_CHAMBER_NO_SIGNAL_V) << 16U);
  words[DAQ_INPUT_HEALTH_EXCITATION_WORD] = s->excitation_min_mv | ((uint32_t)s->excitation_mv << 16U);
  for (unsigned i = DAQ_INPUT_HEALTH_STATS_WORD; i < DAQ_INPUT_HEALTH_EXCITATION_WORD; ++i)
    words[i] = 0x8000FFFFU; /* no statistics unless the owner fills them in */
  s->latched = 0U;
  for (unsigned i = 0; i < DAQ_FAULT_COUNT; ++i)
    if (s->fault[i].raw || s->fault[i].active) s->latched |= (uint16_t)(1U << i);
  s->onset = 0U;
  s->vmain_min_mv = s->vmain_mv;
  s->chamber_min_mv = s->chamber_mv;
  s->excitation_min_mv = s->excitation_mv;
  s->vdda_max_mv = s->vdda_mv;
  s->ovr_latched = s->ovr_now;
  s->p6_imbalance_max_mv = DAQ_INPUT_HEALTH_NO_MV;
}

/* Firmware owner (daq_thread.c): copies the next report into `words` when
 * one is due at `now_ms` (the period has passed, or a new episode started
 * and the minimum spacing has passed). Returns 0 when none is due. */
int daq_input_health_take(uint32_t now_ms, uint32_t words[DAQ_INPUT_HEALTH_WORDS]);
#endif
