#ifndef DAQ_OVER_RANGE_H
#define DAQ_OVER_RANGE_H
/* MCP3564R over-range classification. Pure: no HAL.
 *
 * The DAQ reads the MCP3564R with CONFIG3 DATA_FORMAT[1:0] = 11: a 25-bit
 * two's-complement code (sign in bit 24) with the channel ID in bits 31-28.
 * Microchip DS20006391C, section 5.6 "ADC Output Data Format" and Table 5-8:
 *   - codes -2^23 .. 2^23-1 cover the specified input range
 *     [-VREF/GAIN, +VREF/GAIN - 1 LSb];
 *   - the 25-bit formats "allow overrange": codes continue linearly to the
 *     clip at 0x0FFFFFF (+2^24-1) / 0x1000000 (-2^24), i.e. +/-2 VREF/GAIN;
 *   - "The ADC accuracy is not maintained on the full extended range, but
 *     only on a smaller range, which is approximately equal to +/-1.05 x
 *     VREF". Section 3.2: "The differential input voltage should not exceed
 *     an absolute of +/-VREF/GAIN for accurate measurement. If the input is
 *     out of range, the converter output code will be saturated or
 *     overloaded";
 *   - there is no over-range or saturation status bit (STATUS carries only
 *     DR, CRCCFG and POR), so the code itself is the only indication.
 *
 * A conversion is therefore over range when |code| > 2^23 - 1. The threshold
 * is the end of the specified range itself; no extra margin is needed inside
 * it, because the datasheet specifies the linearity over +/-VREF and only
 * loses accuracy beyond it. The digital code threshold is the same at every
 * PGA gain because the gain is applied before the modulator, so it covers:
 *   - CH0 (KG1000) and CH1 (AMP2) at gain 1: ADC pin beyond +/-2.4 V;
 *   - CH9 (P6 differential) at gain 16: input beyond +/-150 mV;
 *   - CH2..CH5 single-ended at gain 1: +/-2.4 V at the pin;
 *   - CH6/CH7 (HVAIN1/2, 10k:1k dividers): +/-26.4 V at the connector;
 *   - TEMP (die temperature): the same code range; a valid die reading is
 *     about 3 % of full scale, so an over-range TEMP code is a fault.
 * The 2026-11-16 fire's KG1000 code 0xB40000 (+11796480 = 1.40625 x 2^23,
 * "3.375 V") is over range: outside even the +/-1.05 VREF accuracy band. */
#include <stdint.h>

#define DAQ_ADC_FULL_SCALE_CODE 8388607 /* 2^23 - 1 */

/* +1 above +full scale, -1 below -full scale, 0 in range. */
static inline int8_t daq_adc_over_range(int32_t code)
{
  return code > DAQ_ADC_FULL_SCALE_CODE ? 1 : code < -DAQ_ADC_FULL_SCALE_CODE ? -1 : 0;
}

/* DAQ_INPUT_HEALTH over-range bits: bit n = MCP3564R CH n (0..7), then the
 * die temperature and the P6 differential (scan channel 9). */
#define DAQ_OVR_BIT_TEMPERATURE 8U
#define DAQ_OVR_BIT_P6 9U
#define DAQ_OVR_CHANNELS 10U

static inline int daq_ovr_bit(unsigned channel)
{
  return channel < 8U ? (int)channel : channel == 9U ? (int)DAQ_OVR_BIT_P6 : -1;
}

/* Over-range conversions of one channel within one acquisition window. */
typedef struct {
  uint16_t count[DAQ_OVR_CHANNELS];
  int8_t last[DAQ_OVR_CHANNELS]; /* latest conversion: -1/0/+1; 2 = none */
  int8_t dir[DAQ_OVR_CHANNELS];  /* latest over-range direction: -1/+1; 0 = none */
} daq_ovr_window_t;

static inline void daq_ovr_window_reset(daq_ovr_window_t *w)
{
  for (unsigned i = 0U; i < DAQ_OVR_CHANNELS; ++i) { w->count[i] = 0U; w->last[i] = 2; w->dir[i] = 0; }
}

static inline void daq_ovr_window_note(daq_ovr_window_t *w, unsigned bit, int8_t state)
{
  if (bit >= DAQ_OVR_CHANNELS) return;
  w->last[bit] = state;
  if (state == 0) return;
  w->dir[bit] = state;
  if (w->count[bit] != UINT16_MAX) ++w->count[bit];
}
#endif
