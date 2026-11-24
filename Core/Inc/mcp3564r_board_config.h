#ifndef MCP3564R_BOARD_CONFIG_H
#define MCP3564R_BOARD_CONFIG_H

#include <stdint.h>
#include "daq_chamber_pressure.h"
#include "daq_acq_schedule.h"

/* Schematic sheets 1, 9, 10: P7 -> AMP1 -> CH0; P8 -> AMP2 -> CH1.
 * Both inputs are amplified, ground-referenced outputs. CONFIG0 selects the
 * internal 2.4 V reference, also used for bridge excitation and amplifier bias.
 * CONFIG2 GAIN[2:0]=001 is unity gain, not 16x; P6 (CH2-CH3) uses x16.
 * Scan contents, filters and timing: daq_acq_schedule.h. */
#define MCP3564R_P6_GAIN DAQ_ACQ_P6_GAIN
#define MCP3564R_NOMINAL_VREF_V 2.4f
/* Accepted range of the reference setting. The datasheet internal reference
 * is 2.4 V +/-2 % (DS20006391C, IVREF); this board's part drifted to about
 * 2.2 V after heat during repair, so the bound is wide enough for a damaged
 * part and still rejects a typo. */
#define MCP3564R_VREF_MIN_V 1.9f
#define MCP3564R_VREF_MAX_V 2.6f

/* The reference voltage every code is converted with: per board, set by the
 * station (DAQ_MCP_REFERENCE, daq_calibration.c), kept in flash, nominal
 * until then. Full scale at gain G is +/-VREF/G (2^23 codes); the over-range
 * threshold is the code itself (daq_over_range.h) and does not depend on it.
 * Weak, so each translation unit including this header may carry the one
 * definition (host tests included); daq_calibration.c owns the writes. */
__attribute__((weak)) volatile float g_mcp3564r_vref_v = MCP3564R_NOMINAL_VREF_V;
static inline float mcp3564r_vref_v(void) { return g_mcp3564r_vref_v; }

#if ((DAQ_ACQ_CONFIG2_GAIN1 >> 3U) & 7U) != 1U
#error "Update ADC voltage conversion when changing the board PGA gain"
#endif
#if (DAQ_ACQ_CONFIG0_STANDBY & 0x80U) == 0U
#error "Board voltage conversion requires the internal reference"
#endif

static inline float mcp3564r_code_to_voltage(int32_t code)
{
  return ((float)code * mcp3564r_vref_v()) / 8388608.0f;
}

/* CH6/7 are P4 HVAIN1/2 through 10k:1k dividers. */
static inline float mcp3564r_connector_voltage(unsigned channel, float adc_voltage)
{
  return channel >= 6U && channel <= 7U ? adc_voltage * 11.0f : adc_voltage;
}

/* DS20006391C equation 5-1 (in terms of VREF), unity gain; acquire at the
 * temperature clock. */
static inline float mcp3564r_code_to_temperature(int32_t code)
{
  return 0.00040096f * (float)code * mcp3564r_vref_v() - 269.13f;
}

/* KG1000 (CH0) and KG50 AMP2 (CH1) are reported in ADC-pin volts, the same
 * mcp3564r_code_to_voltage() scale as every other channel.
 *
 * Firmware before this change reported them on a historical "raw" scale,
 * code * 2.2104 / 2^24 * 2 / 16 = code * 2.2104 / 2^27, which is neither
 * volts nor kg. Saved calibrations and recordings use that scale. One legacy
 * raw unit is
 *   volts / raw = (2.4 / 2^23) / (2.2104 / 2^27) = 2.4 * 16 / 2.2104
 *               = 38.4 / 2.2104 = 16000 / 921  (~17.3724213)
 * volts, exactly. Convert a legacy value with volts = raw * k; a legacy
 * kg-per-raw slope with kg/V = (kg/raw) / k. */
#define MCP3564R_LEGACY_RAW_TO_VOLTS (16000.0 / 921.0)

/* The pre-volts KG1000/KG50 scale. Only calibration migration and its tests
 * use it; acquisition never reports it. */
static inline float mcp3564r_code_to_legacy_raw_value(int32_t code)
{
  return (((float)code * 2.2104f) / 16777216.0f) * 2.0f / 16.0f;
}

#endif
