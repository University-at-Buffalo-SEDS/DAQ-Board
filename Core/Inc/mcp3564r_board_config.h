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

#if ((DAQ_ACQ_CONFIG2_GAIN1 >> 3U) & 7U) != 1U
#error "Update ADC voltage conversion when changing the board PGA gain"
#endif
#if (DAQ_ACQ_CONFIG0_STANDBY & 0x80U) == 0U
#error "Board voltage conversion requires the internal reference"
#endif

static inline float mcp3564r_code_to_voltage(int32_t code)
{
  return ((float)code * MCP3564R_NOMINAL_VREF_V) / 8388608.0f;
}

/* CH6/7 are P4 HVAIN1/2 through 10k:1k dividers. */
static inline float mcp3564r_connector_voltage(unsigned channel, float adc_voltage)
{
  return channel >= 6U && channel <= 7U ? adc_voltage * 11.0f : adc_voltage;
}

/* DS20006391C equation 5-1, unity gain; acquire at the temperature clock. */
static inline float mcp3564r_code_to_temperature(int32_t code)
{
  return 0.00040096f * (float)code * MCP3564R_NOMINAL_VREF_V - 269.13f;
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
