#ifndef MCP3564R_BOARD_CONFIG_H
#define MCP3564R_BOARD_CONFIG_H

#include <stdint.h>

/* Schematic sheets 1, 9, 10: P7 -> AMP1 -> CH0; P8 -> AMP2 -> CH1.
 * Both inputs are amplified, ground-referenced outputs. CONFIG0 selects the
 * internal 2.4 V reference, also used for bridge excitation and amplifier bias.
 * CONFIG2 GAIN[2:0]=001 is unity gain, not 16x. */
#define MCP3564R_BOARD_CONFIG0 0x82U
#define MCP3564R_BOARD_CONFIG2 0xCFU
#define MCP3564R_BOARD_SCAN 0x000003U
#define MCP3564R_NOMINAL_VREF_V 2.4f

#if ((MCP3564R_BOARD_CONFIG2 >> 3U) & 7U) != 1U
#error "Update ADC voltage conversion when changing the board PGA gain"
#endif
#if (MCP3564R_BOARD_CONFIG0 & 0x80U) == 0U
#error "Board voltage conversion requires the internal reference"
#endif

static inline float mcp3564r_code_to_voltage(int32_t code)
{
  return ((float)code * MCP3564R_NOMINAL_VREF_V) / 8388608.0f;
}

/* Preserve the existing KG1000/KG50 calibration input exactly. This historical
 * scale is neither ADC-pin volts nor kg. Changing it would invalidate saved
 * GroundStation/DAQ calibration coefficients and previously captured points. */
static inline float mcp3564r_code_to_raw_value(int32_t code)
{
  return (((float)code * 2.2104f) / 16777216.0f) * 2.0f / 16.0f;
}

#endif
