#ifndef MCP3564R_BOARD_CONFIG_H
#define MCP3564R_BOARD_CONFIG_H

#include <stdint.h>

/* Schematic sheets 1, 9, 10: P7 -> AMP1 -> CH0; P8 -> AMP2 -> CH1.
 * Both inputs are amplified, ground-referenced outputs. CONFIG0 selects the
 * internal 2.4 V reference, also used for bridge excitation and amplifier bias.
 * CONFIG2 GAIN[2:0]=001 is unity gain, not 16x. */
#define MCP3564R_BOARD_CONFIG0 0x82U
#define MCP3564R_BOARD_CONFIG2 0xCFU
#define MCP3564R_BOARD_SCAN 0x000003U /* CH1, CH0 at the load-cell clock/filter. */
/* One auxiliary pass per period, then resume the two load cells. */
#ifndef MCP3564R_AUX_SCAN
#define MCP3564R_AUX_SCAN 0x0000FCU /* CH2..7, single-ended. Set 0 to disable. */
#endif
#if (MCP3564R_AUX_SCAN & ~0x0000FCU) != 0U
#error "Auxiliary mask must contain only CH2..7"
#endif
#define MCP3564R_AUX_INTERVAL_MS 100U
#define MCP3564R_AUX_TIMEOUT_MS 25U
/* The die sensor reads about -33 C on this board at 16 MHz, even with long
 * settling and TEMP-only scans. Measure it separately at MCLK/4 = 4 MHz,
 * close to the datasheet's 4.9152 MHz characterization, OSR 256, gain 1.
 * Keep the load-cell clock/filter/gain and voltage calibration unchanged. */
#define MCP3564R_TEMPERATURE_CONFIG1 0x8cU
#define MCP3564R_TEMPERATURE_SCAN 0x001000U
#define MCP3564R_TEMPERATURE_INTERVAL_MS 100U
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

/* Preserve the existing KG1000/KG50 calibration input exactly. This historical
 * scale is neither ADC-pin volts nor kg. Changing it would invalidate saved
 * GroundStation/DAQ calibration coefficients and previously captured points. */
static inline float mcp3564r_code_to_raw_value(int32_t code)
{
  return (((float)code * 2.2104f) / 16777216.0f) * 2.0f / 16.0f;
}

#endif
