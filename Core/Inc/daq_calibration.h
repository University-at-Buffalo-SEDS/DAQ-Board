#ifndef DAQ_CALIBRATION_H
#define DAQ_CALIBRATION_H

#include <stdint.h>
/* Internal retained SD capture lease; carried in existing alignment padding. */
typedef uint32_t sd_capture_t;
#ifdef TELEMETRY_ENABLED
#include "sedsnet.h"
#endif

/* Load-cell inputs are ADC-pin volts: slopes are kg/V, KG50 x0 is volts and
 * the thermal terms are V/C. */
typedef struct
{
  float kg1000_slope;
  float kg1000_intercept;
  float iadc_slope;
  float iadc_intercept;
  /* KG50: ascending polynomial coefficients, input shift, tare offset. */
  float kg50[7];
  /* [KG1000 reference C, V/C, KG50 reference C, V/C]. */
  float thermal[4];
  float filter_tau_ms[2]; /* KG1000, KG50; 0 disables smoothing. */
} daq_calibration_t;

float daq_calibration_temperature_raw(const daq_calibration_t *calibration, unsigned channel, float raw, float temperature_c);
void daq_calibration_restore(void);
/* Convert coefficients for the pre-volts raw input scale to volts, keeping kg
 * for the same ADC code (see MCP3564R_LEGACY_RAW_TO_VOLTS). */
void daq_calibration_from_legacy_raw(daq_calibration_t *calibration);
#ifdef TELEMETRY_ENABLED
SedsResult daq_calibration_init(SedsRouter *router);
SedsResult daq_calibration_poll(SedsRouter *router);
/* Config identity for the station's resync (BOARD_HEALTH words 13..15, tag
 * DAQ_CONFIG_TAG): CRC-32 of the applied calibration in the station's payload
 * order (loadcell 4, KG50 7, thermal 4, filter 2 floats), and in *flags which
 * values came from the station this boot, whether flash supplied them and the
 * applied 50 kg input. */
#define DAQ_CONFIG_TAG UINT32_C(0x31474643) /* "CFG1" little-endian */
#define DAQ_CONFIG_FROM_STATION_LOADCELL 0x01U
#define DAQ_CONFIG_FROM_STATION_KG50 0x02U
#define DAQ_CONFIG_FROM_STATION_THERMAL 0x04U
#define DAQ_CONFIG_FROM_STATION_FILTER 0x08U
#define DAQ_CONFIG_FROM_STATION_INPUT 0x10U
#define DAQ_CONFIG_RESTORED 0x20U
#define DAQ_CONFIG_NOT_DURABLE 0x40U
#define DAQ_CONFIG_INPUT_RESTORED 0x80U
#define DAQ_CONFIG_INPUT_SHIFT 8U
uint32_t daq_calibration_identity(uint32_t *flags);
#endif
daq_calibration_t daq_calibration_current(void);
float daq_calibration_apply_kg1000(float volts);
float daq_calibration_apply_kg50(const daq_calibration_t *calibration, float volts);

#endif
