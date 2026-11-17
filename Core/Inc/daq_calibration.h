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
#endif
daq_calibration_t daq_calibration_current(void);
float daq_calibration_apply_kg1000(float volts);
float daq_calibration_apply_kg50(const daq_calibration_t *calibration, float volts);

#endif
