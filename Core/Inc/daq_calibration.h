#ifndef DAQ_CALIBRATION_H
#define DAQ_CALIBRATION_H

#include "sedsnet.h"

typedef struct
{
  float kg1000_slope;
  float kg1000_intercept;
  float iadc_slope;
  float iadc_intercept;
  /* KG50: ascending polynomial coefficients, input shift, tare offset. */
  float kg50[7];
  /* [KG1000 reference C, raw/C, KG50 reference C, raw/C]. */
  float thermal[4];
  float filter_tau_ms[2]; /* KG1000, KG50; 0 disables smoothing. */
} daq_calibration_t;

float daq_calibration_temperature_raw(const daq_calibration_t *calibration, unsigned channel, float raw, float temperature_c);
void daq_calibration_restore(void);
SedsResult daq_calibration_init(SedsRouter *router);
SedsResult daq_calibration_poll(SedsRouter *router);
daq_calibration_t daq_calibration_current(void);
float daq_calibration_apply_kg1000(float raw_value);
float daq_calibration_apply_kg50(const daq_calibration_t *calibration, float raw_value);

#endif
