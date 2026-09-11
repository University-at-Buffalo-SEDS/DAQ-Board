#ifndef DAQ_CALIBRATION_H
#define DAQ_CALIBRATION_H

#include "sedsnet.h"

typedef struct
{
  float kg1000_slope;
  float kg1000_intercept;
  float iadc_slope;
  float iadc_intercept;
} daq_calibration_t;

void daq_calibration_restore(void);
SedsResult daq_calibration_init(SedsRouter *router);
SedsResult daq_calibration_poll(SedsRouter *router);
daq_calibration_t daq_calibration_current(void);
float daq_calibration_apply_kg1000(float raw_value);

#endif
