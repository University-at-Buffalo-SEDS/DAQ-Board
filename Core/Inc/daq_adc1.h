#ifndef DAQ_ADC1_H
#define DAQ_ADC1_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"
#include "tx_api.h"

typedef struct
{
  float input_voltage_v;
  float input_current_a;
  float aux_voltage_v;
} daq_adc1_sample_t;

UINT daq_adc1_init(void);
UINT daq_adc1_sample(daq_adc1_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* DAQ_ADC1_H */
