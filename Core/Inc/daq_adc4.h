#ifndef DAQ_ADC4_H
#define DAQ_ADC4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tx_api.h"

#define DAQ_ADC4_CHANNEL_COUNT 4U

typedef struct
{
  float analog_inputs_v[DAQ_ADC4_CHANNEL_COUNT];
} daq_adc4_sample_t;

UINT daq_adc4_init(void);
UINT daq_adc4_sample(daq_adc4_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* DAQ_ADC4_H */
