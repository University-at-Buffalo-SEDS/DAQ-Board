#ifndef DAQ_DAC_H
#define DAQ_DAC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tx_api.h"

typedef struct
{
  float analog_outputs_v[2];
} daq_dac_sample_t;

UINT daq_dac_init(void);
UINT daq_dac_sample(daq_dac_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* DAQ_DAC_H */
