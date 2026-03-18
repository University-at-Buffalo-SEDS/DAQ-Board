#include "daq_dac.h"

#include "main.h"

extern DAC_HandleTypeDef hdac1;

#define DAQ_VREF_V 3.3f
#define DAC_FULL_SCALE_COUNTS 4095.0f

UINT daq_dac_init(void)
{
  (void)HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  (void)HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  return TX_SUCCESS;
}

UINT daq_dac_sample(daq_dac_sample_t *sample)
{
  if (sample == NULL)
  {
    return TX_PTR_ERROR;
  }

  sample->analog_outputs_v[0] =
      ((float)HAL_DAC_GetValue(&hdac1, DAC_CHANNEL_1) / DAC_FULL_SCALE_COUNTS) * DAQ_VREF_V;
  sample->analog_outputs_v[1] =
      ((float)HAL_DAC_GetValue(&hdac1, DAC_CHANNEL_2) / DAC_FULL_SCALE_COUNTS) * DAQ_VREF_V;

  return TX_SUCCESS;
}
