#include "daq_adc4.h"

#include "main.h"

extern ADC_HandleTypeDef hadc4;

#define DAQ_VREF_V 3.3f
#define ADC4_FULL_SCALE_COUNTS 4095.0f

static const uint32_t g_adc4_channels[DAQ_ADC4_CHANNEL_COUNT] = {
    ADC_CHANNEL_1,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_4,
};

static uint8_t g_adc4_calibrated = 0U;

static HAL_StatusTypeDef daq_adc4_read_raw(uint32_t channel, uint32_t *raw)
{
  ADC_ChannelConfTypeDef cfg = {0};

  cfg.Channel = channel;
  cfg.Rank = ADC4_RANK_CHANNEL_NUMBER;
  cfg.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  cfg.SingleDiff = ADC_SINGLE_ENDED;
  cfg.OffsetNumber = ADC_OFFSET_NONE;
  cfg.Offset = 0U;

  if (HAL_ADC_ConfigChannel(&hadc4, &cfg) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_ADC_Start(&hadc4) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_ADC_PollForConversion(&hadc4, 10U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc4);
    return HAL_TIMEOUT;
  }

  *raw = HAL_ADC_GetValue(&hadc4);
  (void)HAL_ADC_Stop(&hadc4);
  return HAL_OK;
}

static float daq_adc4_counts_to_voltage(uint32_t raw)
{
  return ((float)raw / ADC4_FULL_SCALE_COUNTS) * DAQ_VREF_V;
}

UINT daq_adc4_init(void)
{
  if (g_adc4_calibrated == 0U)
  {
    if (HAL_ADCEx_Calibration_Start(&hadc4, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
      return TX_NOT_DONE;
    }
    g_adc4_calibrated = 1U;
  }

  return TX_SUCCESS;
}

UINT daq_adc4_sample(daq_adc4_sample_t *sample)
{
  uint32_t raw = 0U;

  if (sample == NULL)
  {
    return TX_PTR_ERROR;
  }

  for (uint32_t i = 0; i < DAQ_ADC4_CHANNEL_COUNT; ++i)
  {
    if (daq_adc4_read_raw(g_adc4_channels[i], &raw) != HAL_OK)
    {
      return TX_NOT_DONE;
    }
    sample->analog_inputs_v[i] = daq_adc4_counts_to_voltage(raw);
  }

  return TX_SUCCESS;
}
