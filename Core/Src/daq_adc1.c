#include "daq_adc1.h"

#include "main.h"

extern ADC_HandleTypeDef hadc1;

#define DAQ_VREF_V 3.3f
#define ADC1_DIFF_MID_SCALE 8192.0f

#define INPUT_VOLTAGE_DIVIDER_GAIN 11.0f
#define INPUT_CURRENT_SENSE_GAIN 20.0f
#define ADC1_AUX_GAIN 1.0f

static const uint32_t g_adc1_channels[] = {
    ADC_CHANNEL_5,
    ADC_CHANNEL_7,
    ADC_CHANNEL_11,
};

static uint8_t g_adc1_calibrated = 0U;

static HAL_StatusTypeDef daq_adc1_read_raw(uint32_t channel, uint32_t *raw)
{
  ADC_ChannelConfTypeDef cfg = {0};

  cfg.Channel = channel;
  cfg.Rank = ADC_REGULAR_RANK_1;
  cfg.SamplingTime = ADC_SAMPLETIME_391CYCLES;
  cfg.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  cfg.OffsetNumber = ADC_OFFSET_NONE;
  cfg.Offset = 0U;
  cfg.OffsetRightShift = DISABLE;
  cfg.OffsetSignedSaturation = DISABLE;
  cfg.OffsetSaturation = DISABLE;
  cfg.OffsetSign = ADC_OFFSET_SIGN_NEGATIVE;

  if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return HAL_TIMEOUT;
  }

  *raw = HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);
  return HAL_OK;
}

static float daq_adc1_counts_to_diff_voltage(uint32_t raw)
{
  const float signed_counts = (float)((int32_t)raw - (int32_t)ADC1_DIFF_MID_SCALE);
  return (signed_counts / ADC1_DIFF_MID_SCALE) * DAQ_VREF_V;
}

UINT daq_adc1_init(void)
{
  if (g_adc1_calibrated == 0U)
  {
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
    {
      return TX_NOT_DONE;
    }
    g_adc1_calibrated = 1U;
  }

  return TX_SUCCESS;
}

UINT daq_adc1_sample(daq_adc1_sample_t *sample)
{
  uint32_t raw = 0U;

  if (sample == NULL)
  {
    return TX_PTR_ERROR;
  }

  if (daq_adc1_read_raw(g_adc1_channels[0], &raw) != HAL_OK)
  {
    return TX_NOT_DONE;
  }
  sample->input_voltage_v = daq_adc1_counts_to_diff_voltage(raw) * INPUT_VOLTAGE_DIVIDER_GAIN;

  if (daq_adc1_read_raw(g_adc1_channels[1], &raw) != HAL_OK)
  {
    return TX_NOT_DONE;
  }
  sample->input_current_a = daq_adc1_counts_to_diff_voltage(raw) * INPUT_CURRENT_SENSE_GAIN;

  if (daq_adc1_read_raw(g_adc1_channels[2], &raw) != HAL_OK)
  {
    return TX_NOT_DONE;
  }
  sample->aux_voltage_v = daq_adc1_counts_to_diff_voltage(raw) * ADC1_AUX_GAIN;

  return TX_SUCCESS;
}
