#include "daq_adc1.h"

#include "main.h"
#include <math.h>

extern ADC_HandleTypeDef hadc1;

#define DAQ_VREF_V 3.3f
#define ADC1_FULL_SCALE_COUNTS 16383.0f
/* Sheets 2, 3, 7, 8. All signals are independent and ground-referenced. */
static const uint32_t g_adc1_channels[] = {
    ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8,
    ADC_CHANNEL_12, ADC_CHANNEL_11, /* ISENSE1 (J7), ISENSE2 (J8). */
    ADC_CHANNEL_17, ADC_CHANNEL_16, /* PB2 VMON, PB1 IMON. */
};

static uint8_t g_adc1_calibrated = 0U;
volatile uint32_t g_daq_adc1_read_status = UINT32_MAX;
volatile uint32_t g_daq_adc1_cr_on_error = 0U;
volatile uint32_t g_daq_adc1_isr_on_error = 0U;
volatile uint32_t g_daq_adc1_hal_state_on_error = 0U;

static HAL_StatusTypeDef daq_adc1_read_raw(uint32_t channel, uint32_t *raw)
{
  ADC_ChannelConfTypeDef cfg = {0};

  cfg.Channel = channel;
  cfg.Rank = ADC_REGULAR_RANK_1;
  cfg.SamplingTime = ADC_SAMPLETIME_391CYCLES;
  cfg.SingleDiff = ADC_SINGLE_ENDED;
  cfg.OffsetNumber = ADC_OFFSET_NONE;
  cfg.Offset = 0U;
  cfg.OffsetRightShift = DISABLE;
  cfg.OffsetSignedSaturation = DISABLE;
  cfg.OffsetSaturation = DISABLE;
  cfg.OffsetSign = ADC_OFFSET_SIGN_NEGATIVE;

  if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK)
  {
    g_daq_adc1_read_status = 1U;
    return HAL_ERROR;
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    g_daq_adc1_read_status = 2U;
    g_daq_adc1_cr_on_error = hadc1.Instance->CR;
    g_daq_adc1_isr_on_error = hadc1.Instance->ISR;
    g_daq_adc1_hal_state_on_error = hadc1.State;
    return HAL_ERROR;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
  {
    g_daq_adc1_read_status = 3U;
    (void)HAL_ADC_Stop(&hadc1);
    return HAL_TIMEOUT;
  }

  *raw = HAL_ADC_GetValue(&hadc1);
  g_daq_adc1_read_status = 0U;
  (void)HAL_ADC_Stop(&hadc1);
  return HAL_OK;
}

UINT daq_adc1_init(void)
{
  if (g_adc1_calibrated == 0U)
  {
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
      return TX_NOT_DONE;
    }
    g_adc1_calibrated = 1U;
  }

  return TX_SUCCESS;
}

UINT daq_adc1_sample(daq_adc1_sample_t *sample)
{
  if (sample == NULL) return TX_PTR_ERROR;
  float volts[8];
  UINT result = TX_SUCCESS;
  for (unsigned i = 0; i < 8U; ++i)
  {
    uint32_t raw;
    volts[i] = NAN;
    if (daq_adc1_read_raw(g_adc1_channels[i], &raw) == HAL_OK)
      volts[i] = (float)raw * DAQ_VREF_V / ADC1_FULL_SCALE_COUNTS;
    else
      result = TX_NOT_DONE;
  }
  for (unsigned i = 0; i < 4U; ++i) sample->sar_inputs_v[i] = volts[i];
  for (unsigned i = 0; i < 2U; ++i)
  {
    sample->current_sense_v[i] = volts[4U + i];
    /* TMCS1108A1B: midpoint zero, nominal 50 mV/A (verify zero on hardware). */
    sample->current_sense_a[i] = (volts[4U + i] - DAQ_VREF_V * 0.5f) / 0.05f;
    sample->power_monitor_v[i] = volts[6U + i];
  }
  sample->input_voltage_v = volts[6] * 11.0f; /* R57 100k / R69 10k. */
  /* LM74202: 78.28 uA/A into R60 6.2k. Below 50mA this is only indicative. */
  sample->input_current_a = volts[7] / (0.00007828f * 6200.0f);
  sample->aux_voltage_v = volts[5];
  return result;
}
