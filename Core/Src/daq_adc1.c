#include "daq_adc1.h"

#include "main.h"
#include "board_temperature.h"
#include "daq_vdda.h"
#include "stm32u5xx_ll_adc.h"
#include <stdbool.h>
#include <math.h>

extern ADC_HandleTypeDef hadc1;

#define ADC1_FULL_SCALE_COUNTS 16383.0f
/* Sheets 2, 3, 7, 8. All signals are independent and ground-referenced. */
static const uint32_t g_adc1_channels[] = {
    ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8,
    ADC_CHANNEL_12, ADC_CHANNEL_11, /* ISENSE1 (J7), ISENSE2 (J8). */
    ADC_CHANNEL_17, ADC_CHANNEL_16, /* PB2 VMON, PB1 IMON. */
};

static uint8_t g_adc1_calibrated = 0U;
/* Latest valid VDDA (VREF+); NaN until the first VREFINT reading. */
static volatile float g_adc1_vdda_v = NAN;

float daq_adc1_vref_v(void)
{
  return daq_vdda_reference(g_adc1_vdda_v);
}
volatile uint32_t g_daq_adc1_read_status = UINT32_MAX;
volatile uint32_t g_daq_adc1_cr_on_error = 0U;
volatile uint32_t g_daq_adc1_isr_on_error = 0U;
volatile uint32_t g_daq_adc1_hal_state_on_error = 0U;

static HAL_StatusTypeDef daq_adc1_read_raw(uint32_t channel, uint32_t *raw, uint32_t timeout_ms)
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

  if (HAL_ADC_PollForConversion(&hadc1, timeout_ms) != HAL_OK)
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

bool daq_adc1_temperature_read(uint32_t *raw, uint32_t *ref)
{
  /* Keep acquisition error diagnostics about acquisition, not health sampling. */
  const uint32_t status = g_daq_adc1_read_status;
  const uint32_t cr = g_daq_adc1_cr_on_error, isr = g_daq_adc1_isr_on_error;
  const uint32_t state = g_daq_adc1_hal_state_on_error;
  const bool ok = daq_adc1_read_raw(ADC_CHANNEL_TEMPSENSOR, raw, 1U) == HAL_OK &&
                  daq_adc1_read_raw(ADC_CHANNEL_VREFINT, ref, 1U) == HAL_OK;
  g_daq_adc1_read_status = status;
  g_daq_adc1_cr_on_error = cr; g_daq_adc1_isr_on_error = isr;
  g_daq_adc1_hal_state_on_error = state;
  return ok;
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
    /* VREFINT stays connected (board_temperature_poll adds it too); its
     * start-up time (LL_ADC_DELAY_VREFINT_STAB_US) passes long before the
     * first 100 ms sample. */
    LL_ADC_SetCommonPathInternalChAdd(ADC12_COMMON, LL_ADC_PATH_INTERNAL_VREFINT);
  }

  return TX_SUCCESS;
}

/* VMON (R57 100k / R69 10k) and IMON (LM74202: 78.28 uA/A into R60 6.2k;
 * below 50 mA this is only indicative). volts/codes hold VMON then IMON. */
/* VREFINT first, so this sample's channels convert against the VDDA measured
 * microseconds before them: VMAIN stays right when +3V3A moves. */
static float daq_adc1_measure_vdda(daq_adc1_sample_t *sample, uint32_t timeout_ms)
{
  uint32_t raw = 0U;
  sample->vrefint_code = -1;
  sample->vdda_v = NAN;
  if (daq_adc1_read_raw(ADC_CHANNEL_VREFINT, &raw, timeout_ms) == HAL_OK) {
    sample->vrefint_code = (int32_t)raw;
    sample->vdda_v = daq_vdda_from_vrefint(raw, *VREFINT_CAL_ADDR);
    if (isfinite(sample->vdda_v)) g_adc1_vdda_v = sample->vdda_v;
  }
  return daq_vdda_reference(sample->vdda_v);
}

static void daq_adc1_store_supply(daq_adc1_sample_t *sample, const float volts[2], const int32_t codes[2])
{
  for (unsigned i = 0; i < 2U; ++i)
  {
    sample->power_monitor_v[i] = volts[i];
    sample->power_monitor_code[i] = codes[i];
  }
  sample->input_voltage_v = volts[0] * 11.0f;
  sample->input_current_a = volts[1] / (0.00007828f * 6200.0f);
}

static UINT daq_adc1_read_channels(unsigned first, unsigned count, uint32_t timeout_ms, float vref_v,
                                   float *volts, int32_t *codes)
{
  UINT result = TX_SUCCESS;
  for (unsigned i = 0; i < count; ++i)
  {
    uint32_t raw;
    volts[i] = NAN;
    codes[i] = -1;
    if (daq_adc1_read_raw(g_adc1_channels[first + i], &raw, timeout_ms) == HAL_OK)
    {
      volts[i] = (float)raw * vref_v / ADC1_FULL_SCALE_COUNTS;
      codes[i] = (int32_t)raw;
    }
    else
      result = TX_NOT_DONE;
  }
  return result;
}

UINT daq_adc1_sample_supply(daq_adc1_sample_t *sample)
{
  if (sample == NULL) return TX_PTR_ERROR;
  float volts[2];
  int32_t codes[2];
  /* Each conversion takes about 26 us (391.5 + 17.5 cycles at 16 MHz); a
   * 1 ms poll bound keeps a faulted ADC from stalling acquisition. */
  const float vref = daq_adc1_measure_vdda(sample, 1U);
  const UINT result = daq_adc1_read_channels(6U, 2U, 1U, vref, volts, codes);
  daq_adc1_store_supply(sample, volts, codes);
  return result;
}

UINT daq_adc1_sample(daq_adc1_sample_t *sample)
{
  if (sample == NULL) return TX_PTR_ERROR;
  float volts[8];
  int32_t codes[8];
  const float vref = daq_adc1_measure_vdda(sample, 10U);
  const UINT result = daq_adc1_read_channels(0U, 8U, 10U, vref, volts, codes);
  for (unsigned i = 0; i < 4U; ++i) sample->sar_inputs_v[i] = volts[i];
  for (unsigned i = 0; i < 2U; ++i)
  {
    sample->current_sense_v[i] = volts[4U + i];
    /* TMCS1108A1B: midpoint zero, nominal 50 mV/A (verify zero on hardware).
     * The zero is taken as half of the VREF+ used for this sample, as it was
     * half the assumed 3.3 V before VDDA was measured. */
    sample->current_sense_a[i] = (volts[4U + i] - vref * 0.5f) / 0.05f;
  }
  daq_adc1_store_supply(sample, &volts[6], &codes[6]);
  sample->aux_voltage_v = volts[5];
  /* Same ADC owner, after all acquired channels; no extra SD/network rows. */
  board_temperature_poll();
  return result;
}
