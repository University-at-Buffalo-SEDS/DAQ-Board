#include "daq_board.h"

#include "daq_adc1.h"
#include "daq_adc4.h"
#include "daq_dac.h"
#include "main.h"
#include "mcp3564r.h"
#include "telemetry.h"
#include "daq_rates.h"

#include <string.h>

extern SPI_HandleTypeDef hspi2;

/* Zero means ready. Non-zero values identify the initialization stage that
 * failed, which is readable by the firmware simulator and a debugger. */
volatile uint32_t g_daq_board_init_status = UINT32_MAX;
volatile uint32_t g_daq_board_sample_status = UINT32_MAX;

UINT daq_board_init(void)
{
  if (daq_adc1_init() != TX_SUCCESS)
  {
    g_daq_board_init_status = 1U;
    return TX_NOT_DONE;
  }

  if (daq_adc4_init() != TX_SUCCESS)
  {
    g_daq_board_init_status = 2U;
    return TX_NOT_DONE;
  }

  if (daq_dac_init() != TX_SUCCESS)
  {
    g_daq_board_init_status = 3U;
    return TX_NOT_DONE;
  }

  if (mcp3564r_init(&hspi2) != TX_SUCCESS)
  {
    g_daq_board_init_status = 4U;
    return TX_NOT_DONE;
  }
  g_daq_board_init_status = 0U;
  return TX_SUCCESS;
}

UINT daq_board_sample(daq_snapshot_t *snapshot)
{
  static daq_adc1_sample_t adc1_sample;
  static daq_adc4_sample_t adc4_sample;
  static uint32_t last_analog_ms;
  static uint8_t analog_started;
  daq_dac_sample_t dac_sample;
  mcp3564r_sample_t ext_adc_sample;

  if (snapshot == NULL)
  {
    g_daq_board_sample_status = 5U;
    return TX_PTR_ERROR;
  }

  memset(snapshot, 0, sizeof(*snapshot));
  snapshot->monotonic_ms = telemetry_now_ms();

  if (!analog_started || (uint32_t)(snapshot->monotonic_ms - last_analog_ms) >= DAQ_ANALOG_REPORT_PERIOD_MS)
  {
    /* Drivers mark individual failed inputs NaN. An unused/faulty input must
     * not stop the external ADC ring from being drained. */
    const UINT adc1_status = daq_adc1_sample(&adc1_sample);
    const UINT adc4_status = daq_adc4_sample(&adc4_sample);
    g_daq_board_sample_status = adc1_status != TX_SUCCESS ? 1U : adc4_status != TX_SUCCESS ? 2U : 0U;
    last_analog_ms = (uint32_t)snapshot->monotonic_ms;
    analog_started = 1U;
    snapshot->analog_sample_fresh = 1U;
  }

  if (daq_dac_sample(&dac_sample) != TX_SUCCESS)
  {
    g_daq_board_sample_status = 3U;
    return TX_NOT_DONE;
  }

  if (mcp3564r_get_sample(&ext_adc_sample) != TX_SUCCESS)
  {
    g_daq_board_sample_status = 4U;
    return TX_NOT_DONE;
  }

  snapshot->input_voltage_v = adc1_sample.input_voltage_v;
  snapshot->input_current_a = adc1_sample.input_current_a;
  snapshot->adc1_aux_v = adc1_sample.aux_voltage_v;
  memcpy(snapshot->analog_inputs_v, adc4_sample.analog_inputs_v, sizeof(adc4_sample.analog_inputs_v));
  memcpy(&snapshot->analog_inputs_v[4], adc1_sample.sar_inputs_v, sizeof(adc1_sample.sar_inputs_v));
  memcpy(snapshot->current_sense_v, adc1_sample.current_sense_v, sizeof(snapshot->current_sense_v));
  memcpy(snapshot->current_sense_a, adc1_sample.current_sense_a, sizeof(snapshot->current_sense_a));
  memcpy(snapshot->power_monitor_v, adc1_sample.power_monitor_v, sizeof(snapshot->power_monitor_v));
  memcpy(snapshot->analog_outputs_v, dac_sample.analog_outputs_v, sizeof(snapshot->analog_outputs_v));
  snapshot->ext_adc_channel = ext_adc_sample.channel;
  snapshot->ext_adc_sample_valid = ext_adc_sample.sample_valid;
  snapshot->ext_adc_dma_busy = ext_adc_sample.dma_busy;
  snapshot->ext_adc_monotonic_ms = ext_adc_sample.monotonic_ms;
  snapshot->ext_adc_code = ext_adc_sample.code;
  snapshot->ext_adc_voltage_v = ext_adc_sample.voltage_v;
  snapshot->ext_adc_loadcell_kg1000 = ext_adc_sample.raw_value;
  snapshot->ext_adc_temp_c = ext_adc_sample.temperature_c;
  snapshot->ext_adc_temp_code = ext_adc_sample.temperature_code;

  return TX_SUCCESS;
}

HAL_StatusTypeDef daq_board_ext_adc_start_dma(void)
{
  return mcp3564r_start_dma();
}
