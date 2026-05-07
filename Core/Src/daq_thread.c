#include "DAQ-Threads.h"

#include "daq_board.h"
#include "main.h"
#include "stm32u5xx_hal_gpio.h"
#if (DISABLE_SD_CARD == 0U)
#include "sd_card.h"
#endif
#include "telemetry.h"

TX_THREAD daq_thread;

#define DAQ_THREAD_STACK_SIZE (32U * 1024U)
#define DAQ_SAMPLE_PERIOD_TICKS ((TX_TIMER_TICKS_PER_SECOND + 25U) / 50U)
#define DAQ_INPUT_VOLTAGE_LOW_V 8.5f
#define DAQ_KG1000_LOADCELL_SCALE 1.0f
#define DAQ_KG1000_LOADCELL_OFFSET 0.0f
#define DAQ_ENABLE_DUMMY_CAN_TELEMETRY 1U

static ULONG g_daq_thread_stack[DAQ_THREAD_STACK_SIZE / sizeof(ULONG)];

static void daq_store_snapshot_csv(const daq_snapshot_t *snapshot)
{
#if (DISABLE_SD_CARD == 0U)
  (void)sd_card_enqueue_csv_row("input_voltage_v", snapshot->monotonic_ms, snapshot->input_voltage_v);
  (void)sd_card_enqueue_csv_row("input_current_a", snapshot->monotonic_ms, snapshot->input_current_a);
  (void)sd_card_enqueue_csv_row("adc1_aux_v", snapshot->monotonic_ms, snapshot->adc1_aux_v);
  (void)sd_card_enqueue_csv_row("analog_in_1_v", snapshot->monotonic_ms, snapshot->analog_inputs_v[0]);
  (void)sd_card_enqueue_csv_row("analog_in_2_v", snapshot->monotonic_ms, snapshot->analog_inputs_v[1]);
  (void)sd_card_enqueue_csv_row("analog_in_3_v", snapshot->monotonic_ms, snapshot->analog_inputs_v[2]);
  (void)sd_card_enqueue_csv_row("analog_in_4_v", snapshot->monotonic_ms, snapshot->analog_inputs_v[3]);
  (void)sd_card_enqueue_csv_row("analog_out_1_v", snapshot->monotonic_ms, snapshot->analog_outputs_v[0]);
  (void)sd_card_enqueue_csv_row("analog_out_2_v", snapshot->monotonic_ms, snapshot->analog_outputs_v[1]);
  if (snapshot->ext_adc_sample_valid != 0U)
  {
    (void)sd_card_enqueue_csv_row("mcp3564r_code", snapshot->monotonic_ms, (float)snapshot->ext_adc_code);
    (void)sd_card_enqueue_csv_row("mcp3564r_temp_c", snapshot->monotonic_ms, snapshot->ext_adc_temp_c);
  }
#else
  (void)snapshot;
#endif
}

static void daq_publish_loadcell(const daq_snapshot_t *snapshot)
{
  float loadcell_kg1000;

  if (snapshot->ext_adc_sample_valid == 0U)
  {
    return;
  }

  loadcell_kg1000 = ((float)snapshot->ext_adc_code * DAQ_KG1000_LOADCELL_SCALE)
                  + DAQ_KG1000_LOADCELL_OFFSET;

  (void)log_telemetry_asynchronous(SEDS_DT_KG1000,
                                   &loadcell_kg1000,
                                   1U,
                                   sizeof(loadcell_kg1000));
}

static void daq_publish_dummy_can_telemetry(void)
{
#if (DAQ_ENABLE_DUMMY_CAN_TELEMETRY != 0U)
  static uint32_t dummy_count = 0U;
  float kg1000;

  kg1000 = 100.0f + ((float)(dummy_count % 100U) * 0.25f);
  (void)log_telemetry_asynchronous(SEDS_DT_KG1000,
                                   &kg1000,
                                   1U,
                                   sizeof(kg1000));

  dummy_count++;
#endif
}

void daq_thread_entry(ULONG initial_input)
{
  daq_snapshot_t snapshot;
  uint8_t power_loss_latched = 0U;
  uint8_t daq_ready = 1U;

  (void)initial_input;

  if (daq_board_init() != TX_SUCCESS)
  {
    daq_ready = 0U;
  }

  for (;;)
  {
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    daq_publish_dummy_can_telemetry();

    if (daq_ready == 0U)
    {
      tx_thread_sleep(DAQ_SAMPLE_PERIOD_TICKS);
      continue;
    }

    if (daq_board_sample(&snapshot) != TX_SUCCESS)
    {
      tx_thread_sleep(DAQ_SAMPLE_PERIOD_TICKS);
      continue;
    }

    daq_store_snapshot_csv(&snapshot);
    daq_publish_loadcell(&snapshot);

#if (DISABLE_SD_CARD == 0U)
    (void)sd_card_request_flush();
#endif

    if ((power_loss_latched == 0U) && (snapshot.input_voltage_v <= DAQ_INPUT_VOLTAGE_LOW_V))
    {
      power_loss_latched = 1U;
#if (DISABLE_SD_CARD == 0U)
      (void)sd_card_notify_power_loss();
#endif
    }

    if (power_loss_latched != 0U)
    {
      tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10U);
      continue;
    }

    (void)daq_board_ext_adc_start_dma();
    tx_thread_sleep(DAQ_SAMPLE_PERIOD_TICKS);
  }
}

UINT create_daq_thread(void)
{
  return tx_thread_create(&daq_thread,
                          "DAQ Thread",
                          daq_thread_entry,
                          0U,
                          g_daq_thread_stack,
                          sizeof(g_daq_thread_stack),
                          6U,
                          6U,
                          TX_NO_TIME_SLICE,
                          TX_AUTO_START);
}
