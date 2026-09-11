#include "DAQ-Threads.h"

#include "daq_board.h"
#include "main.h"
#include "stm32u5xx_hal_gpio.h"
#include "sd_card.h"
#include "telemetry.h"
#include "mcp3564r.h"
#include "daq_calibration.h"
#include "daq_rates.h"
#include "daq_downsample.h"
TX_THREAD daq_thread;

#define DAQ_THREAD_STACK_SIZE (32U * 1024U)
#define DAQ_SAMPLE_PERIOD_MS DAQ_ACQUISITION_PERIOD_MS
#define DAQ_SAMPLE_PERIOD_TICKS ((DAQ_SAMPLE_PERIOD_MS * TX_TIMER_TICKS_PER_SECOND) / 1000U)
#define DAQ_INPUT_VOLTAGE_LOW_V 8.5f
#define DAQ_ENABLE_DUMMY_CAN_TELEMETRY 0U
#define DAQ_RAW_BATCH_MAX DAQ_RAW_BATCH_CAPACITY
#define DAQ_SLOW_SENSOR_LOG_DIVIDER ((1000U + DAQ_SAMPLE_PERIOD_MS - 1U) / DAQ_SAMPLE_PERIOD_MS)

volatile uint32_t g_daq_sample_ok_count = 0U;
volatile uint32_t g_daq_sample_fail_count = 0U;
volatile uint32_t g_daq_init_fail_count = 0U;
volatile uint32_t g_daq_loadcell_publish_ok_count = 0U;
volatile uint32_t g_daq_loadcell_publish_fail_count = 0U;
volatile uint32_t g_daq_raw_samples_drained_count = 0U;
volatile uint32_t g_daq_nonzero_raw_sample_count = 0U;
volatile uint32_t g_daq_sd_raw_batch_drop_count = 0U;
volatile uint32_t g_daq_sd_network_row_ok_count = 0U;
volatile uint32_t g_daq_sd_network_row_fail_count = 0U;
volatile uint32_t g_daq_sample_overrun_count = 0U;

static ULONG g_daq_thread_stack[DAQ_THREAD_STACK_SIZE / sizeof(ULONG)];

static uint16_t daq_drain_ext_adc(daq_snapshot_t *snapshot,
                                  sd_raw_adc_record_t *records,
                                  uint16_t capacity,
                                  const daq_calibration_t *calibration)
{
  mcp3564r_sample_t sample;
  uint16_t count = 0U;
  int64_t code_sum = 0;
  float loadcell_sum = 0.0f;
  const uint64_t unix_now = telemetry_unix_ms();
  const uint64_t mono_now = telemetry_now_ms();

  if ((snapshot->ext_adc_sample_valid != 0U) && (capacity != 0U))
  {
    const uint64_t age_ms = (mono_now >= snapshot->ext_adc_monotonic_ms)
                                ? mono_now - snapshot->ext_adc_monotonic_ms
                                : 0U;
    if (records != NULL) records[count].monotonic_ms = (uint32_t)snapshot->ext_adc_monotonic_ms;
    if (records != NULL) records[count].network_unix_ms = (unix_now >= age_ms) ? unix_now - age_ms : 0U;
    if (records != NULL) records[count].raw_adc_code = snapshot->ext_adc_code;
    if (records != NULL) records[count].raw_value = snapshot->ext_adc_loadcell_kg1000;
    if (records != NULL) records[count].calibrated_value =
        (calibration->kg1000_slope * snapshot->ext_adc_loadcell_kg1000 + calibration->kg1000_intercept);
    if (snapshot->ext_adc_code != 0) g_daq_nonzero_raw_sample_count++;
    code_sum += snapshot->ext_adc_code;
    loadcell_sum += snapshot->ext_adc_loadcell_kg1000;
    count++;
  }

  while ((count < capacity) && (mcp3564r_pending_samples() != 0U) &&
         (mcp3564r_get_sample(&sample) == TX_SUCCESS) &&
         (sample.sample_valid != 0U))
  {
    const uint64_t age_ms = (mono_now >= sample.monotonic_ms)
                                ? mono_now - sample.monotonic_ms
                                : 0U;
    if (records != NULL) records[count].network_unix_ms = (unix_now >= age_ms) ? unix_now - age_ms : 0U;
    if (records != NULL) records[count].monotonic_ms = (uint32_t)sample.monotonic_ms;
    if (records != NULL) records[count].raw_adc_code = sample.code;
    if (records != NULL) records[count].raw_value = sample.loadcell_kg1000;
    if (records != NULL) records[count].calibrated_value =
        (calibration->kg1000_slope * sample.loadcell_kg1000 + calibration->kg1000_intercept);
    if (sample.code != 0) g_daq_nonzero_raw_sample_count++;
    code_sum += sample.code;
    loadcell_sum += sample.loadcell_kg1000;
    count++;
  }

  if (count != 0U)
  {
    snapshot->ext_adc_sample_valid = 1U;
    snapshot->ext_adc_code = (int32_t)(code_sum / (int64_t)count);
    snapshot->ext_adc_loadcell_kg1000 = loadcell_sum / (float)count;
    snapshot->ext_adc_voltage_v = snapshot->ext_adc_loadcell_kg1000;
    g_daq_raw_samples_drained_count += count;
  }

  return count;
}

static void daq_store_snapshot_csv(const daq_snapshot_t *snapshot, const daq_calibration_t *calibration)
{
#if (DISABLE_SD_CARD == 0U)
  (void)sd_card_enqueue_csv_row("input_voltage_v", snapshot->monotonic_ms, snapshot->input_voltage_v, calibration);
  (void)sd_card_enqueue_csv_row("input_current_a", snapshot->monotonic_ms, snapshot->input_current_a, calibration);
  (void)sd_card_enqueue_csv_row("adc1_aux_v", snapshot->monotonic_ms, snapshot->adc1_aux_v, calibration);
  (void)sd_card_enqueue_csv_row("analog_in_1_v", snapshot->monotonic_ms, snapshot->analog_inputs_v[0], calibration);
  (void)sd_card_enqueue_csv_row("analog_in_2_v", snapshot->monotonic_ms, snapshot->analog_inputs_v[1], calibration);
  (void)sd_card_enqueue_csv_row("analog_in_3_v", snapshot->monotonic_ms, snapshot->analog_inputs_v[2], calibration);
  (void)sd_card_enqueue_csv_row("analog_in_4_v", snapshot->monotonic_ms, snapshot->analog_inputs_v[3], calibration);
  (void)sd_card_enqueue_csv_row("analog_out_1_v", snapshot->monotonic_ms, snapshot->analog_outputs_v[0], calibration);
  (void)sd_card_enqueue_csv_row("analog_out_2_v", snapshot->monotonic_ms, snapshot->analog_outputs_v[1], calibration);
  if (snapshot->ext_adc_sample_valid != 0U)
  {
    (void)sd_card_enqueue_csv_row("mcp3564r_code", snapshot->monotonic_ms, (float)snapshot->ext_adc_code, calibration);
    (void)sd_card_enqueue_csv_row("mcp3564r_voltage_v", snapshot->monotonic_ms, snapshot->ext_adc_voltage_v, calibration);
    (void)sd_card_enqueue_csv_row("kg1000_raw_window_average", snapshot->monotonic_ms, snapshot->ext_adc_loadcell_kg1000, calibration);
    (void)sd_card_enqueue_csv_row("mcp3564r_temp_c", snapshot->monotonic_ms, snapshot->ext_adc_temp_c, calibration);
  }
#else
  (void)snapshot;
  (void)calibration;
#endif
}

static void daq_publish_loadcell(const daq_snapshot_t *snapshot, const daq_calibration_t *calibration)
{
  float loadcell_kg1000;

  if (snapshot->ext_adc_sample_valid == 0U)
  {
    return;
  }

  /* Use one calibrated 50 Hz aggregate for both live telemetry and the SD
   * replay row. Raw 3.90625 ksps conversions remain in mcp3564r_raw rows. */
  loadcell_kg1000 =
      (calibration->kg1000_slope * snapshot->ext_adc_loadcell_kg1000 + calibration->kg1000_intercept);

#if (DISABLE_SD_CARD == 0U)
  if (sd_card_enqueue_csv_row("kg1000_network",
                              snapshot->monotonic_ms,
                              loadcell_kg1000, calibration) == SD_CARD_STATUS_OK)
  {
    g_daq_sd_network_row_ok_count++;
  }
  else
  {
    g_daq_sd_network_row_fail_count++;
  }
#endif

  if (log_telemetry_asynchronous(SEDS_DT_KG1000,
                                 &loadcell_kg1000,
                                 1U,
                                 sizeof(loadcell_kg1000)) == SEDS_OK)
  {
    g_daq_loadcell_publish_ok_count++;
  }
  else
  {
    g_daq_loadcell_publish_fail_count++;
  }
}

#if (DAQ_ENABLE_DUMMY_CAN_TELEMETRY != 0U)
static void daq_publish_dummy_can_telemetry(void)
{
  const float kg1000 = 0.0f;

  (void)log_telemetry_asynchronous(SEDS_DT_KG1000,
                                   &kg1000,
                                   1U,
                                   sizeof(kg1000));
}
#endif

void daq_thread_entry(ULONG initial_input)
{
  daq_snapshot_t snapshot;
  uint8_t power_loss_latched = 0U;
  uint8_t daq_ready = 1U;
  uint32_t slow_sensor_log_counter = 0U;
  daq_downsample_t downsample = {0};
#if (DISABLE_SD_CARD == 0U)
  sd_raw_adc_record_t raw_records[DAQ_RAW_BATCH_MAX];
#endif

  (void)initial_input;

  if (daq_board_init() != TX_SUCCESS)
  {
    g_daq_init_fail_count++;
    daq_ready = 0U;
  }
  else
  {
    (void)daq_board_ext_adc_start_dma();
  }

  for (;;)
  {
    const ULONG cycle_started = tx_time_get();
#if (DAQ_ENABLE_DUMMY_CAN_TELEMETRY != 0U)
    daq_publish_dummy_can_telemetry();
#endif

    if (daq_ready == 0U)
    {
      tx_thread_sleep(DAQ_SAMPLE_PERIOD_TICKS);
      continue;
    }

    if (daq_board_sample(&snapshot) != TX_SUCCESS)
    {
      g_daq_sample_fail_count++;
      tx_thread_sleep(DAQ_SAMPLE_PERIOD_TICKS);
      continue;
    }

    g_daq_sample_ok_count++;
    const daq_calibration_t calibration = daq_calibration_current();

#if (DISABLE_SD_CARD == 0U)
    const uint16_t raw_count = daq_drain_ext_adc(&snapshot,
                                                 raw_records,
                                                 DAQ_RAW_BATCH_MAX, &calibration);
    if ((raw_count != 0U) &&
        (sd_card_enqueue_raw_adc_samples(raw_records, raw_count, &calibration) != SD_CARD_STATUS_OK))
    {
      g_daq_sd_raw_batch_drop_count++;
    }
#else
    const uint16_t raw_count = daq_drain_ext_adc(&snapshot, NULL, DAQ_RAW_BATCH_MAX, &calibration);
#endif

    if (++slow_sensor_log_counter >= DAQ_SLOW_SENSOR_LOG_DIVIDER)
    {
      slow_sensor_log_counter = 0U;
      daq_store_snapshot_csv(&snapshot, &calibration);
    }
    float filtered;
    if (daq_downsample_add(&downsample, snapshot.ext_adc_loadcell_kg1000,
                           raw_count, (uint32_t)snapshot.monotonic_ms,
                           DAQ_BROADCAST_PERIOD_MS, &filtered))
    {
      /* SD kg1000_network and SEDSNet receive this identical filtered value.
       * Raw records above retain individual, unfiltered conversions. */
      daq_snapshot_t published = snapshot;
      published.ext_adc_loadcell_kg1000 = filtered;
      daq_publish_loadcell(&published, &calibration);
    }

    (void)daq_board_ext_adc_start_dma();

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

    /* Include acquisition/publish work in the 20 ms period. Sleeping for a
     * whole period after doing that work silently reduces the sample rate. */
    const ULONG elapsed = tx_time_get() - cycle_started;
    if (elapsed < DAQ_SAMPLE_PERIOD_TICKS)
    {
      tx_thread_sleep(DAQ_SAMPLE_PERIOD_TICKS - elapsed);
    }
    else
    {
      g_daq_sample_overrun_count++;
      tx_thread_sleep(1U);
    }
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
