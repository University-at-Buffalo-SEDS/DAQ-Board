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
#include "daq_timestamp.h"
TX_THREAD daq_thread;

#define DAQ_THREAD_STACK_SIZE (32U * 1024U)
#define DAQ_SAMPLE_PERIOD_MS DAQ_ACQUISITION_PERIOD_MS
#define DAQ_SAMPLE_PERIOD_TICKS ((DAQ_SAMPLE_PERIOD_MS * TX_TIMER_TICKS_PER_SECOND) / 1000U)
#define DAQ_INPUT_VOLTAGE_LOW_V 8.5f
#define DAQ_INPUT_VOLTAGE_RECOVERED_V 9.0f
#define DAQ_ENABLE_DUMMY_CAN_TELEMETRY 0U
#define DAQ_RAW_BATCH_MAX DAQ_RAW_BATCH_CAPACITY
#define DAQ_SLOW_SENSOR_LOG_DIVIDER ((1000U + DAQ_SAMPLE_PERIOD_MS - 1U) / DAQ_SAMPLE_PERIOD_MS)

volatile uint32_t g_daq_sample_ok_count = 0U;
volatile uint32_t g_daq_sample_fail_count = 0U;
volatile uint32_t g_daq_init_fail_count = 0U;
volatile uint32_t g_daq_loadcell_publish_ok_count = 0U;
volatile uint32_t g_daq_loadcell_publish_fail_count = 0U;
volatile uint32_t g_daq_kg50_publish_ok_count = 0U;
volatile uint32_t g_daq_kg50_publish_fail_count = 0U;
volatile uint32_t g_daq_raw_samples_drained_count = 0U;
volatile uint32_t g_daq_nonzero_raw_sample_count = 0U;
volatile uint32_t g_daq_sd_raw_batch_drop_count = 0U;
volatile uint32_t g_daq_sd_network_row_ok_count = 0U;
volatile uint32_t g_daq_sd_network_row_fail_count = 0U;
volatile uint32_t g_daq_sample_overrun_count = 0U;

static ULONG g_daq_thread_stack[DAQ_THREAD_STACK_SIZE / sizeof(ULONG)];

typedef struct
{
  int64_t code_sum[2];
  float sum[2];
  float voltage_sum[2];
  uint16_t count[2];
} daq_loadcell_window_t;

static uint16_t daq_drain_ext_adc(daq_snapshot_t *snapshot,
                                  sd_raw_adc_record_t *records,
                                  uint16_t capacity,
                                  const daq_calibration_t *calibration,
                                  daq_loadcell_window_t *window)
{
  mcp3564r_sample_t sample = {
    .sample_valid = snapshot->ext_adc_sample_valid,
    .channel = snapshot->ext_adc_channel,
    .monotonic_ms = snapshot->ext_adc_monotonic_ms,
    .code = snapshot->ext_adc_code,
    .voltage_v = snapshot->ext_adc_voltage_v,
    .raw_value = snapshot->ext_adc_loadcell_kg1000,
  };
  uint16_t count = 0U;
  const uint64_t unix_now = telemetry_unix_ms();
  const uint64_t mono_now = telemetry_now_ms();
  *window = (daq_loadcell_window_t){0};

  for (;;)
  {
    if (sample.sample_valid != 0U && sample.channel < 2U && count < capacity)
    {
      const uint8_t channel = sample.channel;
      const float raw = sample.raw_value;
      if (records != NULL)
      {
        records[count].channel = channel;
        records[count].monotonic_ms = (uint32_t)sample.monotonic_ms;
        records[count].network_unix_ms = daq_sample_network_ms(unix_now, mono_now, sample.monotonic_ms);
        records[count].raw_adc_code = sample.code;
        records[count].raw_value = raw;
        records[count].calibrated_value = channel == 1U
            ? daq_calibration_apply_kg50(calibration, raw)
            : calibration->kg1000_slope * raw + calibration->kg1000_intercept;
      }
      if (sample.code != 0) g_daq_nonzero_raw_sample_count++;
      window->code_sum[channel] += sample.code;
      window->sum[channel] += raw;
      window->voltage_sum[channel] += sample.voltage_v;
      window->count[channel]++;
      count++;
    }
    if (count >= capacity || mcp3564r_pending_samples() == 0U ||
        mcp3564r_get_sample(&sample) != TX_SUCCESS || sample.sample_valid == 0U) break;
  }
  /* Legacy snapshot fields describe only the 1000 kg input. */
  snapshot->ext_adc_sample_valid = window->count[0] != 0U;
  if (window->count[0] != 0U)
  {
    snapshot->ext_adc_code = (int32_t)(window->code_sum[0] / window->count[0]);
    snapshot->ext_adc_loadcell_kg1000 = window->sum[0] / window->count[0];
    snapshot->ext_adc_voltage_v = window->voltage_sum[0] / window->count[0];
  }
  g_daq_raw_samples_drained_count += count;
  return count;
}

static void daq_store_snapshot_csv(const daq_snapshot_t *snapshot,
                                   const daq_calibration_t *calibration,
                                   const daq_loadcell_window_t *window)
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
  if (window->count[1] != 0U)
    (void)sd_card_enqueue_csv_row("mcp3564r_ch1_voltage_v", snapshot->monotonic_ms,
                                 window->voltage_sum[1] / window->count[1], calibration);
#else
  (void)snapshot;
  (void)calibration;
  (void)window;
#endif
}

static void daq_publish_loadcell(const daq_snapshot_t *snapshot, const daq_calibration_t *calibration)
{
  float loadcell_kg1000;

  if (snapshot->ext_adc_sample_valid == 0U)
  {
    return;
  }

  /* CAN carries the uncalibrated window average. GroundStation owns display
   * calibration; SD raw records separately retain calibrated values and the
   * coefficients. Never apply calibration twice along the network path. */
  loadcell_kg1000 = snapshot->ext_adc_loadcell_kg1000;

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

static void daq_publish_kg50(float raw, uint64_t monotonic_ms,
                             const daq_calibration_t *calibration)
{
#if (DISABLE_SD_CARD == 0U)
  if (sd_card_enqueue_csv_row("kg50_network", monotonic_ms, raw, calibration) == SD_CARD_STATUS_OK)
    g_daq_sd_network_row_ok_count++;
  else
    g_daq_sd_network_row_fail_count++;
#else
  (void)monotonic_ms;
  (void)calibration;
#endif
  if (log_telemetry_asynchronous(SEDS_DT_KG50, &raw, 1U, sizeof(raw)) == SEDS_OK)
    g_daq_kg50_publish_ok_count++;
  else
    g_daq_kg50_publish_fail_count++;
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
  daq_downsample_t downsample_kg50 = {0};
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
    daq_loadcell_window_t window;

#if (DISABLE_SD_CARD == 0U)
    const uint16_t raw_count = daq_drain_ext_adc(&snapshot,
                                                 raw_records,
                                                 DAQ_RAW_BATCH_MAX, &calibration, &window);
    if ((raw_count != 0U) &&
        (sd_card_enqueue_raw_adc_samples(raw_records, raw_count, &calibration) != SD_CARD_STATUS_OK))
    {
      g_daq_sd_raw_batch_drop_count++;
    }
#else
    (void)daq_drain_ext_adc(&snapshot, NULL, DAQ_RAW_BATCH_MAX, &calibration, &window);
#endif

    if (++slow_sensor_log_counter >= DAQ_SLOW_SENSOR_LOG_DIVIDER)
    {
      slow_sensor_log_counter = 0U;
      daq_store_snapshot_csv(&snapshot, &calibration, &window);
    }
    float filtered;
    if (daq_downsample_add(&downsample, snapshot.ext_adc_loadcell_kg1000,
                           window.count[0], (uint32_t)snapshot.monotonic_ms,
                           DAQ_BROADCAST_PERIOD_MS, &filtered))
    {
      /* SD kg1000_network and SEDSNet receive this identical filtered value.
       * Raw records above retain individual, unfiltered conversions. */
      daq_snapshot_t published = snapshot;
      published.ext_adc_loadcell_kg1000 = filtered;
      published.ext_adc_sample_valid = 1U;
      daq_publish_loadcell(&published, &calibration);
    }

    const float kg50_average = window.count[1] != 0U ? window.sum[1] / window.count[1] : 0.0f;
    if (daq_downsample_add(&downsample_kg50, kg50_average, window.count[1],
                           (uint32_t)snapshot.monotonic_ms, DAQ_BROADCAST_PERIOD_MS, &filtered))
      daq_publish_kg50(filtered, snapshot.monotonic_ms, &calibration);

    (void)daq_board_ext_adc_start_dma();

    if ((power_loss_latched == 0U) && (snapshot.input_voltage_v <= DAQ_INPUT_VOLTAGE_LOW_V))
    {
      power_loss_latched = 1U;
#if (DISABLE_SD_CARD == 0U)
      (void)sd_card_notify_power_loss();
#endif
    }
    else if (snapshot.input_voltage_v >= DAQ_INPUT_VOLTAGE_RECOVERED_V)
    {
      power_loss_latched = 0U; /* Re-arm after recovery, with hysteresis. */
    }

    /* A power-loss notification asks the SD worker to flush. It must not
     * permanently throttle acquisition/network reporting to 10 Hz after
     * one low-voltage observation, including a startup transient. */

    /* Include acquisition/publish work in the configured period. Sleeping for a
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
