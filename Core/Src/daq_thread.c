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
#include "daq_filter.h"
#include <string.h>
#include <math.h>
TX_THREAD daq_thread;

#define DAQ_THREAD_STACK_SIZE (32U * 1024U)
#define DAQ_SAMPLE_PERIOD_MS DAQ_ACQUISITION_PERIOD_MS
#define DAQ_SAMPLE_PERIOD_TICKS ((DAQ_SAMPLE_PERIOD_MS * TX_TIMER_TICKS_PER_SECOND) / 1000U)
#define DAQ_INPUT_VOLTAGE_LOW_V 8.5f
#define DAQ_INPUT_VOLTAGE_RECOVERED_V 9.0f
#define DAQ_ENABLE_DUMMY_CAN_TELEMETRY 0U
#define DAQ_RAW_BATCH_MAX DAQ_RAW_BATCH_CAPACITY

volatile uint32_t g_daq_sample_ok_count = 0U;
volatile uint32_t g_daq_sample_fail_count = 0U;
volatile uint32_t g_daq_init_fail_count = 0U;
volatile uint32_t g_daq_loadcell_publish_ok_count = 0U;
volatile uint32_t g_daq_loadcell_publish_fail_count = 0U;
volatile uint32_t g_daq_kg50_publish_ok_count = 0U;
volatile uint32_t g_daq_kg50_publish_fail_count = 0U;
volatile uint32_t g_daq_temperature_publish_ok_count = 0U;
volatile uint32_t g_daq_temperature_publish_fail_count = 0U;
volatile uint32_t g_daq_raw_samples_drained_count = 0U;
volatile uint32_t g_daq_nonzero_raw_sample_count = 0U;
volatile uint32_t g_daq_sd_raw_batch_drop_count = 0U;
volatile uint32_t g_daq_sd_network_row_ok_count = 0U;
volatile uint32_t g_daq_sd_network_row_fail_count = 0U;
volatile uint32_t g_daq_sample_overrun_count = 0U;

/* Wall-time profiling, including preemption and mutex waits. Stages are board
 * sampling, raw drain/enqueue, publishing, and ADC service. Read deltas of the
 * totals/counts to locate rate limits without stopping acquisition. */
volatile uint32_t g_daq_stage_ticks[4] = {0};
volatile uint32_t g_daq_stage_max_ticks[4] = {0};
volatile uint32_t g_daq_stage_count[4] = {0};

static ULONG daq_profile_stage(unsigned stage, ULONG started)
{
  const ULONG now = tx_time_get();
  const uint32_t elapsed = (uint32_t)(now - started);
  g_daq_stage_ticks[stage] += elapsed;
  if (elapsed > g_daq_stage_max_ticks[stage])
    g_daq_stage_max_ticks[stage] = elapsed;
  g_daq_stage_count[stage]++;
  return now;
}

static ULONG g_daq_thread_stack[DAQ_THREAD_STACK_SIZE / sizeof(ULONG)];

typedef struct
{
  int64_t code_sum[8];
  float sum[8];
  float voltage_sum[8];
  uint16_t count[8];
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
    .temperature_c = snapshot->ext_adc_temp_c,
    .temperature_code = snapshot->ext_adc_temp_code,
  };
  static daq_filter_t filters[2];
  static daq_calibration_t filter_calibration;
  if (memcmp(&filter_calibration, calibration, sizeof(*calibration)) != 0)
  {
    memset(filters, 0, sizeof(filters));
    filter_calibration = *calibration;
  }
  uint16_t count = 0U;
  const uint64_t unix_now = telemetry_unix_ms();
  const uint64_t mono_now = telemetry_now_ms();
  *window = (daq_loadcell_window_t){0};

  for (;;)
  {
    if (sample.sample_valid != 0U && sample.channel < 8U && count < capacity)
    {
      const uint8_t channel = sample.channel;
      const float raw = sample.raw_value;
      const float corrected = channel < 2U ? daq_filter_add(&filters[channel],
          daq_calibration_temperature_raw(calibration, channel, raw, sample.temperature_c),
          (uint32_t)sample.monotonic_ms, calibration->filter_tau_ms[channel]) : mcp3564r_connector_voltage(channel, sample.voltage_v);
      if (records != NULL)
      {
        records[count].channel = channel;
        records[count].monotonic_ms = (uint32_t)sample.monotonic_ms;
        records[count].network_unix_ms = daq_sample_network_ms(unix_now, mono_now, sample.monotonic_ms);
        records[count].raw_adc_code = sample.code;
        records[count].raw_value = raw;
        records[count].adc_temperature_c = sample.temperature_c;
        records[count].adc_temperature_code = sample.temperature_code;
        records[count].calibrated_value = channel > 1U ? corrected : channel == 1U
            ? daq_calibration_apply_kg50(calibration, corrected)
            : calibration->kg1000_slope * corrected + calibration->kg1000_intercept;
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

volatile uint32_t g_daq_sd_snapshot_row_drop_count;

static void daq_store_snapshot_csv(const daq_snapshot_t *snapshot,
                                   const daq_calibration_t *calibration,
                                   const daq_loadcell_window_t *window)
{
#if (DISABLE_SD_CARD == 0U)
  /* Format one slow-channel row per worker call, retaining the capture time.
   * Formatting the whole snapshot here stalls the 2 ms acquisition loop. */
  static const char *const names[] = {
    "input_voltage_v", "input_current_a", "adc1_aux_v",
    "analog_in_1_v", "analog_in_2_v", "analog_in_3_v", "analog_in_4_v",
    "analog_in_5_v", "analog_in_6_v", "analog_in_7_v", "analog_in_8_v",
    "isense1_v", "isense2_v", "isense1_a", "isense2_a", "vmon_v", "imon_v",
    "analog_out_1_v", "analog_out_2_v", "mcp3564r_code", "mcp3564r_voltage_v",
    "kg1000_raw_window_average", "mcp3564r_temp_c", "mcp3564r_ch1_voltage_v"
  };
  static float values[sizeof(names) / sizeof(names[0])];
  static unsigned next = sizeof(names) / sizeof(names[0]);
  static uint64_t captured_ms;
  static daq_calibration_t captured_calibration;
  if (snapshot->analog_sample_fresh)
  {
    g_daq_sd_snapshot_row_drop_count += sizeof(names) / sizeof(names[0]) - next;
    values[0] = snapshot->input_voltage_v;
    values[1] = snapshot->input_current_a;
    values[2] = snapshot->adc1_aux_v;
    memcpy(&values[3], snapshot->analog_inputs_v, sizeof(snapshot->analog_inputs_v));
    memcpy(&values[11], snapshot->current_sense_v, sizeof(snapshot->current_sense_v));
    memcpy(&values[13], snapshot->current_sense_a, sizeof(snapshot->current_sense_a));
    memcpy(&values[15], snapshot->power_monitor_v, sizeof(snapshot->power_monitor_v));
    memcpy(&values[17], snapshot->analog_outputs_v, sizeof(snapshot->analog_outputs_v));
    values[19] = snapshot->ext_adc_sample_valid ? (float)snapshot->ext_adc_code : NAN;
    values[20] = snapshot->ext_adc_sample_valid ? snapshot->ext_adc_voltage_v : NAN;
    values[21] = snapshot->ext_adc_sample_valid ? snapshot->ext_adc_loadcell_kg1000 : NAN;
    values[22] = snapshot->ext_adc_temp_c;
    values[23] = window->count[1] ? window->voltage_sum[1] / window->count[1] : NAN;
    captured_ms = snapshot->monotonic_ms;
    captured_calibration = *calibration;
    next = 0U;
  }
  if (next < sizeof(names) / sizeof(names[0]))
  {
    (void)sd_card_enqueue_csv_row(names[next], captured_ms, values[next], &captured_calibration);
    ++next;
  }
#else
  (void)snapshot;
  (void)calibration;
  (void)window;
#endif
}

volatile uint32_t g_daq_analog_publish_ok_count;
volatile uint32_t g_daq_analog_publish_fail_count;

static void daq_publish_analog_packet(SedsDataType type, const float *values, size_t count)
{
  if (log_telemetry_asynchronous(type, values, count, sizeof(float)) == SEDS_OK)
    g_daq_analog_publish_ok_count++;
  else
    g_daq_analog_publish_fail_count++;
}

static void daq_publish_analog(const daq_snapshot_t *snapshot, const daq_loadcell_window_t *window)
{
  static float auxiliary[6];
  static uint32_t updated[6];
  static uint8_t valid;
  static uint32_t last_report;
  const uint32_t now = (uint32_t)snapshot->monotonic_ms;
  for (unsigned i = 0; i < 6U; ++i)
  {
    if (window->count[i + 2U] != 0U)
    {
      auxiliary[i] = mcp3564r_connector_voltage(i + 2U, window->voltage_sum[i + 2U] / window->count[i + 2U]);
      updated[i] = now;
      valid |= (uint8_t)(1U << i);
    }
  }
  if ((uint32_t)(now - last_report) >= DAQ_ANALOG_REPORT_PERIOD_MS)
  {
    last_report = now;
    float values[6];
    for (unsigned i = 0; i < 6U; ++i)
      values[i] = (valid & (1U << i)) && (uint32_t)(now - updated[i]) <= 3U * MCP3564R_AUX_INTERVAL_MS
          ? auxiliary[i] : NAN;
    daq_publish_analog_packet(SEDS_DT_DAQ_SDADC_VOLTAGES, values, 6U);
  }
  if (snapshot->analog_sample_fresh)
  {
    const float current[] = {snapshot->current_sense_v[0], snapshot->current_sense_v[1],
                            snapshot->current_sense_a[0], snapshot->current_sense_a[1]};
    const float power[] = {snapshot->power_monitor_v[0], snapshot->power_monitor_v[1],
                          snapshot->input_voltage_v, snapshot->input_current_a};
    daq_publish_analog_packet(SEDS_DT_DAQ_SAR_VOLTAGES, snapshot->analog_inputs_v, 8U);
    daq_publish_analog_packet(SEDS_DT_DAQ_CURRENT_SENSE, current, 4U);
    daq_publish_analog_packet(SEDS_DT_DAQ_POWER_MONITOR, power, 4U);
  }
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

static void daq_enqueue_analog(const daq_snapshot_t *snapshot,
                               const daq_calibration_t *calibration,
                               const daq_loadcell_window_t *window);

void daq_thread_entry(ULONG initial_input)
{
  daq_snapshot_t snapshot;
  uint8_t power_loss_latched = 0U;
  uint8_t daq_ready = 1U;
  uint32_t last_temperature_report_ms = 0U;
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

    ULONG stage_started = tx_time_get();
    if (daq_board_sample(&snapshot) != TX_SUCCESS)
    {
      g_daq_sample_fail_count++;
      tx_thread_sleep(DAQ_SAMPLE_PERIOD_TICKS);
      continue;
    }

    stage_started = daq_profile_stage(0U, stage_started);
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

    stage_started = daq_profile_stage(1U, stage_started);
    const uint32_t report_ms = (uint32_t)snapshot.monotonic_ms;
    if ((uint32_t)(report_ms - last_temperature_report_ms) >= DAQ_TEMPERATURE_REPORT_PERIOD_MS)
    {
      last_temperature_report_ms = report_ms;
      if (log_telemetry_asynchronous(SEDS_DT_DAQ_ADC_TEMPERATURE, &snapshot.ext_adc_temp_c, 1U, sizeof(float)) == SEDS_OK)
        g_daq_temperature_publish_ok_count++;
      else
        g_daq_temperature_publish_fail_count++;
    }
    daq_enqueue_analog(&snapshot, &calibration, &window);
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

    stage_started = daq_profile_stage(2U, stage_started);
    (void)daq_board_ext_adc_start_dma();
    (void)daq_profile_stage(3U, stage_started);

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

/* Bound slow telemetry work independently of the load-cell drain. */
typedef struct {
  daq_snapshot_t snapshot;
  daq_calibration_t calibration;
  daq_loadcell_window_t window;
} daq_analog_work_t;
static TX_THREAD g_analog_thread;
static TX_QUEUE g_analog_queue;
static TX_BLOCK_POOL g_analog_pool;
static ULONG g_analog_stack[8192U / sizeof(ULONG)];
static ULONG g_analog_queue_storage[3];
static ULONG g_analog_pool_storage[(3U * (sizeof(daq_analog_work_t) + sizeof(void *)) + sizeof(ULONG) - 1U) / sizeof(ULONG)];
volatile uint32_t g_daq_analog_work_drop_count;

static void daq_enqueue_analog(const daq_snapshot_t *snapshot,
                               const daq_calibration_t *calibration,
                               const daq_loadcell_window_t *window)
{
  static daq_loadcell_window_t accumulated;
  for (unsigned i = 2U; i < 8U; ++i) {
    accumulated.count[i] += window->count[i];
    accumulated.voltage_sum[i] += window->voltage_sum[i];
  }
  if (!snapshot->analog_sample_fresh) return;
  daq_analog_work_t *work;
  if (tx_block_allocate(&g_analog_pool, (VOID **)&work, TX_NO_WAIT) == TX_SUCCESS) {
    work->snapshot = *snapshot;
    work->calibration = *calibration;
    work->window = accumulated;
    work->window.count[1] = window->count[1];
    work->window.voltage_sum[1] = window->voltage_sum[1];
    const ULONG message = (ULONG)(uintptr_t)work;
    if (tx_queue_send(&g_analog_queue, (VOID *)&message, TX_NO_WAIT) != TX_SUCCESS) {
      (void)tx_block_release(work);
      g_daq_analog_work_drop_count++;
    }
  } else g_daq_analog_work_drop_count++;
  memset(&accumulated, 0, sizeof(accumulated));
}

static void daq_analog_thread_entry(ULONG argument)
{
  (void)argument;
  for (;;) {
    ULONG message;
    if (tx_queue_receive(&g_analog_queue, &message, TX_WAIT_FOREVER) != TX_SUCCESS) continue;
    daq_analog_work_t *work = (daq_analog_work_t *)(uintptr_t)message;
    daq_publish_analog(&work->snapshot, &work->window);
    /* Each call emits one row from the same captured snapshot. */
    for (unsigned i = 0; i < 24U; ++i) {
      daq_store_snapshot_csv(&work->snapshot, &work->calibration, &work->window);
      work->snapshot.analog_sample_fresh = 0U;
    }
    (void)tx_block_release(work);
  }
}

UINT create_daq_thread(void)
{
  UINT status = tx_block_pool_create(&g_analog_pool, "Analog snapshots", sizeof(daq_analog_work_t),
                                     g_analog_pool_storage, sizeof(g_analog_pool_storage));
  if (status != TX_SUCCESS) return status;
  status = tx_queue_create(&g_analog_queue, "Analog snapshots", TX_1_ULONG,
                           g_analog_queue_storage, sizeof(g_analog_queue_storage));
  if (status != TX_SUCCESS) return status;
  status = tx_thread_create(&g_analog_thread, "Analog reporting", daq_analog_thread_entry, 0U,
                            g_analog_stack, sizeof(g_analog_stack), DAQ_IO_THREAD_PRIORITY + 1U,
                            DAQ_IO_THREAD_PRIORITY + 1U, TX_NO_TIME_SLICE, TX_AUTO_START);
  if (status != TX_SUCCESS) return status;
  return tx_thread_create(&daq_thread,
                          "DAQ Thread",
                          daq_thread_entry,
                          0U,
                          g_daq_thread_stack,
                          sizeof(g_daq_thread_stack),
                          DAQ_IO_THREAD_PRIORITY,
                          DAQ_IO_THREAD_PRIORITY,
                          (TX_TIMER_TICKS_PER_SECOND * DAQ_IO_THREAD_SLICE_MS + 999U) / 1000U,
                          TX_AUTO_START);
}
